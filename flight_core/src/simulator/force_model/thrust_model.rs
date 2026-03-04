use crate::{
    simulator::{
        drone::Drone,
        force_model::{context::Context, force_model::ForceModel},
        types::{acceleration_3d::WorldFrameAcceleration, quaternion::Quaternion},
    },
    units::{acceleration::Acceleration, units::Seconds},
};

pub struct ThrustModel;

impl<'a, Vehicle: Drone> ForceModel<Vehicle> for ThrustModel {
    fn acceleration_contribution(
        &self,
        ctx: &Context<Vehicle>,
        _: Seconds,
    ) -> WorldFrameAcceleration {
        if ctx.state.battery_pct <= 0.0 {
            // No thrust without battery.
            return WorldFrameAcceleration::zero();
        }

        let inputs = ctx.inputs;

        let base_acceleration = inputs.throttle.get() * ctx.vehicle.max_thrust_acceleration();
        // Body frame thrust is always down, independent of the body rotation.
        let thrust_body_frame = Quaternion::pure(0.0, 0.0, -base_acceleration.raw());
        let conjugate = ctx.state.attitude.conjugate();
        let thrust_world_frame = ctx.state.attitude * thrust_body_frame * conjugate;

        // x -> Forward
        // y -> Sides
        // z -> Vertical
        return WorldFrameAcceleration::new(
            Acceleration(thrust_world_frame.x),
            Acceleration(thrust_world_frame.y),
            Acceleration(thrust_world_frame.z),
        );
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_4, FRAC_PI_8};

    use approx::assert_relative_eq;

    use crate::{
        simulator::{
            default_drone::DefaultDrone, inputs::Inputs, state::State, types::throttle::Throttle,
        },
        units::{angles::DegreesLiteral, consts::G_EARTH, units::SecondsLiteral},
    };

    use super::*;

    #[test]
    fn identity() {
        let model = ThrustModel {};
        let drone = DefaultDrone {};
        let state = State::default();
        let inputs = Inputs {
            throttle: Throttle::max(),
            ..Default::default()
        };
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };

        let acceleration = model.acceleration_contribution(&ctx, 1.seconds());

        assert_eq!(acceleration.north().raw(), 0.0);
        assert_eq!(acceleration.east().raw(), 0.0);
        assert_eq!(
            acceleration.down().raw(),
            -drone.max_thrust_acceleration().raw()
        );
    }

    #[test]
    fn roll_rotation() {
        let model = ThrustModel {};
        let drone = DefaultDrone {};
        let mut state = State::default();

        // 90° roll
        state.attitude = Quaternion {
            w: FRAC_PI_4.cos(),
            x: FRAC_PI_4.sin(),
            y: 0.0,
            z: 0.0,
        };
        let inputs = Inputs {
            throttle: Throttle::max(),
            ..Default::default()
        };
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };

        let acceleration = model.acceleration_contribution(&ctx, 1.seconds());
        assert_eq!(acceleration.north().raw(), 0.0);
        assert_eq!(
            acceleration.east().raw(),
            drone.max_thrust_acceleration().raw()
        );
        assert_relative_eq!(acceleration.down().raw(), 0.0, epsilon = 1e-4);
    }

    #[test]
    fn pitch_rotation() {
        let model = ThrustModel {};
        let drone = DefaultDrone {};
        let mut state = State::default();

        // 90° pitch
        state.attitude = Quaternion {
            w: FRAC_PI_4.cos(),
            x: 0.0,
            y: FRAC_PI_4.sin(),
            z: 0.0,
        };
        let inputs = Inputs {
            throttle: Throttle::max(),
            ..Default::default()
        };
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };

        let acceleration = model.acceleration_contribution(&ctx, 1.seconds());
        assert_eq!(
            acceleration.north().raw(),
            -drone.max_thrust_acceleration().raw()
        );
        assert_eq!(acceleration.east().raw(), 0.0);
        assert_relative_eq!(acceleration.down().raw(), 0.0, epsilon = 1e-4);
    }

    #[test]
    fn combined_rotation() {
        let model = ThrustModel {};
        let drone = DefaultDrone {};
        let mut state = State::default();

        // 45° pitch
        let quaternion_pitch = Quaternion {
            w: FRAC_PI_8.cos(),
            x: 0.0,
            y: FRAC_PI_8.sin(),
            z: 0.0,
        };

        // 45° roll
        let quaternion_roll = Quaternion {
            w: FRAC_PI_8.cos(),
            x: FRAC_PI_8.sin(),
            y: 0.0,
            z: 0.0,
        };

        state.attitude = quaternion_roll * quaternion_pitch;

        let inputs = Inputs {
            throttle: Throttle::max(),
            ..Default::default()
        };
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };

        let max_thrust = drone.max_thrust_acceleration();
        let expected_north = max_thrust * 45.degrees().to_radians().sin();
        let expected_east = expected_north * 45.degrees().to_radians().cos();

        let acceleration = model.acceleration_contribution(&ctx, 1.seconds());
        assert_relative_eq!(
            acceleration.north().raw(),
            -expected_north.raw(),
            epsilon = 1e-2
        );
        assert_relative_eq!(
            acceleration.east().raw(),
            expected_east.raw(),
            epsilon = 1e-2
        );
        assert_relative_eq!(acceleration.down().raw(), -G_EARTH.raw(), epsilon = 1e-2);
        // total magnitude is equal to the max thrust
        assert_relative_eq!(
            acceleration.norm().raw(),
            drone.max_thrust_acceleration().raw()
        );
    }

    #[test]
    fn quaternion_rotation_pitch_direction() {
        // A positive pitch quaternion (nose down, positive Y)
        let angle = 17.0.degrees().to_radians();
        let q = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: -(angle / 2.0).sin(),
            z: 0.0,
        };

        // Thrust pointing up in body frame (negative Z in FRD)
        let v = Quaternion::pure(0.0, 0.0, -1.0);

        // Rotate body to world
        let result = q * v * q.conjugate();

        // Nose down + upward thrust should give positive north component
        assert!(
            result.x > 0.0,
            "positive pitch should tilt thrust northward"
        );
    }

    #[test]
    fn nose_down_pitch_produces_north_thrust() {
        let model = ThrustModel {};
        let drone = DefaultDrone {};
        let mut state = State::default();
        // Negative pitch = nose down in aerospace
        let angle = 17.0.degrees().to_radians();
        state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: -(angle / 2.0).sin(), // negative Y = nose down
            z: 0.0,
        };
        let inputs = Inputs {
            throttle: Throttle::max(),
            ..Default::default()
        };
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };
        let acc = model.acceleration_contribution(&ctx, 1.0.seconds());
        println!("north acc: {}", acc.north().0);
        assert!(
            acc.north().0 > 0.0,
            "nose down should produce positive north acceleration"
        );
    }
}
