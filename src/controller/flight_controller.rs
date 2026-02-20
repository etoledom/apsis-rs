use crate::{
    controller::{
        altitude_controller::AltitudeController, attitude_controller::AttitudeController,
        rate_controller::RateController, velocity_ned_controller::VelocityNedController,
    },
    simulator::{
        inputs::Inputs,
        state::State,
        types::{
            acceleration_3d::WorldFrameAcceleration,
            quaternion::Quaternion,
            vec3::{CrossMult, DotMult, Vec3},
            velocity_ned::VelocityNED,
        },
    },
    units::{
        angles::Radians,
        consts::G_EARTH,
        units::{Meters, Seconds},
    },
};

#[derive(Default)]
pub struct AirframeLimits {
    pub max_pitch: Radians,
    pub max_roll: Radians,
}

pub struct FlightController {
    altitude_controller: AltitudeController,
    velocity_ned_controller: VelocityNedController,
    attitude_controller: AttitudeController,
    rate_controller: RateController,
    limits: AirframeLimits,
}

impl FlightController {
    pub fn new(
        altitude_target: Meters,
        velocity_target: VelocityNED,
        limits: AirframeLimits,
    ) -> Self {
        Self {
            altitude_controller: AltitudeController::new(altitude_target),
            velocity_ned_controller: VelocityNedController::new(velocity_target),
            attitude_controller: AttitudeController::new(),
            rate_controller: RateController::new(),
            limits,
        }
    }

    // Cascade:
    // velocity_target (from user input) -> acceleration_target
    // acceleration_target -> quaternion_target
    // quaternion_target -> angular_rates
    // angular_rates -> drone inputs (pitch - roll - yaw - thrust)
    pub fn update(&mut self, telemetry: &State, dt: Seconds) -> Inputs {
        let throttle = self.altitude_controller.update(telemetry, dt);
        let v_ned = telemetry.velocity_ned;

        let acc_target = self.velocity_ned_controller.update(v_ned, dt);

        let q_target = Self::q_target_from_acceleration(acc_target, telemetry);

        let angular_rates_target =
            self.attitude_controller
                .update(q_target, telemetry.attitude, dt);

        let (roll, pitch, yaw) =
            self.rate_controller
                .update(angular_rates_target, telemetry.angular_velocity_body, dt);

        // println!("acc_target north: {}", acc_target.north().0);
        // println!("q_target pitch: {}", q_target.pitch().to_degrees().0);
        // println!(
        //     "q_current pitch: {}",
        //     telemetry.attitude.pitch().to_degrees().0
        // );

        Inputs {
            throttle,
            pitch,
            roll,
            yaw,
        }
    }

    pub fn q_target_from_acceleration(
        acc_target: WorldFrameAcceleration,
        telemetry: &State,
    ) -> Quaternion {
        let gravity = WorldFrameAcceleration::from_down(G_EARTH);
        let thrust = acc_target + gravity;
        let thrust_dir = thrust.normalized();
        let z_body_desired = Vec3 {
            x: -thrust_dir.x,
            y: -thrust_dir.y,
            z: thrust_dir.z, // keep Z positive (down)
        };

        let yaw = telemetry.attitude.yaw();
        let x_body_desired = Vec3 {
            x: yaw.cos(),
            y: yaw.sin(),
            z: 0.0,
        }
        .normalized();

        let y_body_desired = z_body_desired.cross(x_body_desired).normalized();
        let x_body_desired =
            (x_body_desired - z_body_desired * x_body_desired.dot(z_body_desired)).normalized();

        let matrix = [
            [x_body_desired.x, y_body_desired.x, z_body_desired.x],
            [x_body_desired.y, y_body_desired.y, z_body_desired.y],
            [x_body_desired.z, y_body_desired.z, z_body_desired.z],
        ];

        Quaternion::from_rotation_matrix(matrix)
    }

    pub fn set_target_velocity(&mut self, velocity: VelocityNED) {
        self.velocity_ned_controller.target = velocity;
    }

    pub fn get_target_velocity(&self) -> VelocityNED {
        self.velocity_ned_controller.target
    }
}

#[cfg(test)]
mod flight_controller_tests {
    use approx::assert_relative_eq;

    use crate::{
        simulator::types::{angular_velocity_3d::AngularVelocity3D, position_ned::PositionNed},
        units::{
            acceleration::AccelerationLiteral,
            angles::DegreesLiteral,
            units::{MettersLiteral, SecondsLiteral, VelocityLiteral},
        },
    };

    use super::*;

    fn make_controller() -> FlightController {
        FlightController::new(
            Meters(10.0),        // target altitude
            VelocityNED::zero(), // hover
            AirframeLimits::default(),
        )
    }

    fn nominal_state() -> State {
        State {
            position_ned: PositionNed::from_altitude_ned(-10.meters()), // at target
            ..Default::default()
        }
    }

    #[test]
    fn nominal_state_produces_stable_inputs() {
        let mut ctrl = make_controller();
        let inputs = ctrl.update(&nominal_state(), 0.1.seconds());

        assert!((inputs.roll.get()).abs() < 0.1, "roll should be near zero");
        assert!(
            (inputs.pitch.get()).abs() < 0.1,
            "pitch should be near zero"
        );
        assert!((inputs.yaw.get()).abs() < 0.1, "yaw should be near zero");
        // throttle near hover — exact value depends on your airframe limits
        assert!(inputs.throttle.get() >= 0.0);
        assert!(inputs.throttle.get() <= 1.0);
    }

    #[test]
    fn below_target_altitude_increases_throttle() {
        let mut ctrl = make_controller();

        let mut low_state = nominal_state();
        low_state.position_ned = PositionNed::from_altitude_ned(-5.meters()); // 5m below target

        let inputs = ctrl.update(&low_state, Seconds(0.1));

        assert!(
            inputs.throttle.get() > 0.5,
            "should increase throttle when below target"
        );
    }

    #[test]
    fn above_target_altitude_decreases_throttle() {
        let mut ctrl = make_controller();

        let mut high_state = nominal_state();
        high_state.position_ned = PositionNed::from_altitude_ned(-15.meters()); // 5m above target

        let inputs = ctrl.update(&high_state, Seconds(0.1));
        assert!(
            inputs.throttle.get() < 0.5,
            "should decrease throttle when above target"
        );
    }

    #[test]
    fn rolled_drone_gets_corrective_roll_input() {
        let mut ctrl = make_controller();

        let angle = 17.degrees().to_radians();
        let mut rolled_state = nominal_state();
        rolled_state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: (angle / 2.0).sin(),
            y: 0.0,
            z: 0.0,
        };

        let inputs = ctrl.update(&rolled_state, Seconds(0.1));
        assert!(
            inputs.roll.get() < 0.0,
            "should correct against positive roll"
        );
    }

    #[test]
    fn inputs_always_within_valid_range() {
        let mut ctrl = make_controller();

        // Extreme state — far from target, high rates
        let extreme_state = State {
            altitude: Meters(0.0),
            velocity_ned: VelocityNED::new(10.mps(), 10.mps(), 10.mps()),
            attitude: Quaternion {
                w: 0.5,
                x: 0.5,
                y: 0.5,
                z: 0.5,
            }, // large tilt
            angular_velocity_body: AngularVelocity3D::new(5.0, 5.0, 5.0),
            ..nominal_state()
        };

        let inputs = ctrl.update(&extreme_state, Seconds(0.1));
        assert!(inputs.throttle.get() >= 0.0 && inputs.throttle.get() <= 1.0);
        assert!(inputs.roll.get() >= -1.0 && inputs.roll.get() <= 1.0);
        assert!(inputs.pitch.get() >= -1.0 && inputs.pitch.get() <= 1.0);
        assert!(inputs.yaw.get() >= -1.0 && inputs.yaw.get() <= 1.0);
    }

    #[test]
    fn larger_error_produces_larger_response() {
        let mut ctrl_small = make_controller();
        let mut ctrl_large = make_controller();

        let mut small_error = nominal_state();
        small_error.position_ned = PositionNed::from_altitude_ned(-Meters(9.0)); // 1m below

        let mut large_error = nominal_state();
        large_error.position_ned = PositionNed::from_altitude_ned(-Meters(2.0)); // 5m below

        let inputs_small = ctrl_small.update(&small_error, Seconds(0.1));
        let inputs_large = ctrl_large.update(&large_error, Seconds(0.1));

        assert!(
            inputs_large.throttle.get() > inputs_small.throttle.get(),
            "larger altitude error should produce larger throttle response"
        );
    }

    #[test]
    fn pitched_drone_gets_corrective_pitch_input() {
        let mut ctrl = make_controller();

        let angle = 17.degrees().to_radians();
        let mut pitched_state = nominal_state();
        pitched_state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };

        let inputs = ctrl.update(&pitched_state, 0.1.seconds());
        assert!(
            inputs.pitch.get() < 0.0,
            "should correct against positive pitch"
        );
    }

    #[test]
    fn pitched_drone_gets_corrective_pitch_input_trace() {
        let mut ctrl = make_controller();

        let angle = Radians(0.3);
        let mut pitched_state = nominal_state();
        pitched_state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };

        let inputs = ctrl.update(&pitched_state, Seconds(0.1));

        assert!(inputs.pitch.get() < 0.0);
    }

    #[test]
    fn forward_velocity_target_produces_negative_pitch_input() {
        let mut ctrl = FlightController::new(
            Meters(-10.0),
            VelocityNED::new(1.mps(), 0.mps(), 0.mps()), // target moving north
            AirframeLimits::default(),
        );

        let inputs = ctrl.update(&nominal_state(), Seconds(0.1));

        assert!(
            inputs.pitch.get() < 0.0,
            "forward velocity target should produce forward pitch"
        );
    }

    #[test]
    fn pure_north_acceleration_produces_nose_down_pitch() {
        let state = nominal_state();
        let acc = WorldFrameAcceleration::new(5.mps2(), 0.mps2(), 0.mps2());
        let q_target = FlightController::q_target_from_acceleration(acc, &state);

        assert!(
            q_target.y < 0.0,
            "north acc should produce negative pitch (aerospace convention), got {}",
            q_target.pitch().to_degrees().0
        );
    }

    #[test]
    fn pure_north_acceleration_produces_pitch() {
        let state = nominal_state();
        let acc = WorldFrameAcceleration::new(5.mps2(), 0.mps2(), 0.mps2());

        let q_target = FlightController::q_target_from_acceleration(acc, &state);

        // North acceleration should produce positive pitch, no roll
        assert!(
            q_target.pitch().to_degrees().0 < 0.0,
            "north acc should produce negative pitch"
        );
        assert!(
            q_target.roll().to_degrees().0.abs() < 1e-6,
            "north acc should not affect roll"
        );
    }

    #[test]
    fn pure_east_acceleration_produces_roll() {
        // Pure east acceleration should produce a roll (rotation around X axis)
        let state = nominal_state(); // level, yaw=0
        let acc = WorldFrameAcceleration::new(0.mps2(), 5.mps2(), 0.mps2());

        let q_target = FlightController::q_target_from_acceleration(acc, &state);

        // East acceleration should produce positive roll, no pitch, no yaw
        assert!(
            q_target.roll().to_degrees().0 > 0.0,
            "east acc should produce positive roll"
        );
        assert!(
            q_target.pitch().to_degrees().0.abs() < 1e-6,
            "east acc should not affect pitch"
        );
    }

    #[test]
    fn level_drone_with_positive_pitch_target_produces_positive_pitch_error() {
        // q_current = level (identity)
        // q_target = positive pitch (nose up, aerospace convention)
        // q_error should be positive pitch → command nose down
        let q_current = Quaternion::identity();
        let angle = 0.3_f64;
        let q_target = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };

        let q_error = q_current.conjugate() * q_target;
        let q_error = if q_error.w < 0.0 { -q_error } else { q_error };

        assert!(
            q_error.y > 0.0,
            "positive pitch target from level should give positive pitch error"
        );
    }

    #[test]
    fn positive_pitched_drone_with_level_target_produces_negative_pitch_error() {
        // q_current = positive pitch (nose down)
        // q_target = level
        // q_error should be negative → command nose up to recover
        let angle = 0.3_f64;
        let q_current = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };
        let q_target = Quaternion::identity();

        let q_error = q_current.conjugate() * q_target;
        let q_error = if q_error.w < 0.0 { -q_error } else { q_error };

        assert!(
            q_error.y < 0.0,
            "positive pitched drone with level target should give negative pitch error"
        );
    }

    #[test]
    fn acc_target_6_produces_31_degree_pitch() {
        let state = nominal_state();
        let acc = WorldFrameAcceleration::new(6.0.mps2(), 0.mps2(), 0.mps2());
        let q_target = FlightController::q_target_from_acceleration(acc, &state);

        // atan(6 / 9.81) = 31.5°, nose down = negative in aerospace
        assert_relative_eq!(q_target.pitch().to_degrees().0, -31.5, epsilon = 0.5);
    }
}
