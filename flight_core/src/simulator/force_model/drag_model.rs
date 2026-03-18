use primitives::{
    frames::{AccelerationFrd, AccelerationNed, VelocityNed},
    traits::UnitsArithmetics,
    units::{AccelerationLiteral, Seconds, VelocityLiteral},
};

use crate::simulator::{
    drone::Drone,
    force_model::{context::Context, force_model::ForceModel},
};

pub struct DragModel;

impl DragModel {
    pub fn new() -> Self {
        DragModel {}
    }
}

impl<Vehicle: Drone> ForceModel<Vehicle> for DragModel {
    fn acceleration_contribution<'a>(
        &self,
        ctx: &Context<'a, Vehicle>,
        _: Seconds,
    ) -> AccelerationNed {
        // a_drag [m/s2] = -|v|*v*k where k -> drag coef [1/m]
        let velocity_ned = ctx.state.velocity_ned;
        let drag = ctx.vehicle.drag_coefficient();

        // - Calculate horizontal drag in body frame
        // - Rotate horizontal drag back to world frame
        let vel_h_body = VelocityNed::new(velocity_ned.north(), velocity_ned.east(), 0.mps())
            .to_frd(&ctx.state.attitude);
        let drag_h_world = AccelerationFrd::new(
            -(vel_h_body.forward() * vel_h_body.forward().abs() * drag.forward.value()),
            -(vel_h_body.right() * vel_h_body.right().abs() * drag.horizontal.value()),
            0.mps2(),
        )
        .to_ned(&ctx.state.attitude);

        // - Calculate vertical drag in world frame
        let drag_v_world =
            -(velocity_ned.down() * velocity_ned.down().abs() * drag.vertical.value());

        // - Mix all three axis back
        AccelerationNed::new(drag_h_world.north(), drag_h_world.east(), drag_v_world)
    }
}

#[cfg(test)]
mod tests {
    use crate::simulator::{default_drone::DefaultDrone, inputs::Inputs, state::State};

    use primitives::prelude::*;

    use super::*;

    #[test]
    fn drag_opposes_north_velocity() {
        let model = DragModel {};
        let drone = DefaultDrone {};
        let mut state = State::default();
        state.velocity_ned = VelocityNed::new(5.0.mps(), 0.mps(), 0.mps()); // moving north
        let inputs = Inputs::default();
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };
        let acc = model.acceleration_contribution(&ctx, 1.seconds());

        assert!(acc.north().raw() < 0.0, "drag should oppose north velocity");
    }

    #[test]
    fn drag_opposes_east_velocity() {
        let model = DragModel {};
        let drone = DefaultDrone {};
        let mut state = State::default();
        state.velocity_ned = VelocityNed::new(0.mps(), 5.0.mps(), 0.mps()); // moving east
        let inputs = Inputs::default();
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };
        let acc = model.acceleration_contribution(&ctx, 1.0.seconds());

        assert!(acc.east().raw() < 0.0, "drag should oppose east velocity");
    }

    #[test]
    fn drag_acceleration_applies_correct_coefficients_at_arbitrary_yaw() {
        let drag_model = DragModel::new();

        // Moving north at 4 m/s, zero yaw — forward drag (0.2) applies to north
        let state_zero_yaw = State {
            velocity_ned: VelocityNed::from_north(4.0.mps()),
            attitude: Quaternion::identity(),
            position_ned: PositionNed::from_altitude_ned(-10.meters()),
            ..Default::default()
        };
        let ctx = Context {
            state: &state_zero_yaw,
            vehicle: &DefaultDrone {},
            inputs: &Default::default(),
        };
        let drag_zero = drag_model.acceleration_contribution(&ctx, 0.1.seconds());

        // Moving east at 4 m/s, 90° yaw — forward drag (0.2) should apply to east
        let state_yaw_90 = State {
            velocity_ned: VelocityNed::from_east(4.mps()),
            attitude: Quaternion::from_yaw(90.degrees().to_radians()),
            position_ned: PositionNed::from_altitude_ned(-10.meters()),
            ..Default::default()
        };
        let ctx = Context {
            state: &state_yaw_90,
            vehicle: &DefaultDrone {},
            inputs: &Default::default(),
        };
        let drag_90 = drag_model.acceleration_contribution(&ctx, 0.1.seconds());

        let tolerance = 1e-6;
        // Same speed, same body-frame direction (forward) — should get same magnitude
        assert!(
            (drag_zero.north().raw().abs() - drag_90.east().raw().abs()).abs() < tolerance,
            "forward drag should apply to north at 0° yaw ({}) and east at 90° yaw ({})",
            drag_zero.north().raw(),
            drag_90.east().raw()
        );

        // Cross axes should be near zero
        assert!(
            drag_zero.east().raw().abs() < tolerance,
            "no east drag at 0° yaw moving north"
        );
        assert!(
            drag_90.north().raw().abs() < tolerance,
            "no north drag at 90° yaw moving east"
        );
    }
}
