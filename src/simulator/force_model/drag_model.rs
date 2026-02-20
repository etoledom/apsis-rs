use crate::{
    simulator::{
        drone::Drone,
        force_model::{context::Context, force_model::ForceModel},
        types::acceleration_3d::WorldFrameAcceleration,
    },
    units::units::Seconds,
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
    ) -> WorldFrameAcceleration {
        let ground_speed = ctx.state.velocity_ned.ground_speed();
        let vertical_velocity = ctx.state.velocity_ned.down();
        let drag_coefficient = ctx.vehicle.drag_coefficient();

        // a_drag [m/s2] = -|v|*v*k where k -> drag coef [1/m]
        WorldFrameAcceleration::new(
            -(ground_speed.north().abs() * ground_speed.north()) * drag_coefficient.forward.value(),
            -(ground_speed.east().abs() * ground_speed.east())
                * drag_coefficient.horizontal.value(),
            -(vertical_velocity.abs() * vertical_velocity) * drag_coefficient.vertical.value(),
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        simulator::{
            default_drone::DefaultDrone, inputs::Inputs, state::State,
            types::velocity_ned::VelocityNED,
        },
        units::units::{SecondsLiteral, VelocityLiteral},
    };

    use super::*;

    #[test]
    fn drag_opposes_north_velocity() {
        let model = DragModel {};
        let drone = DefaultDrone {};
        let mut state = State::default();
        state.velocity_ned = VelocityNED::new(5.0.mps(), 0.mps(), 0.mps()); // moving north
        let inputs = Inputs::default();
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };
        let acc = model.acceleration_contribution(&ctx, 1.seconds());

        assert!(acc.north().0 < 0.0, "drag should oppose north velocity");
    }

    #[test]
    fn drag_opposes_east_velocity() {
        let model = DragModel {};
        let drone = DefaultDrone {};
        let mut state = State::default();
        state.velocity_ned = VelocityNED::new(0.mps(), 5.0.mps(), 0.mps()); // moving east
        let inputs = Inputs::default();
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };
        let acc = model.acceleration_contribution(&ctx, 1.0.seconds());

        assert!(acc.east().0 < 0.0, "drag should oppose east velocity");
    }
}
