use crate::{
    simulator::{
        drone::Drone,
        force_model::{context::Context, force_model::ForceModel},
        types::{acceleration_3d::WorldFrameAcceleration, drag::Drag},
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
