use crate::{
    simulator::{
        drone::Drone,
        force_model::{context::Context, force_model::ForceModel},
        types::acceleration_3d::WorldFrameAcceleration,
    },
    units::units::Seconds,
};

pub struct ThrustModel;

impl<'a, Vehicle: Drone> ForceModel<Vehicle> for ThrustModel {
    fn acceleration_contribution(
        &self,
        ctx: &Context<Vehicle>,
        _: Seconds,
    ) -> WorldFrameAcceleration {
        let inputs = ctx.inputs;
        let body_frame_acceleration =
            ctx.vehicle
                .body_frame_acceleration(inputs.throttle, inputs.pitch, inputs.roll);

        body_frame_acceleration.rotate_to_world_frame_by(ctx.state.heading.to_radians())
    }
}
