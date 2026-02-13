use crate::{
    simulator::{
        drone::Drone, force_model::context::Context, types::acceleration_3d::WorldFrameAcceleration,
    },
    units::units::Seconds,
};

pub trait ForceModel<Vehicle: Drone> {
    fn acceleration_contribution<'a>(
        &self,
        ctx: &Context<Vehicle>,
        delta_t: Seconds,
    ) -> WorldFrameAcceleration;
}
