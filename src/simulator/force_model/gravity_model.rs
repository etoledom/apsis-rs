use crate::{
    simulator::{
        drone::Drone,
        force_model::{context::Context, force_model::ForceModel},
        types::acceleration_3d::WorldFrameAcceleration,
    },
    units::{consts::G_EARTH, units::Seconds},
};

pub struct GravityModel;

impl<Vehicle: Drone> ForceModel<Vehicle> for GravityModel {
    fn acceleration_contribution<'a>(
        &self,
        _: &Context<Vehicle>,
        _: Seconds,
    ) -> crate::simulator::types::acceleration_3d::WorldFrameAcceleration {
        WorldFrameAcceleration::from_vertical(-G_EARTH)
    }
}
