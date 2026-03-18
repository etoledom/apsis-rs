use primitives::{frames::AccelerationNed, units::Seconds};

use crate::simulator::{drone::Drone, force_model::context::Context};

pub trait ForceModel<Vehicle: Drone> {
    fn acceleration_contribution<'a>(
        &self,
        ctx: &Context<Vehicle>,
        delta_t: Seconds,
    ) -> AccelerationNed;
}
