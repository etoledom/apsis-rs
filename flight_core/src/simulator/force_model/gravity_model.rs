use primitives::{
    frames::AccelerationNed,
    units::{Seconds, consts::G_EARTH},
};

use crate::simulator::{
    drone::Drone,
    force_model::{context::Context, force_model::ForceModel},
};

pub struct GravityModel;

impl<Vehicle: Drone> ForceModel<Vehicle> for GravityModel {
    fn acceleration_contribution<'a>(&self, _: &Context<Vehicle>, _: Seconds) -> AccelerationNed {
        AccelerationNed::from_down(G_EARTH)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulator::{default_drone::DefaultDrone, inputs::Inputs, state::State};
    use primitives::prelude::*;

    #[test]
    fn gravity_always_points_down() {
        let model = GravityModel {};
        let drone = DefaultDrone {};
        let state = State::default();
        let inputs = Inputs::default();
        let ctx = Context {
            state: &state,
            inputs: &inputs,
            vehicle: &drone,
        };
        let acc = model.acceleration_contribution(&ctx, 1.0.seconds());

        assert_eq!(
            acc.north().raw(),
            0.0,
            "gravity should have no north component"
        );
        assert_eq!(
            acc.east().raw(),
            0.0,
            "gravity should have no east component"
        );
        assert!(
            acc.down().raw() > 0.0,
            "gravity should be positive down in NED"
        );
    }
}
