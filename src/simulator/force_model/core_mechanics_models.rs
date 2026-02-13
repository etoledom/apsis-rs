use crate::{
    simulator::{
        drone::Drone,
        force_model::{
            context::Context, drag_model::DragModel, force_model::ForceModel,
            gravity_model::GravityModel, thrust_model::ThrustModel,
        },
        types::acceleration_3d::WorldFrameAcceleration,
    },
    units::units::Seconds,
};

pub struct CoreMechanicsModel {
    gravity: GravityModel,
    drag: DragModel,
    thrust: ThrustModel,
}

impl CoreMechanicsModel {
    pub fn new() -> Self {
        Self {
            gravity: GravityModel {},
            drag: DragModel::new(),
            thrust: ThrustModel {},
        }
    }
}

impl<'a, Vehicle: Drone> ForceModel<Vehicle> for CoreMechanicsModel {
    fn acceleration_contribution(
        &self,
        ctx: &Context<Vehicle>,
        delta_t: Seconds,
    ) -> WorldFrameAcceleration {
        self.gravity.acceleration_contribution(ctx, delta_t)
            + self.drag.acceleration_contribution(ctx, delta_t)
            + self.thrust.acceleration_contribution(ctx, delta_t)
    }
}
