use crate::{
    controller::{
        altitude_controller::AltitudeController,
        forward_velocity_controller::ForwardVelocityController,
    },
    simulator::{inputs::Inputs, state::State},
    units::{
        angles::Radians,
        units::{Meters, Seconds, Velocity},
    },
};

pub struct AirframeLimits {
    pub max_pitch: Radians,
    pub max_roll: Radians,
}

pub struct Controller {
    altitude_controller: AltitudeController,
    forward_velocity_controller: ForwardVelocityController,
    limits: AirframeLimits,
}

impl Controller {
    pub fn new(
        altitude_target: Meters,
        target_forward_velocity: Velocity,
        limits: AirframeLimits,
    ) -> Self {
        Self {
            altitude_controller: AltitudeController::new(altitude_target),
            forward_velocity_controller: ForwardVelocityController::new(
                target_forward_velocity,
                limits.max_pitch,
            ),
            limits,
        }
    }
    pub fn update(&mut self, telemetry: &State, dt: Seconds) -> Inputs {
        Inputs {
            throttle: self.altitude_controller.update(telemetry, dt),
            ..Default::default()
        }
    }
}
