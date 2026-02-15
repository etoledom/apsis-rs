use crate::{
    controller::altitude_controller::AltitudeController,
    simulator::{inputs::Inputs, state::State},
    units::units::{Meters, Seconds},
};

pub struct Controller {
    altitude_controller: AltitudeController,
}

impl Controller {
    pub fn new(altitude_target: Meters) -> Self {
        Self {
            altitude_controller: AltitudeController::new(altitude_target),
        }
    }
    pub fn update(&mut self, telemetry: &State, dt: Seconds) -> Inputs {
        Inputs {
            throttle: self.altitude_controller.update(telemetry, dt),
            ..Default::default()
        }
    }
}
