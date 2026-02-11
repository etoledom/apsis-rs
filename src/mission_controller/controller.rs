use crate::{
    mission_controller::setpoint::SetPoint,
    simulator::{inputs::Inputs, state::State, types::throttle::Throttle},
};

pub struct Controller;

impl Controller {
    pub fn new() -> Self {
        Self {}
    }

    pub fn control(&self, setpoint: SetPoint, state: &State, current_inputs: Inputs) -> Inputs {
        let throttle_adjustment = if setpoint.target_altitude > state.altitude {
            0.5
        } else if setpoint.target_altitude < state.altitude {
            -0.5
        } else {
            0.0
        };

        Inputs {
            throttle: Throttle::clamp(current_inputs.throttle.get() + throttle_adjustment),
            ..Default::default()
        }
    }
}
