use crate::simulator::{inputs::Inputs, state::State};

pub struct Context<'a, Vehicle> {
    pub state: &'a State,
    pub inputs: &'a Inputs,
    pub vehicle: &'a Vehicle,
}

impl<'a, Vehicle> Context<'a, Vehicle> {
    pub fn new(state: &'a State, inputs: &'a Inputs, vehicle: &'a Vehicle) -> Self {
        Self {
            state,
            inputs,
            vehicle,
        }
    }
}
