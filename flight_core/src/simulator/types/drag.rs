use crate::units::units::PerMeter;

#[derive(Clone)]
pub struct Drag(PerMeter);

impl Drag {
    pub fn new(value: f64) -> Self {
        Self(PerMeter(value))
    }

    pub fn value(&self) -> PerMeter {
        self.0
    }
}
