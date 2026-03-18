use crate::{
    traits::{Initializable, RawRepresentable},
    units::PerMeter,
};

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Drag(PerMeter);

impl Drag {
    pub fn new(value: f64) -> Self {
        Self(PerMeter::new(value))
    }
    pub fn value(&self) -> PerMeter {
        self.0
    }
    pub fn raw(&self) -> f64 {
        self.value().raw()
    }
}
