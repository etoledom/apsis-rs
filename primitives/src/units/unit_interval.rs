use crate::{
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Default, PartialOrd)]
pub struct UnitInterval(f64);

impl UnitInterval {
    pub fn clamp(value: f64) -> Self {
        Self(value.clamp(0.0, 1.0))
    }
}

impl Initializable for UnitInterval {
    fn new(value: impl Into<f64>) -> Self {
        Self::clamp(value.into())
    }
}

impl RawRepresentable for UnitInterval {
    fn raw(&self) -> f64 {
        self.0.clamp(0.0, 1.0)
    }
}

impl_units_arithmetics!(UnitInterval);
