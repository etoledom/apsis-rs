use crate::{impl_initializable, impl_raw_representable, impl_units_arithmetics};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Default, PartialOrd)]
pub struct UnitInterval(f64);

impl UnitInterval {
    pub fn clamp(value: f64) -> Self {
        Self(value.clamp(0.0, 1.0))
    }
}

impl_initializable!(UnitInterval);
impl_raw_representable!(UnitInterval);
impl_units_arithmetics!(UnitInterval);
