use crate::{impl_initializable, impl_raw_representable, impl_units_arithmetics};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct SignedUnitInterval(f64);

impl SignedUnitInterval {
    pub fn clamp(value: f64) -> Self {
        Self(value.clamp(-1.0, 1.0))
    }
}

impl_initializable!(SignedUnitInterval);
impl_raw_representable!(SignedUnitInterval);
impl_units_arithmetics!(SignedUnitInterval);
