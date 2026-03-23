use crate::{
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct SignedUnitInterval(f64);

impl SignedUnitInterval {
    pub fn clamp(value: f64) -> Self {
        Self(value.clamp(-1.0, 1.0))
    }
}

impl Initializable for SignedUnitInterval {
    #[inline(always)]
    fn new(value: impl Into<f64>) -> Self {
        Self::clamp(value.into())
    }
}

impl RawRepresentable for SignedUnitInterval {
    #[inline(always)]
    fn raw(&self) -> f64 {
        self.0.clamp(-1.0, 1.0)
    }
}

impl_units_arithmetics!(SignedUnitInterval);
