use crate::{
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::SignedUnitInterval,
};

#[repr(transparent)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Pitch(SignedUnitInterval);

impl Pitch {
    pub fn clamp(value: f64) -> Self {
        Self(SignedUnitInterval::clamp(value))
    }
}

impl_units_arithmetics!(Pitch);

impl Initializable for Pitch {
    #[inline(always)]
    fn new(value: impl Into<f64>) -> Self {
        Self::clamp(value.into())
    }
}

impl RawRepresentable for Pitch {
    #[inline(always)]
    fn raw(&self) -> f64 {
        self.0.raw()
    }
}
