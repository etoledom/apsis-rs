use crate::{
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::UnitInterval,
};

#[repr(transparent)]
#[derive(Clone, Copy, Default)]
pub struct Throttle(UnitInterval);

impl Throttle {
    pub fn max() -> Self {
        Self::clamp(1.0)
    }

    pub fn clamp(value: f64) -> Self {
        Self(UnitInterval::clamp(value))
    }

    pub fn clamping(self, value: f64) -> Self {
        Self::clamp(value)
    }
}

impl Initializable for Throttle {
    #[inline(always)]
    fn new(value: impl Into<f64>) -> Self {
        Self::clamp(value.into())
    }
}

impl RawRepresentable for Throttle {
    #[inline(always)]
    fn raw(&self) -> f64 {
        self.0.raw()
    }
}

impl_units_arithmetics!(Throttle);
