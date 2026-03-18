use crate::{
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::SignedUnitInterval,
};

#[repr(transparent)]
#[derive(Clone, Copy, Default)]
pub struct Yaw(SignedUnitInterval);

impl Yaw {
    pub fn clamp(value: f64) -> Self {
        Self(SignedUnitInterval::clamp(value))
    }
}

impl Initializable for Yaw {
    #[inline(always)]
    fn new(value: impl Into<f64>) -> Self {
        Self::clamp(value.into())
    }
}

impl RawRepresentable for Yaw {
    #[inline(always)]
    fn raw(&self) -> f64 {
        self.0.raw()
    }
}

impl_units_arithmetics!(Yaw);
