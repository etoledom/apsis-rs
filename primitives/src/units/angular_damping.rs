use std::ops::{Div, Mul};

use crate::{
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{AngularAcceleration, AngularVelocity, PerSecond, Seconds},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct AngularDamping(PerSecond);

impl AngularDamping {
    #[inline(always)]
    pub fn new(value: f64) -> Self {
        Self(PerSecond(value))
    }
    #[inline(always)]
    pub fn value(&self) -> PerSecond {
        self.0
    }
    #[inline(always)]
    pub fn raw(&self) -> f64 {
        self.0.0
    }
}

impl_units_arithmetics!(AngularDamping);

/// 1/s * rad/s = rad/s^2
impl Mul<AngularVelocity> for AngularDamping {
    type Output = AngularAcceleration;

    fn mul(self, rhs: AngularVelocity) -> Self::Output {
        AngularAcceleration::new(self.raw() * rhs.raw())
    }
}

/// 1/s / rad/s^2 = s
impl Div<AngularAcceleration> for AngularDamping {
    type Output = Seconds;

    fn div(self, rhs: AngularAcceleration) -> Self::Output {
        Seconds::new(self.raw() / rhs.raw())
    }
}
