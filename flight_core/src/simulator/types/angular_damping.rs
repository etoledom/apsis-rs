use std::ops::{Div, Mul};

use crate::units::{
    PerSecond, Seconds,
    angles::{AngularAcceleration, AngularVelocity},
    traits::{Initializable, RawRepresentable},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct AngularDamping(PerSecond);

impl AngularDamping {
    pub fn new(value: f64) -> Self {
        Self(PerSecond(value))
    }
    pub fn raw(&self) -> f64 {
        self.0.0
    }
}

impl Mul<AngularVelocity> for AngularDamping {
    type Output = AngularAcceleration;

    fn mul(self, rhs: AngularVelocity) -> Self::Output {
        AngularAcceleration::new(self.raw() * rhs.raw())
    }
}

/// 1/s / [rad]/s^2 = s
impl Div<AngularAcceleration> for AngularDamping {
    type Output = Seconds;

    fn div(self, rhs: AngularAcceleration) -> Self::Output {
        Seconds::new(self.raw() / rhs.raw())
    }
}
