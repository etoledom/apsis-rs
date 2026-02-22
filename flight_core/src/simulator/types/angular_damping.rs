use std::ops::Mul;

use crate::units::{
    angles::{AngularAcceleration, AngularVelocity},
    units::PerSecond,
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
