use std::{f64::consts::PI, ops::Mul};

use crate::{
    impl_debug_unit, impl_initializable, impl_raw_representable, impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{AngularVelocity, Degrees, Seconds},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Default, PartialOrd)]
pub struct DegreesPerSecond(pub f64); // deg/s

impl DegreesPerSecond {
    pub fn to_angular_velocity(self) -> AngularVelocity {
        AngularVelocity::new(self.0 * (PI / 180.0))
    }
    pub fn clamp(&mut self, min: DegreesPerSecond, max: DegreesPerSecond) {
        self.0 = self.0.clamp(min.0, max.0);
    }
}

impl_initializable!(DegreesPerSecond);
impl_raw_representable!(DegreesPerSecond);
impl_units_arithmetics!(DegreesPerSecond);
impl_debug_unit!(DegreesPerSecond, "°/s");

/// (Deg/s) * s = Deg
impl Mul<Seconds> for DegreesPerSecond {
    type Output = Degrees;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Degrees(self.0 * rhs.raw())
    }
}
