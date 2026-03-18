use std::ops::{Div, Mul};

use crate::{
    impl_debug_unit, impl_initializable, impl_raw_representable, impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{AngularAcceleration, Radians, Seconds},
};

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Default, Debug, PartialOrd)]
/// Rad/s
pub struct AngularVelocity(f64);

impl_initializable!(AngularVelocity);
impl_raw_representable!(AngularVelocity);
impl_units_arithmetics!(AngularVelocity);
impl_debug_unit!(AngularVelocity, "rads/s");

impl From<f64> for AngularVelocity {
    fn from(value: f64) -> Self {
        AngularVelocity(value)
    }
}

/// (Rad/s) * s = Rad
impl Mul<Seconds> for AngularVelocity {
    type Output = Radians;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Radians(self.0 * rhs.raw())
    }
}

/// (Rad/s) / s = Rad/s^2
impl Div<Seconds> for AngularVelocity {
    type Output = AngularAcceleration;

    fn div(self, rhs: Seconds) -> Self::Output {
        AngularAcceleration::new(self.0 / rhs.raw())
    }
}
