use std::ops::{Div, Mul};

use crate::{
    impl_debug_unit, impl_initializable, impl_raw_representable, impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{AngularDamping, AngularVelocity, Seconds},
};

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Default, Debug, PartialOrd)]
/// rad/s^2
pub struct AngularAcceleration(f64);

impl_initializable!(AngularAcceleration);
impl_raw_representable!(AngularAcceleration);
impl_units_arithmetics!(AngularAcceleration);
impl_debug_unit!(AngularAcceleration, "rads/s^2");

/// (Rad/s^2) / s = Rad/s
impl Mul<Seconds> for AngularAcceleration {
    type Output = AngularVelocity;

    fn mul(self, rhs: Seconds) -> Self::Output {
        AngularVelocity::new(self.0 * rhs.raw())
    }
}

/// (Rad/s^2) / (1/s) = Rad/s
impl Div<AngularDamping> for AngularAcceleration {
    type Output = AngularVelocity;

    fn div(self, rhs: AngularDamping) -> Self::Output {
        AngularVelocity::new(self.0 / rhs.raw())
    }
}
