use std::ops::Div;

use crate::{
    impl_debug_unit, impl_initializable, impl_literal, impl_raw_representable,
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{AngularVelocity, Degrees, Seconds},
};

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Default)]
pub struct Radians(pub f64);

impl Radians {
    pub fn sin(&self) -> f64 {
        self.0.sin()
    }

    pub fn cos(&self) -> f64 {
        self.0.cos()
    }

    pub fn tan(&self) -> f64 {
        self.0.tan()
    }

    pub fn to_degrees(self) -> Degrees {
        Degrees(self.0.to_degrees())
    }
}

impl_initializable!(Radians);
impl_raw_representable!(Radians);
impl_units_arithmetics!(Radians);
impl_literal!(Radians, radians, RadiansLiteral);
impl_debug_unit!(Radians, "rads");

/// Rad / Rad = ()
impl Div<Radians> for Radians {
    type Output = f64;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

/// Rad / s = Rad/s
impl Div<Seconds> for Radians {
    type Output = AngularVelocity;

    fn div(self, rhs: Seconds) -> Self::Output {
        AngularVelocity::new(self.0 / rhs.raw())
    }
}
