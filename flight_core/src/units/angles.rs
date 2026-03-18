use crate::units::traits::RawRepresentable;
use crate::{
    AngularDamping, impl_initializable, impl_literal, impl_raw_representable,
    impl_units_arithmetics,
};
use crate::{impl_debug_unit, units::seconds::Seconds};
use std::f64::consts::PI;
use std::ops::{Div, Mul};

// =====
// Degrees
// =====

#[repr(transparent)]
#[derive(Copy, Clone, Default, PartialEq, PartialOrd)]
pub struct Degrees(pub f64);

impl Degrees {
    pub fn to_radians(&self) -> Radians {
        Radians(self.0.to_radians())
    }

    pub fn sin(self) -> f64 {
        self.0.sin()
    }

    pub fn cos(self) -> f64 {
        self.0.cos()
    }
}

impl_initializable!(Degrees);
impl_raw_representable!(Degrees);
impl_units_arithmetics!(Degrees);
impl_literal!(Degrees, degrees, DegreesLiteral);
impl_debug_unit!(Degrees, "°");

// =====
// DegreesPerSecond
// =====

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Default, PartialOrd)]
pub struct DegreesPerSecond(pub f64); // deg/s

impl DegreesPerSecond {
    fn zero() -> Self {
        Self(0.0)
    }
    pub fn to_angular_velocity(self) -> AngularVelocity {
        AngularVelocity(self.0 * (PI / 180.0))
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

// =====
// Radians
// =====

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
        AngularVelocity(self.0 / rhs.raw())
    }
}

// =======
// Angular velocity [Rad/s]
// =======

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Default, Debug, PartialOrd)]
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
        AngularAcceleration(self.0 / rhs.raw())
    }
}

// =======
// Angular acceleration [Rad/s^2]
// =======

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Default, Debug, PartialOrd)]
pub struct AngularAcceleration(f64);

impl_initializable!(AngularAcceleration);
impl_raw_representable!(AngularAcceleration);
impl_units_arithmetics!(AngularAcceleration);
impl_debug_unit!(AngularAcceleration, "rads/s^2");

/// (Rad/s^2) / s = Rad/s
impl Mul<Seconds> for AngularAcceleration {
    type Output = AngularVelocity;

    fn mul(self, rhs: Seconds) -> Self::Output {
        AngularVelocity(self.0 * rhs.raw())
    }
}

/// (Rad/s^2) / (1/s) = Rad/s
impl Div<AngularDamping> for AngularAcceleration {
    type Output = AngularVelocity;

    fn div(self, rhs: AngularDamping) -> Self::Output {
        AngularVelocity(self.0 / rhs.raw())
    }
}
