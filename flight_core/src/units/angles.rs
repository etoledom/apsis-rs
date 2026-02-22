use crate::{impl_debug_unit, units::units::Seconds};
use std::fmt;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

// =====
// Degrees
// =====

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq)]
pub struct Degrees(pub f64);

impl Degrees {
    pub fn to_radians(&self) -> Radians {
        Radians(self.0.to_radians())
    }

    pub fn zero() -> Self {
        Self(0.0)
    }

    pub fn sin(self) -> f64 {
        self.0.sin()
    }
}

impl Default for Degrees {
    fn default() -> Self {
        Self::zero()
    }
}

pub trait DegreesLiteral {
    fn degrees(self) -> Degrees;
}

impl DegreesLiteral for f64 {
    fn degrees(self) -> Degrees {
        Degrees(self)
    }
}

impl DegreesLiteral for i32 {
    fn degrees(self) -> Degrees {
        Degrees(self as f64)
    }
}

// =====
// DegreesPerSecond
// =====

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DegreesPerSecond(pub f64); // deg/s

impl DegreesPerSecond {
    fn zero() -> Self {
        Self(0.0)
    }
}

impl Default for DegreesPerSecond {
    fn default() -> Self {
        Self::zero()
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

    pub fn clamp(self, min: Radians, max: Radians) -> Radians {
        Radians(self.0.clamp(min.0, max.0))
    }

    pub fn to_degrees(self) -> Degrees {
        Degrees(self.0.to_degrees())
    }
}

impl Div<Radians> for Radians {
    type Output = f64;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

impl Mul<f64> for Degrees {
    type Output = Degrees;

    fn mul(self, rhs: f64) -> Self::Output {
        Degrees(self.0 * rhs)
    }
}

impl Add for Radians {
    type Output = Radians;

    fn add(self, rhs: Self) -> Self::Output {
        Radians(self.0 + rhs.0)
    }
}

impl Neg for Radians {
    type Output = Radians;

    fn neg(self) -> Self::Output {
        Radians(-self.0)
    }
}

impl Sub for Radians {
    type Output = Radians;

    fn sub(self, rhs: Self) -> Self::Output {
        Radians(self.0 - rhs.0)
    }
}

impl Mul<f64> for Radians {
    type Output = Radians;

    fn mul(self, rhs: f64) -> Self::Output {
        Radians(self.0 * rhs)
    }
}

impl Div<f64> for Radians {
    type Output = Radians;

    fn div(self, rhs: f64) -> Self::Output {
        Radians(self.0 / rhs)
    }
}

impl AddAssign for Radians {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Add<DegreesPerSecond> for DegreesPerSecond {
    type Output = DegreesPerSecond;

    fn add(self, rhs: DegreesPerSecond) -> Self::Output {
        DegreesPerSecond(self.0 + rhs.0)
    }
}

impl Div<f64> for DegreesPerSecond {
    type Output = DegreesPerSecond;

    fn div(self, rhs: f64) -> Self::Output {
        DegreesPerSecond(self.0 / rhs)
    }
}

impl Mul<Seconds> for DegreesPerSecond {
    type Output = Degrees;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Degrees(self.0 * rhs.0)
    }
}

impl Mul<f64> for DegreesPerSecond {
    type Output = DegreesPerSecond;

    fn mul(self, rhs: f64) -> Self::Output {
        DegreesPerSecond(self.0 * rhs)
    }
}

impl Add<Degrees> for Degrees {
    type Output = Degrees;

    fn add(self, rhs: Degrees) -> Self::Output {
        Degrees(self.0 + rhs.0)
    }
}

// =======
// Angular velocity [Rad/s]
// =======

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Default, Debug)]
pub struct AngularVelocity(f64);

impl AngularVelocity {
    pub fn raw(&self) -> f64 {
        self.0
    }
}

impl From<f64> for AngularVelocity {
    fn from(value: f64) -> Self {
        AngularVelocity(value)
    }
}

impl Mul<Seconds> for AngularVelocity {
    type Output = Radians;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Radians(self.0 * rhs.0)
    }
}

impl Div<Seconds> for Radians {
    type Output = AngularVelocity;

    fn div(self, rhs: Seconds) -> Self::Output {
        AngularVelocity(self.0 / rhs.0)
    }
}

impl AddAssign for AngularVelocity {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Add for AngularVelocity {
    type Output = AngularVelocity;

    fn add(self, rhs: Self) -> Self::Output {
        AngularVelocity(self.0 + rhs.0)
    }
}

impl Div<f64> for AngularVelocity {
    type Output = AngularVelocity;

    fn div(self, rhs: f64) -> Self::Output {
        AngularVelocity(self.0 * rhs)
    }
}

impl Mul<f64> for AngularVelocity {
    type Output = AngularVelocity;

    fn mul(self, rhs: f64) -> Self::Output {
        AngularVelocity(self.0 * rhs)
    }
}

impl Sub for AngularVelocity {
    type Output = AngularVelocity;

    fn sub(self, rhs: Self) -> Self::Output {
        AngularVelocity(self.0 - rhs.0)
    }
}

// =======
// Angular acceleration [Rad/s^2]
// =======

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Default, Debug)]
pub struct AngularAcceleration(f64);

impl AngularAcceleration {
    pub fn new(value: f64) -> Self {
        AngularAcceleration(value)
    }
    pub fn raw(&self) -> f64 {
        self.0
    }
}

impl Mul<Seconds> for AngularAcceleration {
    type Output = AngularVelocity;

    fn mul(self, rhs: Seconds) -> Self::Output {
        AngularVelocity(self.0 * rhs.0)
    }
}

impl Mul<f64> for AngularAcceleration {
    type Output = AngularAcceleration;

    fn mul(self, rhs: f64) -> Self::Output {
        AngularAcceleration::new(self.0 * rhs)
    }
}

impl Neg for AngularAcceleration {
    type Output = AngularAcceleration;

    fn neg(self) -> Self::Output {
        AngularAcceleration(-self.0)
    }
}

impl Div<Seconds> for AngularVelocity {
    type Output = AngularAcceleration;

    fn div(self, rhs: Seconds) -> Self::Output {
        AngularAcceleration(self.0 / rhs.0)
    }
}

impl SubAssign for AngularAcceleration {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0
    }
}

impl Sub for AngularAcceleration {
    type Output = AngularAcceleration;

    fn sub(self, rhs: Self) -> Self::Output {
        AngularAcceleration::new(self.raw() - rhs.raw())
    }
}

impl_debug_unit!(AngularVelocity, "rad/s");
impl_debug_unit!(AngularAcceleration, "rad/s2");
