use std::ops::{Add, Div, Mul, Neg};

use crate::units::units::Seconds;

// =====
// Degrees
// =====

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq)]
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
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Radians(pub f64);

impl Radians {
    pub fn hypot(&self, other: Radians) -> Radians {
        Radians(self.0.hypot(other.0))
    }

    pub fn sin(&self) -> f64 {
        self.0.sin()
    }

    pub fn cos(&self) -> f64 {
        self.0.cos()
    }

    pub fn is_zero(&self) -> bool {
        self.0 == 0.0
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
