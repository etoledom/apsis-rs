use std::ops::{Mul, Neg};

use crate::units::{
    Meters, Seconds, SecondsCube, SecondsSquare, Velocity, acceleration::Acceleration,
    acceleration_square::AccelerationSquare, traits::RawRepresentable,
};

#[derive(Debug, Clone, Copy, Default, PartialEq)]
/// m/s^3 - Describing the change of acceleration over time.
pub struct Jerk(f64);

impl Jerk {
    pub fn new(value: f64) -> Self {
        Self(value)
    }
    pub fn clamping(self, min: Jerk, max: Jerk) -> Self {
        Self(self.0.clamp(min.0, max.0))
    }
}

impl RawRepresentable for Jerk {
    fn raw(&self) -> f64 {
        self.0
    }
}

impl Mul<Jerk> for f64 {
    type Output = Jerk;

    fn mul(self, rhs: Jerk) -> Self::Output {
        Jerk(self * rhs.0)
    }
}

impl Mul<f64> for Jerk {
    type Output = Jerk;

    fn mul(self, rhs: f64) -> Self::Output {
        Jerk(self.0 * rhs)
    }
}

/// (m/s^3) * s = m/s^2
impl Mul<Seconds> for Jerk {
    type Output = Acceleration;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Acceleration(self.0 * rhs.0)
    }
}

/// (m/s^3) * s^2 = m/s
impl Mul<SecondsSquare> for Jerk {
    type Output = Velocity;

    fn mul(self, rhs: SecondsSquare) -> Self::Output {
        Velocity(self.0 * rhs.raw())
    }
}

/// (m/s^3) * s^3 = m
impl Mul<SecondsCube> for Jerk {
    type Output = Meters;

    fn mul(self, rhs: SecondsCube) -> Self::Output {
        Meters(self.0 * rhs.raw())
    }
}

/// (m/s^3) * m/s = (m^2/s^4) = (m/s^2)^2
impl Mul<Velocity> for Jerk {
    type Output = AccelerationSquare;

    fn mul(self, rhs: Velocity) -> Self::Output {
        AccelerationSquare::new(self.0 * rhs.raw())
    }
}

impl Neg for Jerk {
    type Output = Jerk;

    fn neg(self) -> Self::Output {
        Jerk(-self.0)
    }
}

pub trait JerkLiteral {
    fn mps3(self) -> Jerk;
}

impl JerkLiteral for i32 {
    fn mps3(self) -> Jerk {
        Jerk(self as f64)
    }
}

impl JerkLiteral for f64 {
    fn mps3(self) -> Jerk {
        Jerk(self)
    }
}
