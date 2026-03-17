use core::ops::Mul;
use std::ops::{Add, Div, Neg, Sub};

use crate::{
    Drag, Jerk,
    units::{
        Meters, PerSecond, SecondsSquare,
        acceleration_square::AccelerationSquare,
        angles::AngularVelocity,
        traits::{Initializable, RawRepresentable},
        units::{PerMeter, Seconds, Velocity, VelocitySquare},
    },
};

#[repr(transparent)]
#[derive(Copy, Clone, Default, Debug, PartialEq, PartialOrd)]
pub struct Acceleration(pub f64); // m/s²

impl Acceleration {
    pub fn raw(&self) -> f64 {
        self.0
    }
    pub fn clamping(self, min: Acceleration, max: Acceleration) -> Acceleration {
        Acceleration(self.0.clamp(min.0, max.0))
    }
    pub fn abs(self) -> Acceleration {
        Acceleration(self.0.abs())
    }
}

impl Initializable for Acceleration {
    fn new(value: f64) -> Self {
        Self(value)
    }
}

impl Neg for Acceleration {
    type Output = Acceleration;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl Mul<f64> for Acceleration {
    type Output = Acceleration;

    fn mul(self, rhs: f64) -> Acceleration {
        Acceleration(self.0 * rhs)
    }
}

impl Mul<Acceleration> for f64 {
    type Output = Acceleration;

    fn mul(self, rhs: Acceleration) -> Self::Output {
        Acceleration(self * rhs.0)
    }
}

/// m/s^2 / 1/m = (m/s)^2 = VelocitySquare
impl Div<Drag> for Acceleration {
    type Output = VelocitySquare;

    fn div(self, rhs: Drag) -> Self::Output {
        VelocitySquare(self.0 / rhs.raw())
    }
}

impl Div<Acceleration> for Acceleration {
    type Output = f64;

    fn div(self, rhs: Acceleration) -> Self::Output {
        self.0 / rhs.0
    }
}

impl Div<f64> for Acceleration {
    type Output = Acceleration;

    fn div(self, rhs: f64) -> Self::Output {
        Acceleration(self.0 / rhs)
    }
}

/// m/s^2 / s = m/s^3 = Jerk
impl Div<Seconds> for Acceleration {
    type Output = Jerk;

    fn div(self, rhs: Seconds) -> Self::Output {
        Jerk::new(self.0 / rhs.0)
    }
}

impl Sub<Acceleration> for Acceleration {
    type Output = Acceleration;

    fn sub(self, rhs: Acceleration) -> Self::Output {
        Acceleration(self.0 - rhs.0)
    }
}

/// m/s^2 * rad/s = m/s^2 * 1/s = m/s3 = Jerk
impl Mul<AngularVelocity> for Acceleration {
    type Output = Jerk;

    fn mul(self, rhs: AngularVelocity) -> Self::Output {
        Jerk::new(self.0 * rhs.raw())
    }
}

// v = a * t;
impl Mul<Seconds> for Acceleration {
    type Output = Velocity;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Velocity(self.0 * rhs.0)
    }
}

impl Add<Acceleration> for Acceleration {
    type Output = Acceleration;

    fn add(self, rhs: Acceleration) -> Self::Output {
        Acceleration(self.0 + rhs.0)
    }
}

pub trait AccelerationLiteral {
    fn mps2(self) -> Acceleration;
}

impl AccelerationLiteral for f64 {
    fn mps2(self) -> Acceleration {
        Acceleration(self)
    }
}

impl AccelerationLiteral for i32 {
    fn mps2(self) -> Acceleration {
        Acceleration(self as f64)
    }
}

/// acc[m/s^2] * drag_coef[1/m] = vel^2[(m/s)^2]
impl Div<PerMeter> for Acceleration {
    type Output = VelocitySquare;

    fn div(self, rhs: PerMeter) -> Self::Output {
        VelocitySquare(self.0 / rhs.0)
    }
}

/// m/s^2 / m/s^3 = s
impl Div<Jerk> for Acceleration {
    type Output = Seconds;

    fn div(self, rhs: Jerk) -> Self::Output {
        Seconds::new(self.0 / rhs.raw())
    }
}

/// m/s^2 * 1/s = m/s^3
impl Mul<PerSecond> for Acceleration {
    type Output = Jerk;

    fn mul(self, rhs: PerSecond) -> Self::Output {
        Jerk::new(self.0 * rhs.0)
    }
}

/// (m/s^2) * m = (m/s)^2
impl Mul<Meters> for Acceleration {
    type Output = VelocitySquare;

    fn mul(self, rhs: Meters) -> Self::Output {
        VelocitySquare(self.0 * rhs.0)
    }
}

/// m/s^2 * s^2 = m
impl Mul<SecondsSquare> for Acceleration {
    type Output = Meters;

    fn mul(self, rhs: SecondsSquare) -> Self::Output {
        Meters::new(self.0 * rhs.raw())
    }
}

impl Mul for Acceleration {
    type Output = AccelerationSquare;

    fn mul(self, rhs: Self) -> Self::Output {
        AccelerationSquare::new(self.0 * rhs.0)
    }
}
