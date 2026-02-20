use core::ops::Mul;
use std::ops::{Add, Div, Neg, Sub};

use crate::units::units::{PerMeter, Seconds, Velocity, VelocitySquare};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Acceleration(pub f64); // m/s²

impl Acceleration {
    pub fn raw(&self) -> f64 {
        self.0
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

impl Div<Acceleration> for Acceleration {
    type Output = f64;

    fn div(self, rhs: Acceleration) -> Self::Output {
        self.0 / rhs.0
    }
}

impl Sub<Acceleration> for Acceleration {
    type Output = Acceleration;

    fn sub(self, rhs: Acceleration) -> Self::Output {
        Acceleration(self.0 - rhs.0)
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
