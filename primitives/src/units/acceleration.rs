use core::ops::Mul;
use std::ops::Div;

use crate::{
    impl_debug_unit, impl_initializable, impl_literal, impl_raw_representable,
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{
        AccelerationSquare, AngularVelocity, Drag, Jerk, Meters, PerMeter, PerSecond, Seconds,
        SecondsSquare, Velocity, VelocitySquare,
    },
};

#[repr(transparent)]
#[derive(Copy, Clone, Default, Debug, PartialEq, PartialOrd)]
pub struct Acceleration(f64); // m/s²

impl Acceleration {
    pub const fn gravity() -> Self {
        Self(9.80665)
    }
}

impl_initializable!(Acceleration);
impl_raw_representable!(Acceleration);
impl_units_arithmetics!(Acceleration);
impl_literal!(Acceleration, mps2, AccelerationLiteral);
impl_debug_unit!(Acceleration, "m/s2");

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

/// m/s^2 / s = m/s^3 = Jerk
impl Div<Seconds> for Acceleration {
    type Output = Jerk;

    fn div(self, rhs: Seconds) -> Self::Output {
        Jerk::new(self.0 / rhs.raw())
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
        Velocity::new(self.0 * rhs.raw())
    }
}

/// acc[m/s^2] * drag_coef[1/m] = vel^2[(m/s)^2]
impl Div<PerMeter> for Acceleration {
    type Output = VelocitySquare;

    fn div(self, rhs: PerMeter) -> Self::Output {
        VelocitySquare(self.0 / rhs.raw())
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
        VelocitySquare(self.0 * rhs.raw())
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
