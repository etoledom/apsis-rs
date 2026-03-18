use std::ops::{Div, Mul};

use crate::{
    impl_debug_unit, impl_initializable, impl_literal, impl_raw_representable,
    impl_units_arithmetics,
    units::{
        Meters, PerSecond, Seconds, VelocitySquare,
        acceleration::Acceleration,
        traits::{Initializable, RawRepresentable},
    },
};

#[repr(transparent)]
#[derive(Copy, Clone, Default, PartialEq, Debug, PartialOrd)]
pub struct Velocity(f64); // m/s

impl Velocity {
    pub fn clamp(&mut self, min: Velocity, max: Velocity) {
        self.0 = self.0.clamp(min.0, max.0);
    }

    pub fn clamping(&self, min: Velocity, max: Velocity) -> Self {
        Self(self.0.clamp(min.0, max.0))
    }
}

impl_raw_representable!(Velocity);
impl_initializable!(Velocity);
impl_units_arithmetics!(Velocity);
impl_literal!(Velocity, mps, VelocityLiteral);
impl_debug_unit!(Velocity, "m/s");

/// (m/s) / 1/s = m/s^2
impl Mul<PerSecond> for Velocity {
    type Output = Acceleration;

    fn mul(self, rhs: PerSecond) -> Self::Output {
        Acceleration::new(self.0 * rhs.0)
    }
}

/// (m/s) / m/s^2 = s
impl Div<Acceleration> for Velocity {
    type Output = Seconds;

    fn div(self, rhs: Acceleration) -> Self::Output {
        Seconds::new(self.0 / rhs.raw())
    }
}

/// (m/s) * s = m
impl Mul<Seconds> for Velocity {
    type Output = Meters;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Meters::new(self.0 * rhs.raw())
    }
}

/// (m/s) / (m/s) = (m/s)^2
impl Mul<Velocity> for Velocity {
    type Output = VelocitySquare;

    fn mul(self, rhs: Velocity) -> Self::Output {
        VelocitySquare(self.0 * rhs.0)
    }
}
