use std::ops::Mul;

use crate::{
    impl_debug_unit, impl_initializable, impl_literal, impl_raw_representable,
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{
        Acceleration, Meters, Seconds, SecondsCube, SecondsSquare, Velocity,
        acceleration_square::AccelerationSquare,
    },
};

#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
/// m/s^3 - Describing the change of acceleration over time.
pub struct Jerk(f64);

impl_initializable!(Jerk);
impl_raw_representable!(Jerk);
impl_units_arithmetics!(Jerk);
impl_literal!(Jerk, mps3, JerkLiteral);
impl_debug_unit!(Jerk, "m/s^3");

/// (m/s^3) * s = m/s^2
impl Mul<Seconds> for Jerk {
    type Output = Acceleration;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Acceleration::new(self.0 * rhs.raw())
    }
}

/// (m/s^3) * s^2 = m/s
impl Mul<SecondsSquare> for Jerk {
    type Output = Velocity;

    fn mul(self, rhs: SecondsSquare) -> Self::Output {
        Velocity::new(self.0 * rhs.raw())
    }
}

/// (m/s^3) * s^3 = m
impl Mul<SecondsCube> for Jerk {
    type Output = Meters;

    fn mul(self, rhs: SecondsCube) -> Self::Output {
        Meters::new(self.0 * rhs.raw())
    }
}

/// (m/s^3) * m/s = (m^2/s^4) = (m/s^2)^2
impl Mul<Velocity> for Jerk {
    type Output = AccelerationSquare;

    fn mul(self, rhs: Velocity) -> Self::Output {
        AccelerationSquare::new(self.0 * rhs.raw())
    }
}
