use std::ops::{Div, Mul};

use crate::{
    impl_debug_unit, impl_initializable, impl_literal, impl_raw_representable,
    impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{PerSecond, Seconds, Velocity},
};

#[repr(transparent)]
#[derive(Copy, Clone, Default, PartialEq, PartialOrd, Debug)]
pub struct Meters(f64);

impl Meters {
    pub const fn radius_earth() -> Self {
        Self(6371000.0)
    }
}

impl_initializable!(Meters);
impl_raw_representable!(Meters);
impl_units_arithmetics!(Meters);
impl_literal!(Meters, meters, MetersLiteral);
impl_debug_unit!(Meters, "m");

impl Mul<PerSecond> for Meters {
    type Output = Velocity;

    fn mul(self, rhs: PerSecond) -> Self::Output {
        Velocity::new(self.0 * rhs.0)
    }
}

impl Div<Seconds> for Meters {
    type Output = Velocity;

    fn div(self, rhs: Seconds) -> Self::Output {
        Velocity::new(self.0 / rhs.raw())
    }
}

impl Div<Meters> for Meters {
    type Output = f64;

    fn div(self, rhs: Meters) -> Self::Output {
        self.0 / rhs.0
    }
}
