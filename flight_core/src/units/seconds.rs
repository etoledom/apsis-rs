use std::ops::Mul;

use crate::impl_debug_unit;
use crate::impl_literal;
use crate::impl_units_arithmetics;

use crate::units::SecondsSquare;
use crate::units::traits::Initializable;
use crate::{impl_initializable, impl_raw_representable, units::SecondsCube};

#[repr(transparent)]
#[derive(Debug, Default, Copy, Clone, PartialEq, PartialOrd)]
pub struct Seconds(f64);

impl Seconds {
    pub fn squared(self) -> SecondsSquare {
        SecondsSquare::new(self.0 * self.0)
    }
    pub fn cubed(self) -> SecondsCube {
        SecondsCube::new(self.0 * self.0 * self.0)
    }
}

impl_initializable!(Seconds);
impl_raw_representable!(Seconds);
impl_units_arithmetics!(Seconds);
impl_debug_unit!(Seconds, "s");
impl_literal!(Seconds, seconds, SecondsLiteral);

impl Mul for Seconds {
    type Output = SecondsSquare;

    fn mul(self, rhs: Self) -> Self::Output {
        SecondsSquare::new(self.0 * rhs.0)
    }
}
