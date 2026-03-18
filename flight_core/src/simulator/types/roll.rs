use std::ops::{Add, Neg, Sub};

use crate::{
    simulator::types::signed_unit_interval::SignedUnitInterval,
    units::traits::{Initializable, RawRepresentable},
};

#[derive(Clone, Copy, Default)]
pub struct Roll(SignedUnitInterval);

impl Roll {
    pub fn clamp(value: f64) -> Self {
        Self(SignedUnitInterval::clamp(value))
    }

    #[inline(always)]
    pub fn get(self) -> f64 {
        self.0.get()
    }
}

impl Initializable for Roll {
    fn new(value: impl Into<f64>) -> Self {
        Self::clamp(value.into())
    }
}

impl Add for Roll {
    type Output = Roll;

    fn add(self, rhs: Self) -> Self::Output {
        Roll::clamp(self.get() + rhs.get())
    }
}

impl Sub<Roll> for Roll {
    type Output = Roll;

    fn sub(self, rhs: Roll) -> Self::Output {
        Roll::clamp(self.get() - rhs.get())
    }
}

impl RawRepresentable for Roll {
    fn raw(&self) -> f64 {
        self.0.get()
    }
}

impl Neg for Roll {
    type Output = Roll;

    fn neg(self) -> Self::Output {
        Roll(SignedUnitInterval::clamp(-self.0.get()))
    }
}
