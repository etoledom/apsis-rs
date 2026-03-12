use std::ops::{Add, Neg, Sub};

use crate::{
    simulator::types::signed_unit_interval::SignedUnitInterval, units::traits::RawRepresentable,
};

#[derive(Clone, Copy, Default)]
pub struct Pitch(SignedUnitInterval);

impl Pitch {
    pub fn new(value: f64) -> Option<Self> {
        if let Some(value) = SignedUnitInterval::new(value) {
            Some(Self(value))
        } else {
            None
        }
    }

    pub fn clamp(value: f64) -> Self {
        Self(SignedUnitInterval::clamp(value))
    }

    #[inline(always)]
    pub fn get(self) -> f64 {
        self.0.get()
    }

    pub fn zero() -> Self {
        Self(SignedUnitInterval::clamp(0.0))
    }
}

impl Add for Pitch {
    type Output = Pitch;

    fn add(self, rhs: Self) -> Self::Output {
        Pitch::clamp(self.0.get() + rhs.0.get())
    }
}

impl Sub<Pitch> for Pitch {
    type Output = Pitch;

    fn sub(self, rhs: Pitch) -> Self::Output {
        Pitch::clamp(self.0.get() - rhs.0.get())
    }
}

impl RawRepresentable for Pitch {
    fn raw(&self) -> f64 {
        self.0.get()
    }
}

impl Neg for Pitch {
    type Output = Pitch;

    fn neg(self) -> Self::Output {
        Pitch(SignedUnitInterval::clamp(-self.0.get()))
    }
}
