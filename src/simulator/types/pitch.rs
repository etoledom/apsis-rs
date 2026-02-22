use std::ops::Add;

use crate::simulator::types::signed_unit_interval::SignedUnitInterval;

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
