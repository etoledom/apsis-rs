use std::ops::{Add, Neg, Sub};

use crate::{
    simulator::types::signed_unit_interval::SignedUnitInterval, units::traits::RawRepresentable,
};

#[derive(Clone, Copy, Default)]
pub struct Yaw(SignedUnitInterval);

impl Yaw {
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
}

impl Add for Yaw {
    type Output = Yaw;

    fn add(self, rhs: Self) -> Self::Output {
        Yaw::clamp(self.get() + rhs.get())
    }
}

impl Sub<Yaw> for Yaw {
    type Output = Yaw;

    fn sub(self, rhs: Yaw) -> Self::Output {
        Yaw::clamp(self.get() - rhs.get())
    }
}

impl RawRepresentable for Yaw {
    fn raw(&self) -> f64 {
        self.0.get()
    }
}

impl Neg for Yaw {
    type Output = Yaw;

    fn neg(self) -> Self::Output {
        Yaw(SignedUnitInterval::clamp(-self.0.get()))
    }
}
