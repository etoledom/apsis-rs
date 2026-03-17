use std::ops::Add;

use crate::units::{acceleration::Acceleration, traits::RawRepresentable};

#[repr(transparent)]
#[derive(Copy, Clone, Default, Debug, PartialEq, PartialOrd)]
pub struct AccelerationSquare(f64); // m²/s⁴

impl AccelerationSquare {
    pub fn new(value: impl Into<f64>) -> Self {
        Self(value.into())
    }

    pub fn sqrt(self) -> Acceleration {
        Acceleration(self.0.sqrt())
    }
}

impl RawRepresentable for AccelerationSquare {
    fn raw(&self) -> f64 {
        self.0
    }
}

impl Add for AccelerationSquare {
    type Output = AccelerationSquare;

    fn add(self, rhs: Self) -> Self::Output {
        AccelerationSquare(self.0 + rhs.0)
    }
}
