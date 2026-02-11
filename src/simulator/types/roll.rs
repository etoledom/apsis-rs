use crate::simulator::types::signed_unit_interval::SignedUnitInterval;

#[derive(Clone, Copy, Default)]
pub struct Roll(SignedUnitInterval);

impl Roll {
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
