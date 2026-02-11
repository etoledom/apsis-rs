use crate::simulator::types::unit_interval::UnitInterval;

#[derive(Clone, Copy, Default)]
pub struct Throttle(UnitInterval);

impl Throttle {
    pub fn new(value: f64) -> Option<Self> {
        if let Some(value) = UnitInterval::new(value) {
            Some(Throttle(value))
        } else {
            None
        }
    }

    pub fn max() -> Self {
        Self::clamp(1.0)
    }

    pub fn clamp(value: f64) -> Self {
        Self(UnitInterval::clamp(value))
    }

    #[inline(always)]
    pub fn get(self) -> f64 {
        self.0.get()
    }
}
