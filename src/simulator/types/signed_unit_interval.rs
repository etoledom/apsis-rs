#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SignedUnitInterval(f64);

impl SignedUnitInterval {
    pub fn new(value: f64) -> Option<Self> {
        if (-1.0..=1.0).contains(&value) {
            Some(Self(value))
        } else {
            None
        }
    }

    pub fn clamp(value: f64) -> Self {
        Self(value.clamp(-1.0, 1.0))
    }

    #[inline(always)]
    pub fn get(self) -> f64 {
        self.0
    }
}

impl Default for SignedUnitInterval {
    fn default() -> Self {
        Self(Default::default())
    }
}
