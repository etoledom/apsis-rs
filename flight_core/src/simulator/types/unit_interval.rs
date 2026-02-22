use std::ops::{Add, AddAssign};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UnitInterval(f64);

impl UnitInterval {
    pub fn new(value: f64) -> Option<Self> {
        if (0.0..=1.0).contains(&value) {
            Some(Self(value))
        } else {
            None
        }
    }

    pub fn clamp(value: f64) -> Self {
        Self(value.clamp(0.0, 1.0))
    }

    #[inline(always)]
    pub fn get(self) -> f64 {
        self.0
    }
}

impl Default for UnitInterval {
    fn default() -> Self {
        Self(Default::default())
    }
}

impl AddAssign for UnitInterval {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Add for UnitInterval {
    type Output = UnitInterval;

    fn add(self, rhs: Self) -> Self::Output {
        UnitInterval(self.0 + rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let success_values = [0.0, 0.3, 0.5, 0.99999, 1.0];

        for unit_interval in success_values.map(|v| UnitInterval::new(v)) {
            assert!(unit_interval.is_some());
        }

        let unallowed_values = [1.000001, -0.000001, 2.0, -0.4, -10.3];

        for unit_interval in unallowed_values.map(|v| UnitInterval::new(v)) {
            assert!(unit_interval.is_none());
        }
    }
}
