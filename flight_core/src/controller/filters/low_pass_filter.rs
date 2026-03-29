use std::{
    f64::consts::PI,
    ops::{Add, Mul},
};

use primitives::{
    traits::{Initializable, RawRepresentable},
    units::{PerSecond, Seconds},
};

/// First-order IIR low-pass filter (exponential moving average).
///
/// Transfer function: y[n] = α * x[n] + (1 - α) * y[n-1]
/// where α = dt / (rc + dt), rc = 1 / (2π * cutoff_freq)
///
pub struct LowPassFilter<T> {
    cutoff_hz: PerSecond,
    state: Option<T>,
}

impl<T> LowPassFilter<T>
where
    T: Add<T, Output = T> + Mul<f64, Output = T> + RawRepresentable + Initializable + Clone + Copy,
{
    pub fn new(cutoff_hz: PerSecond) -> Self {
        Self {
            cutoff_hz,
            state: None,
        }
    }

    pub fn update(&mut self, sample: T, dt: Seconds) -> T {
        let rc = 1.0 / (self.cutoff_hz * 2.0 * PI);
        let alpha = dt / (rc + dt);

        let filtered = match self.state {
            Some(prev) => sample * alpha + prev * (1.0 - alpha),
            None => sample,
        };

        self.state = Some(filtered);
        filtered
    }
}
