use crate::{
    simulator::types::throttle::Throttle,
    units::{
        angles::Radians,
        units::{Meters, Velocity},
    },
};

pub trait Gain<Error, Output> {
    fn apply(&self, value: Error) -> Output;
}

#[derive(Clone, Copy, Default)]
pub struct LinearGain(f64);

impl LinearGain {
    pub fn new(value: f64) -> Self {
        LinearGain(value)
    }
    pub fn zero() -> Self {
        LinearGain(0.0)
    }
}

impl Gain<Meters, Velocity> for LinearGain {
    fn apply(&self, value: Meters) -> Velocity {
        Velocity(self.0 * value.0)
    }
}

impl Gain<Velocity, Throttle> for LinearGain {
    fn apply(&self, value: Velocity) -> Throttle {
        Throttle::clamp(self.0 * value.0)
    }
}

impl Gain<Velocity, Radians> for LinearGain {
    fn apply(&self, value: Velocity) -> Radians {
        Radians(self.0 * value.0)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn linear_gain_scales_correctly() {
        let gain = LinearGain::new(2.0);
        let distance = Meters(3.0);

        let output = gain.apply(distance);

        assert_relative_eq!(output.0, 6.0, epsilon = 1e-6);
    }
}
