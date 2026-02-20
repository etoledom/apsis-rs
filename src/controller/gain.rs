use crate::{
    simulator::types::{pitch::Pitch, roll::Roll, throttle::Throttle, yaw::Yaw},
    units::{
        acceleration::Acceleration,
        angles::{AngularVelocity, Radians},
        units::{Meters, Velocity},
    },
};

pub trait Gain<Error, Output> {
    fn apply(&self, value: Error) -> Output;
}

#[derive(Clone, Copy, Default)]
pub struct LinearGain(pub f64);

impl LinearGain {
    pub fn new(value: f64) -> Self {
        LinearGain(value)
    }
    pub fn zero() -> Self {
        LinearGain(0.0)
    }
}

impl From<f64> for LinearGain {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl From<i32> for LinearGain {
    fn from(value: i32) -> Self {
        Self(value as f64)
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

impl Gain<Radians, Pitch> for LinearGain {
    fn apply(&self, value: Radians) -> Pitch {
        Pitch::clamp(self.0 * value.0)
    }
}

impl Gain<Velocity, Acceleration> for LinearGain {
    fn apply(&self, value: Velocity) -> Acceleration {
        Acceleration(self.0 * value.0)
    }
}

impl Gain<Radians, AngularVelocity> for LinearGain {
    fn apply(&self, value: Radians) -> AngularVelocity {
        (self.0 * value.0).into()
    }
}

impl Gain<AngularVelocity, Roll> for LinearGain {
    fn apply(&self, value: AngularVelocity) -> Roll {
        Roll::clamp(self.0 * value.raw())
    }
}

impl Gain<AngularVelocity, Pitch> for LinearGain {
    fn apply(&self, value: AngularVelocity) -> Pitch {
        Pitch::clamp(self.0 * value.raw())
    }
}

impl Gain<AngularVelocity, Yaw> for LinearGain {
    fn apply(&self, value: AngularVelocity) -> Yaw {
        Yaw::clamp(self.0 * value.raw())
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
