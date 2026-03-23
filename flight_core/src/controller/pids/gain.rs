use primitives::{
    control::*,
    frames::{AccelerationNed, Ned, VelocityNed},
    traits::{Initializable, RawRepresentable},
    units::{Acceleration, AngularVelocity, Meters, Radians, Velocity},
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
    pub fn gain(&self) -> f64 {
        self.0
    }
}

impl RawRepresentable for LinearGain {
    fn raw(&self) -> f64 {
        self.0
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

trait NedVector {}
impl NedVector for VelocityNed {}
impl NedVector for AccelerationNed {}

impl<Error, Output> Gain<Error, Output> for LinearGain
where
    Error: Ned<Velocity> + NedVector,
    Output: Ned<Acceleration> + NedVector,
{
    fn apply(&self, value: Error) -> Output {
        Output::new(
            Acceleration::new(value.north().raw() * self.0),
            Acceleration::new(value.east().raw() * self.0),
            Acceleration::new(value.down().raw() * self.0),
        )
    }
}

impl Gain<Meters, Velocity> for LinearGain {
    fn apply(&self, value: Meters) -> Velocity {
        Velocity::new(self.0 * value.raw())
    }
}

impl Gain<Velocity, Throttle> for LinearGain {
    fn apply(&self, value: Velocity) -> Throttle {
        Throttle::clamp(self.0 * value.raw())
    }
}

impl Gain<Velocity, Radians> for LinearGain {
    fn apply(&self, value: Velocity) -> Radians {
        Radians(self.0 * value.raw())
    }
}

impl Gain<Radians, Pitch> for LinearGain {
    fn apply(&self, value: Radians) -> Pitch {
        Pitch::clamp(self.0 * value.0)
    }
}

impl Gain<Velocity, Acceleration> for LinearGain {
    fn apply(&self, value: Velocity) -> Acceleration {
        Acceleration::new(self.0 * value.raw())
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

// --- Converter ---
//

#[derive(Default)]
pub struct Converter;

impl Converter {
    pub fn convert<In, Out>(input: In) -> Out
    where
        In: RawRepresentable,
        Out: Initializable,
    {
        Out::new(input.raw())
    }
}

impl Gain<Acceleration, Velocity> for LinearGain {
    fn apply(&self, value: Acceleration) -> Velocity {
        Velocity::new(self.0 * value.raw())
    }
}

impl Gain<Pitch, AngularVelocity> for LinearGain {
    fn apply(&self, value: Pitch) -> AngularVelocity {
        AngularVelocity::new(self.0 * value.raw())
    }
}

impl Gain<Roll, AngularVelocity> for LinearGain {
    fn apply(&self, value: Roll) -> AngularVelocity {
        AngularVelocity::new(self.0 * value.raw())
    }
}

impl Gain<Yaw, AngularVelocity> for LinearGain {
    fn apply(&self, value: Yaw) -> AngularVelocity {
        AngularVelocity::new(self.0 * value.raw())
    }
}

impl Gain<Velocity, Meters> for LinearGain {
    fn apply(&self, value: Velocity) -> Meters {
        Meters::new(self.0 * value.raw())
    }
}

impl Gain<AngularVelocity, Radians> for LinearGain {
    fn apply(&self, value: AngularVelocity) -> Radians {
        Radians(self.0 * value.raw())
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use primitives::units::MetersLiteral;

    use super::*;

    #[test]
    fn linear_gain_scales_correctly() {
        let gain = LinearGain::new(2.0);
        let distance = 3.meters();

        let output = gain.apply(distance);

        assert_relative_eq!(output.raw(), 6.0, epsilon = 1e-6);
    }
}
