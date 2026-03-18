use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

use primitives::{
    control::*,
    traits::{Initializable, RawRepresentable, UnitsArithmetics},
    units::{Acceleration, AngularVelocity, Meters, Radians, Seconds, Velocity},
};

use crate::controller::gain::{Converter, Gain, LinearGain};

pub trait Clampable<T> {
    fn clamped(self, limits: PIDLimits<T>) -> T;
}

impl Clampable<Pitch> for Pitch {
    fn clamped(self, limits: PIDLimits<Pitch>) -> Pitch {
        Pitch::clamp(self.raw().clamp(limits.min.raw(), limits.max.raw()))
    }
}

impl Clampable<Roll> for Roll {
    fn clamped(self, limits: PIDLimits<Roll>) -> Roll {
        Roll::clamp(self.raw().clamp(limits.min.raw(), limits.max.raw()))
    }
}

impl Clampable<Yaw> for Yaw {
    fn clamped(self, limits: PIDLimits<Yaw>) -> Yaw {
        Yaw::clamp(self.raw().clamp(limits.min.raw(), limits.max.raw()))
    }
}

macro_rules! impl_clampable {
    ($t:ty) => {
        impl Clampable<$t> for $t {
            fn clamped(self, limits: PIDLimits<$t>) -> $t {
                self.clamping(limits.min, limits.max)
            }
        }
    };
}

impl_clampable!(Velocity);
impl_clampable!(Acceleration);
impl_clampable!(AngularVelocity);

#[derive(Default, Clone, Copy)]
pub struct PIDLimits<T> {
    min: T,
    max: T,
}

#[derive(Default)]
pub struct PID<Error, Output, Kp, Ki, Kd> {
    pub kp: Kp,
    pub ki: Ki,
    pub kd: Kd,

    limits: Option<PIDLimits<Output>>,
    conditional_integration: bool,

    pub integral: Error,
    pub previous_error: Option<Error>,
}

impl<Error, Output, Kp, Ki, Kd> PID<Error, Output, Kp, Ki, Kd>
where
    Error: Copy
        + Add<Output = Error>
        + Sub<Output = Error>
        + Mul<f64, Output = Error>
        + Div<f64, Output = Error>
        + AddAssign
        + SubAssign
        + Default
        + Initializable,
    Output: Copy
        + Add<Output = Output>
        + Default
        + Clampable<Output>
        + Sub<Output = Output>
        + RawRepresentable
        + Neg<Output = Output>
        + Initializable,
    Kp: Gain<Error, Output> + Default + RawRepresentable,
    Ki: Gain<Error, Output> + Default + RawRepresentable,
    Kd: Gain<Error, Output> + Default + RawRepresentable,
{
    pub fn new(kp: impl Into<Kp>, ki: impl Into<Ki>, kd: impl Into<Kd>) -> Self {
        Self {
            kp: kp.into(),
            ki: ki.into(),
            kd: kd.into(),
            limits: None,
            integral: Error::default(),
            previous_error: None,
            conditional_integration: false,
        }
    }

    pub fn with_conditional_integration(self) -> Self {
        Self {
            conditional_integration: true,
            ..self
        }
    }

    pub fn update(&mut self, error: Error, dt: Seconds) -> Output {
        let p = self.kp.apply(error);

        let i = if self.ki.raw() > 0.0 {
            self.integral += error * dt.raw();
            self.ki.apply(self.integral)
        } else {
            Output::new(0.0)
        };

        let d = if self.kd.raw() > 0.0 {
            let derivative = if let Some(previous_error) = self.previous_error {
                (error - previous_error) / dt.raw()
            } else {
                Error::default()
            };

            self.previous_error = Some(error);
            self.kd.apply(derivative)
        } else {
            Output::new(0.0)
        };

        let raw_output = p + d + i;

        if let Some(limits) = self.limits {
            let clamped_output = raw_output.clamped(limits);

            if self.ki.raw() > 0.0 {
                let windup_error: Error = Converter::convert(raw_output - clamped_output);
                self.integral -= windup_error * dt.raw();
            }
            return clamped_output;
        }

        return raw_output;
    }

    pub fn with_limits(self, clamp: Output) -> Self {
        Self {
            limits: Some(PIDLimits {
                min: -clamp,
                max: clamp,
            }),
            ..self
        }
    }
}

pub type PositionPID = PID<Meters, Velocity, LinearGain, LinearGain, LinearGain>;
pub type AngularPID = PID<Radians, AngularVelocity, LinearGain, LinearGain, LinearGain>;
pub type VelocityPID = PID<Velocity, Acceleration, LinearGain, LinearGain, LinearGain>;
pub type RollRatePID = PID<AngularVelocity, Roll, LinearGain, LinearGain, LinearGain>;
pub type PitchRatePID = PID<AngularVelocity, Pitch, LinearGain, LinearGain, LinearGain>;
pub type YawRatePID = PID<AngularVelocity, Yaw, LinearGain, LinearGain, LinearGain>;

#[cfg(test)]
mod test {
    use crate::controller::gain::LinearGain;
    use approx::assert_relative_eq;
    use primitives::prelude::*;

    use super::*;

    #[test]
    fn test_pid_proportional_only() {
        let mut pid: PositionPID = PID {
            kp: LinearGain::new(2.0),
            ..Default::default()
        };

        let error = 5.meters();
        let output = pid.update(error, 0.seconds());

        assert_eq!(output, 10.mps())
    }

    #[test]
    fn test_pid_integral_accumulates() {
        let mut pid: PositionPID = PID {
            ki: LinearGain::new(1.0),
            ..Default::default()
        };

        let delta_t = 1.seconds();

        // updating 2 metters 2 times = 4
        pid.update(2.meters(), delta_t);
        let output = pid.update(2.meters(), delta_t);

        assert_eq!(output, 4.mps());
    }

    #[test]
    fn test_pid_derivative_computes_difference() {
        let mut pid: PositionPID = PID {
            kd: LinearGain::new(1.0),
            ..Default::default()
        };

        let delta_t = 1.seconds();

        pid.update(1.meters(), delta_t);
        let output = pid.update(3.meters(), delta_t);

        //(prev_err - err) / dt
        //(3m - 1m) / 1s = 2mps
        assert_eq!(output, 2.mps())
    }

    #[test]
    fn test_pid_non_derivative_on_first_loop() {
        let mut pid: PositionPID = PID {
            kd: LinearGain::new(1.0),
            ..Default::default()
        };

        let delta_t = 1.seconds();

        let output = pid.update(3.meters(), delta_t);

        assert_eq!(output, 0.mps())
    }

    #[test]
    fn windup_prevented_when_saturated() {
        let mut pid: PositionPID = PositionPID::new(0, 1, 0).with_limits(4.mps());

        let error = 10.meters();
        let dt = 0.1.seconds();

        for _ in 0..100 {
            pid.update(error, dt);
        }

        let integral_after_100 = pid.integral;

        for _ in 0..100 {
            pid.update(error, dt);
        }

        // Integral should not keep growing after saturation
        assert_relative_eq!(pid.integral.raw(), integral_after_100.raw(), epsilon = 1e-3);
    }

    #[test]
    fn response_is_immediate_after_saturation() {
        let mut pid: PositionPID = PositionPID::new(0, 1, 0).with_limits(4.mps());

        let dt = 0.1.seconds();

        // Saturate with large positive error
        for _ in 0..100 {
            pid.update(100.meters(), dt);
        }

        // Now reverse — should respond immediately
        for _ in 0..6 {
            pid.update(-100.meters(), dt);
        }
        let output = pid.update(-100.meters(), dt);
        assert!(
            output.raw() < 0.0,
            "should immediately respond to reversed error {}",
            output
        );
    }
}
