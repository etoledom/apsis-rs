use std::{
    marker::PhantomData,
    ops::{Add, AddAssign, Div, Mul, Sub},
};

use crate::{
    controller::gain::{Gain, LinearGain},
    simulator::types::{pitch::Pitch, roll::Roll, throttle::Throttle, yaw::Yaw},
    units::{
        acceleration::Acceleration,
        angles::{AngularVelocity, Radians},
        units::{Meters, Seconds, Velocity},
    },
};

#[derive(Default)]
pub struct PID<Error, Output, Kp, Ki, Kd> {
    pub kp: Kp,
    pub ki: Ki,
    pub kd: Kd,

    pub integral: Error,
    previous_error: Option<Error>,

    _output: PhantomData<Output>, // Output is only used in the trait
}

impl<Error, Output, Kp, Ki, Kd> PID<Error, Output, Kp, Ki, Kd>
where
    Error: Copy
        + Add<Output = Error>
        + Sub<Output = Error>
        + Mul<f64, Output = Error>
        + Div<f64, Output = Error>
        + AddAssign
        + Default,
    Output: Copy + Add<Output = Output> + Default,
    Kp: Gain<Error, Output> + Default,
    Ki: Gain<Error, Output> + Default,
    Kd: Gain<Error, Output> + Default,
{
    pub fn new(kp: impl Into<Kp>, ki: impl Into<Ki>, kd: impl Into<Kd>) -> Self {
        Self {
            kp: kp.into(),
            ki: ki.into(),
            kd: kd.into(),
            integral: Error::default(),
            previous_error: None,
            _output: PhantomData,
        }
    }

    pub fn update(&mut self, error: Error, dt: Seconds) -> Output {
        let p = self.kp.apply(error);

        self.integral += error * dt.0;
        let i = self.ki.apply(self.integral);

        let derivative = if let Some(previous_error) = self.previous_error {
            (error - previous_error) / dt.0
        } else {
            Error::default()
        };

        let d = self.kd.apply(derivative);

        self.previous_error = Some(error);

        return p + i + d;
    }
}

pub type AltitudePID = PID<Meters, Velocity, LinearGain, LinearGain, LinearGain>;
pub type ClimbPID = PID<Velocity, Throttle, LinearGain, LinearGain, LinearGain>;

pub type PitchPID = PID<Radians, AngularVelocity, LinearGain, LinearGain, LinearGain>;
pub type RollPID = PID<Radians, AngularVelocity, LinearGain, LinearGain, LinearGain>;
pub type YawPID = PID<Radians, AngularVelocity, LinearGain, LinearGain, LinearGain>;

pub type VelocityNorthPID = PID<Velocity, Acceleration, LinearGain, LinearGain, LinearGain>;
pub type VelocityEastPID = PID<Velocity, Acceleration, LinearGain, LinearGain, LinearGain>;
pub type VelocityDownPID = PID<Velocity, Acceleration, LinearGain, LinearGain, LinearGain>;

pub type RollRatePID = PID<AngularVelocity, Roll, LinearGain, LinearGain, LinearGain>;
pub type PitchRatePID = PID<AngularVelocity, Pitch, LinearGain, LinearGain, LinearGain>;
pub type YawRatePID = PID<AngularVelocity, Yaw, LinearGain, LinearGain, LinearGain>;

#[cfg(test)]
mod test {
    use crate::{
        controller::gain::LinearGain,
        units::units::{MettersLiteral, SecondsLiteral, VelocityLiteral},
    };

    use super::*;

    #[test]
    fn test_pid_proportional_only() {
        let mut pid: AltitudePID = PID {
            kp: LinearGain::new(2.0),
            ..Default::default()
        };

        let error = 5.meters();
        let output = pid.update(error, 0.seconds());

        assert_eq!(output, 10.mps())
    }

    #[test]
    fn test_pid_integral_accumulates() {
        let mut pid: AltitudePID = PID {
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
        let mut pid: AltitudePID = PID {
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
        let mut pid: AltitudePID = PID {
            kd: LinearGain::new(1.0),
            ..Default::default()
        };

        let delta_t = 1.seconds();

        let output = pid.update(3.meters(), delta_t);

        assert_eq!(output, 0.mps())
    }
}
