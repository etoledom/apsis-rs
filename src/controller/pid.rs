use std::{
    marker::PhantomData,
    ops::{Add, AddAssign, Div, Mul, Sub},
};

use crate::{
    simulator::types::throttle::Throttle,
    units::units::{Meters, Seconds, Velocity},
};

pub struct PID<Error, Output, Kp, Ki, Kd> {
    kp: Kp,
    ki: Ki,
    kd: Kd,

    integral: Error,
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
    pub fn new(kp: Kp, ki: Ki, kd: Kd) -> Self {
        Self {
            kp,
            ki,
            kd,
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

// ==================
//

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
