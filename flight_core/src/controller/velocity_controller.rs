use primitives::{
    frames::{AccelerationNed, Ned, VelocityNed},
    traits::RawRepresentable,
    units::{AccelerationLiteral, Seconds},
};

use crate::controller::pids::gain::{Gain, LinearGain};

struct LinearNedGain {
    north: LinearGain,
    east: LinearGain,
    down: LinearGain,
}

impl LinearNedGain {
    fn new(north: f64, east: f64, down: f64) -> Self {
        Self {
            north: north.into(),
            east: east.into(),
            down: down.into(),
        }
    }
    fn apply(&self, vel: VelocityNed) -> AccelerationNed {
        AccelerationNed::new(
            self.north.apply(vel.north()),
            self.east.apply(vel.east()),
            self.down.apply(vel.down()),
        )
    }
}

#[derive(Debug, Default)]
struct DampingGainNed {
    north: f64,
    east: f64,
    down: f64,
}

impl DampingGainNed {
    fn apply(&self, acc: AccelerationNed) -> AccelerationNed {
        AccelerationNed::new(
            self.north * acc.north(),
            self.east * acc.east(),
            self.down * acc.down(),
        )
    }
}

pub struct Integral {
    ki: LinearNedGain,
    accumulator: AccelerationNed,
}

impl Integral {
    fn new(north: f64, east: f64, down: f64) -> Self {
        Self {
            ki: LinearNedGain::new(north, east, down),
            accumulator: AccelerationNed::zero(),
        }
    }
    fn update(
        &mut self,
        vel_error: VelocityNed,
        saturation_error: AccelerationNed,
        dt: Seconds,
    ) -> AccelerationNed {
        let integral = self.ki.apply(vel_error);

        if saturation_error.north().raw() * vel_error.north().raw() <= 0.0 {
            self.accumulator.add_north(integral.north() * dt.raw());
        }
        if saturation_error.east().raw() * vel_error.east().raw() <= 0.0 {
            self.accumulator.add_east(integral.east() * dt.raw());
        }
        if saturation_error.down().raw() * vel_error.down().raw() <= 0.0 {
            self.accumulator.add_down(integral.down() * dt.raw());
        }
        self.accumulator = self
            .accumulator
            .clamping_down(3.mps2())
            .clamping_north(3.mps2())
            .clamping_east(3.mps2());

        self.accumulator
    }
}

pub struct VelocityController {
    kp: LinearNedGain,
    kd: DampingGainNed,
    integral: Integral,
}

impl VelocityController {
    pub fn new() -> Self {
        Self {
            kp: LinearNedGain::new(1.8, 1.8, 4.0),
            kd: DampingGainNed {
                north: 0.2,
                east: 0.2,
                down: 0.0,
            },
            integral: Integral::new(0.4, 0.4, 0.2),
        }
    }
    pub fn update(
        &mut self,
        v_current: VelocityNed,
        v_target: VelocityNed,
        acc_current: AccelerationNed,
        acc_ff: AccelerationNed,
        saturation_error: AccelerationNed,
        dt: Seconds,
    ) -> AccelerationNed {
        let error = v_target - v_current;

        let p = self.kp.apply(error);
        let i = self.integral.update(error, saturation_error, dt);
        let d = self.kd.apply(acc_current);

        acc_ff + p + i - d
    }
}
