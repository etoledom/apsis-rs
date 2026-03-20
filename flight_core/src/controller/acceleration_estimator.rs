use primitives::{
    frames::{AccelerationNed, Ned, VelocityNed},
    units::{Acceleration, PerSecond, Seconds},
};

use crate::controller::filters::low_pass_filter::LowPassFilter;

/// Estimates world-frame acceleration by numerically differentiating
/// velocity and low-pass filtering the result.
///
/// PX4 equivalent: vel_dot computation in MulticopterPositionControl::set_vehicle_states
pub struct AccelerationEstimator {
    prev_velocity: Option<VelocityNed>,
    filter_north: LowPassFilter<Acceleration>,
    filter_east: LowPassFilter<Acceleration>,
    filter_down: LowPassFilter<Acceleration>,
}

impl AccelerationEstimator {
    /// cutoff_hz: filter cutoff frequency.
    /// PX4 default for vel derivative: ~5 Hz (MPC_VELD_LP = 5.0)
    /// Lower = smoother but more lag. Higher = more responsive but noisier.
    /// For sim with clean data, 5-10 Hz is a good range.
    pub fn new(cutoff_hz: PerSecond) -> Self {
        Self {
            prev_velocity: None,
            filter_north: LowPassFilter::new(cutoff_hz),
            filter_east: LowPassFilter::new(cutoff_hz),
            filter_down: LowPassFilter::new(cutoff_hz),
        }
    }

    pub fn update(&mut self, velocity: VelocityNed, dt: Seconds) -> AccelerationNed {
        let raw = match self.prev_velocity {
            Some(prev) => AccelerationNed::new(
                (velocity.north() - prev.north()) / dt,
                (velocity.east() - prev.east()) / dt,
                (velocity.down() - prev.down()) / dt,
            ),
            None => AccelerationNed::zero(),
        };

        self.prev_velocity = Some(velocity);

        AccelerationNed::new(
            self.filter_north.update(raw.north(), dt),
            self.filter_east.update(raw.east(), dt),
            self.filter_down.update(raw.down(), dt),
        )
    }

    pub fn reset(&mut self) {
        self.prev_velocity = None;
        self.filter_north.reset();
        self.filter_east.reset();
        self.filter_down.reset();
    }
}
