use primitives::{
    traits::UnitsArithmetics,
    units::{Acceleration, Meters, PerSecond, Seconds, SecondsLiteral, Velocity},
};

use crate::controller::{
    AxisTarget,
    trajectory_generator::{
        s_curve_profile::{SCurveProfile, SCurveSetpoint},
        trajectory_limits::TrajectoryLimits,
    },
};

pub struct AxisTrajectory {
    profile: SCurveProfile,
    loiter_target: Option<Meters>,
    position_gain: PerSecond,
    cascade_delay: Seconds,
    auto_limits: TrajectoryLimits,
    manual_limits: TrajectoryLimits,
}

impl AxisTrajectory {
    pub fn new(
        profile: SCurveProfile,
        gain: PerSecond,
        auto_limits: TrajectoryLimits,
        manual_limits: TrajectoryLimits,
    ) -> Self {
        Self {
            profile,
            loiter_target: None,
            position_gain: gain,
            cascade_delay: 0.seconds(),
            auto_limits,
            manual_limits,
        }
    }

    pub fn update(&mut self, target: &AxisTarget, dt: Seconds) -> SCurveSetpoint {
        let vel_target = match target {
            AxisTarget::Velocity(velocity_target) => {
                self.loiter_target = None;
                self.profile.set_trajectory_limits(self.manual_limits);
                *velocity_target
            }
            AxisTarget::Position(position_target) => {
                self.loiter_target = None;
                self.profile.set_trajectory_limits(self.auto_limits);
                self.velocity_from_position_error(*position_target)
            }
            AxisTarget::Loiter => {
                let pos_target = *self.loiter_target.get_or_insert_with(|| {
                    let stop = self.profile.stopping_position();
                    let delay_distance = self.profile.velocity() * self.cascade_delay;
                    stop + delay_distance
                });
                self.profile.set_trajectory_limits(self.auto_limits);

                self.velocity_from_position_error(pos_target)
            }
        };
        self.profile.update(vel_target, dt)
    }

    /// Simple P controller: position error × gain = velocity target,
    /// clamped to max velocity.
    fn velocity_from_position_error(&self, position_target: Meters) -> Velocity {
        let pos_error = position_target - self.profile.position();
        (pos_error * self.position_gain)
            .clamping(-self.profile.max_velocity(), self.profile.max_velocity())
    }

    #[cfg(test)]
    pub fn position(&self) -> Meters {
        self.profile.position()
    }

    #[cfg(test)]
    pub fn velocity(&self) -> Velocity {
        self.profile.velocity()
    }

    #[allow(dead_code)]
    pub fn acceleration(&self) -> Acceleration {
        self.profile.acceleration()
    }

    pub fn loiter_target(&self) -> Option<Meters> {
        self.loiter_target
    }

    pub fn reset(&mut self, position: Meters, velocity: Velocity, acceleration: Acceleration) {
        self.profile.reset(position, velocity, acceleration);
        self.loiter_target = None;
    }

    pub fn with_delay(self, cascade_delay: Seconds) -> Self {
        Self {
            cascade_delay,
            ..self
        }
    }
}

#[cfg(test)]
mod tests {
    const EPSILON: f64 = 1e-10;

    use primitives::prelude::*;

    use super::*;

    fn limits() -> TrajectoryLimits {
        TrajectoryLimits {
            max_velocity: 10.mps(),
            max_acceleration: 3.mps2(),
            max_jerk: 5.mps3(),
        }
    }

    #[test]
    fn axis_trajectory_velocity_mode() {
        // Velocity mode passes target directly to profile
        let profile = SCurveProfile::new();
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), limits(), limits());
        let dt = 0.01.seconds();

        for _ in 0..500 {
            axis.update(&AxisTarget::Velocity(4.mps()), dt);
        }
        assert!((axis.velocity().raw() - 4.0).abs() < 0.05);
    }

    #[test]
    fn axis_trajectory_velocity_mode_clears_loiter() {
        let profile = SCurveProfile::new();
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), limits(), limits());
        let dt = 0.01.seconds();

        // Enter loiter
        axis.update(&AxisTarget::Velocity(3.mps()), dt);
        axis.update(&AxisTarget::Loiter, dt);
        assert!(axis.loiter_target.is_some());

        // Back to velocity clears it
        axis.update(&AxisTarget::Velocity(2.mps()), dt);
        assert!(axis.loiter_target.is_none());
    }

    #[test]
    fn axis_trajectory_loiter_captures_stopping_position() {
        let profile = SCurveProfile::new().with_state(0.meters(), 5.mps(), 0.mps2());
        let expected_stop = profile.stopping_position();

        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), limits(), limits());
        axis.update(&AxisTarget::Loiter, 0.01.seconds());

        assert!(
            (axis.loiter_target.unwrap().raw() - expected_stop.raw()).abs() < 1e-9,
            "Should capture stopping possition. Captured: {}, expected: {}",
            axis.loiter_target.unwrap(),
            expected_stop
        );
    }

    #[test]
    fn axis_trajectory_loiter_converges_to_stop_position() {
        // Fly at 5 m/s, then loiter — should stop near the predicted position
        let profile = SCurveProfile::new().with_state(0.meters(), 5.mps(), 0.mps2());
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), limits(), limits());
        let dt = 0.01.seconds();

        let mut setpoint = SCurveSetpoint {
            position: 0.meters(),
            velocity: 0.mps(),
            acceleration: 0.mps2(),
        };
        for _ in 0..2000 {
            setpoint = axis.update(&AxisTarget::Loiter, dt);
        }

        let target = axis.loiter_target.unwrap();
        assert!((setpoint.position.raw() - target.raw()).abs() < 0.1);
        assert!(setpoint.velocity.raw().abs() < 0.05);
    }

    #[test]
    fn axis_trajectory_position_converges() {
        // From rest, go to position 20m
        let profile = SCurveProfile::new();
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), limits(), limits());
        let dt = 0.01.seconds();

        let mut setpoint = SCurveSetpoint {
            position: 0.meters(),
            velocity: 0.mps(),
            acceleration: 0.mps2(),
        };
        for _ in 0..5000 {
            setpoint = axis.update(&AxisTarget::Position(20.meters()), dt);
        }

        assert!((setpoint.position.raw() - 20.0).abs() < 0.1);
        assert!(setpoint.velocity.raw().abs() < 0.05);
    }

    #[test]
    fn axis_trajectory_loiter_target_is_stable() {
        // Once loiter captures, target doesn't change on subsequent ticks
        let profile = SCurveProfile::new().with_state(0.meters(), 4.mps(), 0.mps2());
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), limits(), limits());
        let dt = 0.01.seconds();

        axis.update(&AxisTarget::Loiter, dt);
        let first_target = axis.loiter_target.unwrap();

        for _ in 0..100 {
            axis.update(&AxisTarget::Loiter, dt);
        }
        let later_target = axis.loiter_target.unwrap();

        assert!((first_target.raw() - later_target.raw()).abs() < EPSILON);
    }
}
