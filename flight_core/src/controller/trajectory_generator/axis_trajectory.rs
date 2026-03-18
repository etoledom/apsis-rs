use crate::{
    controller::{
        AxisTarget,
        trajectory_generator::s_curve_profile::{SCurveProfile, SCurveSetpoint},
    },
    units::{Acceleration, Meters, PerSecond, Seconds, Velocity, traits::RawRepresentable},
};

pub struct AxisTrajectory {
    profile: SCurveProfile,
    loiter_target: Option<Meters>,
    position_gain: PerSecond,
    cascade_delay: Seconds,
}

impl AxisTrajectory {
    pub fn new(profile: SCurveProfile, gain: PerSecond, cascade_delay: Seconds) -> Self {
        Self {
            profile,
            loiter_target: None,
            position_gain: gain,
            cascade_delay,
        }
    }

    pub fn update(&mut self, target: &AxisTarget, dt: Seconds) -> SCurveSetpoint {
        let vel_target = match target {
            AxisTarget::Velocity(velocity_target) => {
                self.loiter_target = None;
                *velocity_target
            }
            AxisTarget::Position(position_target) => {
                self.loiter_target = None;
                self.velocity_from_position_error(*position_target)
            }
            AxisTarget::Loiter => {
                if self.loiter_target.is_none() {
                    let stop = self.profile.stopping_position();
                    println!(
                        "LOITER CAPTURE: profile_vel={:.4}, profile_pos={:.4}, stopping_pos={:.4}",
                        self.profile.velocity().raw(),
                        self.profile.position().raw(),
                        stop.raw(),
                    );
                }

                let pos_target = *self.loiter_target.get_or_insert_with(|| {
                    let stop = self.profile.stopping_position();
                    let delay_distance = self.profile.velocity() * self.cascade_delay;
                    stop + delay_distance
                });

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

    pub fn position(&self) -> Meters {
        self.profile.position()
    }

    pub fn velocity(&self) -> Velocity {
        self.profile.velocity()
    }

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
}

#[cfg(test)]
mod tests {
    const EPSILON: f64 = 1e-10;

    use crate::units::{
        AccelerationLiteral, JerkLiteral, MetersLiteral, PerSecond, SecondsLiteral,
        VelocityLiteral, traits::RawRepresentable,
    };

    use super::*;

    #[test]
    fn axis_trajectory_velocity_mode() {
        // Velocity mode passes target directly to profile
        let profile = SCurveProfile::new(5.mps3(), 3.mps2(), 10.mps());
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), 0.seconds());
        let dt = 0.01.seconds();

        for _ in 0..500 {
            axis.update(&AxisTarget::Velocity(4.mps()), dt);
        }
        assert!((axis.velocity().raw() - 4.0).abs() < 0.05);
    }

    #[test]
    fn axis_trajectory_velocity_mode_clears_loiter() {
        let profile = SCurveProfile::new(5.mps3(), 3.mps2(), 10.mps());
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), 0.seconds());
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
        let profile = SCurveProfile::new(5.mps3(), 3.mps2(), 10.mps()).with_state(
            0.meters(),
            5.mps(),
            0.mps2(),
        );
        let expected_stop = profile.stopping_position();

        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), 0.seconds());
        axis.update(&AxisTarget::Loiter, 0.01.seconds());

        assert!((axis.loiter_target.unwrap().raw() - expected_stop.raw()).abs() < 1e-9);
    }

    #[test]
    fn axis_trajectory_loiter_converges_to_stop_position() {
        // Fly at 5 m/s, then loiter — should stop near the predicted position
        let profile = SCurveProfile::new(5.mps3(), 3.mps2(), 10.mps()).with_state(
            0.meters(),
            5.mps(),
            0.mps2(),
        );
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), 0.seconds());
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
        let profile = SCurveProfile::new(5.mps3(), 3.mps2(), 5.mps());
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), 0.seconds());
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
        let profile = SCurveProfile::new(5.mps3(), 3.mps2(), 10.mps()).with_state(
            0.meters(),
            4.mps(),
            0.mps2(),
        );
        let mut axis = AxisTrajectory::new(profile, PerSecond(0.5), 0.seconds());
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
