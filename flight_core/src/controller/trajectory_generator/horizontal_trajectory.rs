use crate::{
    Vec2, Vec3,
    controller::{
        HorizontalTarget,
        trajectory_generator::{
            s_curve_profile::{SCurveProfile, SCurveSetpoint},
            trajectory_limits::TrajectoryLimits,
        },
    },
    units::{
        Meters, PerSecond, Seconds, Velocity, acceleration::Acceleration, traits::RawRepresentable,
    },
};

pub struct HorizontalTrajectory {
    /// 1D S-curve for horizontal speed (magnitude)
    profile: SCurveProfile,
    /// Current direction unit vector (north, east). None when stationary.
    direction: Option<(f64, f64)>,
    /// Captured loiter position (north, east)
    loiter_target: Option<(Meters, Meters)>,
    /// P gain for position → velocity conversion
    position_gain: PerSecond,
    /// Cascade delay for loiter stopping distance
    cascade_delay: Seconds,
}

pub struct HorizontalSetpoint {
    pub north: SCurveSetpoint,
    pub east: SCurveSetpoint,
}

impl HorizontalTrajectory {
    pub fn new(limits: &TrajectoryLimits, position_gain: PerSecond) -> Self {
        Self {
            profile: SCurveProfile::new(
                limits.max_jerk,
                limits.max_acceleration,
                limits.max_velocity,
            ),
            direction: None,
            loiter_target: None,
            position_gain,
            cascade_delay: limits.cascade_delay,
        }
    }

    pub fn update(
        &mut self,
        target: &HorizontalTarget,
        current_position: (Meters, Meters), // (north, east)
        current_velocity: (Velocity, Velocity),
        limits: &TrajectoryLimits,
        dt: Seconds,
    ) -> HorizontalSetpoint {
        let target_speed = match target {
            HorizontalTarget::Velocity { north, east } => {
                if self.loiter_target.is_some() {
                    self.sync_to_current(current_velocity);
                }
                self.loiter_target = None;
                let speed = ((north.raw()).powi(2) + (east.raw()).powi(2)).sqrt();
                if speed > 1e-6 {
                    self.direction = Some((north.raw() / speed, east.raw() / speed));
                }
                Velocity(speed)
            }
            HorizontalTarget::Loiter => {
                if self.loiter_target.is_none() {
                    let target = self.capture_loiter_target(current_position);
                    self.loiter_target = Some(target);
                }
                let loiter_pos = self.loiter_target.unwrap();
                self.velocity_from_position_error(current_position, loiter_pos, limits)
            }
            HorizontalTarget::Position { north, east } => {
                self.loiter_target = None;
                self.velocity_from_position_error(current_position, (*north, *east), limits)
            }
        };

        // Run 1D S-curve on speed magnitude
        let setpoint = self.profile.update(target_speed, dt);

        // Project back to north/east using direction
        let (dir_n, dir_e) = self.direction.unwrap_or((1.0, 0.0));

        HorizontalSetpoint {
            north: SCurveSetpoint {
                position: Meters(setpoint.position.raw() * dir_n),
                velocity: Velocity(setpoint.velocity.raw() * dir_n),
                acceleration: Acceleration(setpoint.acceleration.raw() * dir_n),
            },
            east: SCurveSetpoint {
                position: Meters(setpoint.position.raw() * dir_e),
                velocity: Velocity(setpoint.velocity.raw() * dir_e),
                acceleration: Acceleration(setpoint.acceleration.raw() * dir_e),
            },
        }
    }

    fn sync_to_current(&mut self, current_velocity: (Velocity, Velocity)) {
        let speed =
            ((current_velocity.0.raw()).powi(2) + (current_velocity.1.raw()).powi(2)).sqrt();
        if speed > 1e-6 {
            self.direction = Some((
                current_velocity.0.raw() / speed,
                current_velocity.1.raw() / speed,
            ));
        }
        // Sync profile speed to current horizontal speed
        self.profile = self.profile.with_state(
            self.profile.position(),
            Velocity(speed),
            self.profile.acceleration(),
        );
    }

    fn capture_loiter_target(&self, current_position: (Meters, Meters)) -> (Meters, Meters) {
        let stop = self.profile.stopping_position();
        let delay_distance = self.profile.velocity() * self.cascade_delay;
        let total_stop_distance = stop + delay_distance;

        let (dir_n, dir_e) = self.direction.unwrap_or((1.0, 0.0));
        (
            Meters(current_position.0.raw() + total_stop_distance.raw() * dir_n),
            Meters(current_position.1.raw() + total_stop_distance.raw() * dir_e),
        )
    }

    fn velocity_from_position_error(
        &mut self,
        current_position: (Meters, Meters),
        target_position: (Meters, Meters),
        limits: &TrajectoryLimits,
    ) -> Velocity {
        let error_n = target_position.0 - current_position.0;
        let error_e = target_position.1 - current_position.1;
        let distance = ((error_n.raw()).powi(2) + (error_e.raw()).powi(2)).sqrt();

        if distance > 1e-6 {
            self.direction = Some((error_n.raw() / distance, error_e.raw() / distance));
        }

        // P controller: distance → speed, clamped to max velocity
        Velocity(distance * self.position_gain.0)
            .clamping(-limits.max_velocity, limits.max_velocity)
    }

    pub fn loiter_target(&self) -> Option<(Meters, Meters)> {
        self.loiter_target.clone()
    }
}
