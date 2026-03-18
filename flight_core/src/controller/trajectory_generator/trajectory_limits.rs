use primitives::units::{Acceleration, Jerk, Seconds, Velocity};

#[derive(Debug, Default, Clone, Copy)]
pub struct TrajectoryLimits {
    pub max_velocity: Velocity,
    pub max_acceleration: Acceleration,
    pub max_jerk: Jerk,
    pub cascade_delay: Seconds,
}
