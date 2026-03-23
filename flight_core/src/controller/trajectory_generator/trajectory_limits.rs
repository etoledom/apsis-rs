use primitives::units::{Acceleration, Jerk, Velocity};

#[derive(Debug, Default, Clone, Copy)]
pub struct TrajectoryLimits {
    pub max_velocity: Velocity,
    pub max_acceleration: Acceleration,
    pub max_jerk: Jerk,
}
