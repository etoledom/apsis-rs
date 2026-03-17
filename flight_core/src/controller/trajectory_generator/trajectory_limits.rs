use crate::{
    Jerk,
    units::{Seconds, Velocity, acceleration::Acceleration},
};

#[derive(Debug, Default, Clone, Copy)]
pub struct TrajectoryLimits {
    pub max_velocity: Velocity,
    pub max_acceleration: Acceleration,
    pub max_jerk: Jerk,
    pub cascade_delay: Seconds,
}
