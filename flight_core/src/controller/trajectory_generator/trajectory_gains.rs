use crate::units::PerSecond;

pub struct TrajectoryGains {
    pub position: PerSecond,
    pub velocity: PerSecond,
    pub acceleration: PerSecond,
}
