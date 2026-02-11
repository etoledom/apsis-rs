use crate::{
    simulator::{phase::Phase, types::vec2::Vec2},
    units::{
        angles::{Degrees, DegreesPerSecond},
        units::{Meters, Seconds, Velocity},
    },
};

#[derive(Default)]
pub struct State {
    pub phase: Phase,
    pub phase_start_time: Seconds,
    pub altitude: Meters,
    pub vertical_velocity: Velocity,
    pub ground_speed_ned: Vec2<Velocity>,
    pub heading: Degrees,
    pub heading_rate: DegreesPerSecond,
    pub battery_pct: f64,
    pub latitude: f64,
    pub longitude: f64,
}

impl Default for Vec2<Velocity> {
    fn default() -> Self {
        Self {
            x: Default::default(),
            y: Default::default(),
        }
    }
}
