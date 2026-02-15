use crate::{
    simulator::types::{position_ned::PositionNed, vec2::Vec2, velocity_ned::VelocityNed},
    units::{
        angles::{Degrees, DegreesPerSecond},
        units::{Meters, Velocity},
    },
};

#[derive(Default)]
pub struct State {
    pub altitude: Meters,

    pub velocity_ned: VelocityNed,
    pub position_ned: PositionNed,

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
