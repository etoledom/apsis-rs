use crate::{
    simulator::types::{
        angular_velocity_3d::AngularVelocity3D, position_ned::PositionNed, quaternion::Quaternion,
        vec2::Vec2, velocity_ned::VelocityNED,
    },
    units::{
        angles::{Degrees, DegreesPerSecond, Radians},
        units::{Meters, Velocity},
    },
};

#[derive(Default)]
pub struct State {
    pub altitude: Meters,

    pub velocity_ned: VelocityNED,
    pub position_ned: PositionNed,

    pub attitude: Quaternion,
    pub angular_velocity_body: AngularVelocity3D,

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
