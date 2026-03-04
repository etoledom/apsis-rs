use crate::{
    inputs::Inputs,
    simulator::types::{
        angular_velocity_3d::AngularVelocity3D, position_ned::PositionNed, quaternion::Quaternion,
        vec2::Vec2, velocity_ned::VelocityNED,
    },
    units::units::{Meters, Velocity},
};

#[derive(Clone, Copy)]
pub struct State {
    pub altitude: Meters,

    pub velocity_ned: VelocityNED,
    pub position_ned: PositionNed,

    pub attitude: Quaternion,
    pub angular_velocity_body: AngularVelocity3D,

    pub last_inputs: Inputs,

    pub battery_pct: f64,
    pub latitude: f64,
    pub longitude: f64,
}

impl Default for State {
    fn default() -> Self {
        Self {
            battery_pct: 100.0,

            altitude: Default::default(),
            velocity_ned: Default::default(),
            position_ned: Default::default(),
            attitude: Default::default(),
            angular_velocity_body: Default::default(),
            last_inputs: Default::default(),
            latitude: Default::default(),
            longitude: Default::default(),
        }
    }
}

impl Default for Vec2<Velocity> {
    fn default() -> Self {
        Self {
            x: Default::default(),
            y: Default::default(),
        }
    }
}
