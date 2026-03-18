use crate::{
    simulator::types::{angular_damping::AngularDamping, drag::Drag, throttle::Throttle},
    units::{
        Acceleration, Seconds,
        angles::{AngularAcceleration, Degrees},
        consts::G_EARTH,
    },
};

pub trait Drone {
    // Attitude
    fn max_roll(&self) -> Degrees; // side tilt
    fn max_pitch(&self) -> Degrees; // Forward / backward tilt
    fn max_pitch_acceleration(&self) -> AngularAcceleration;
    fn max_roll_acceleration(&self) -> AngularAcceleration;
    fn max_yaw_acceleration(&self) -> AngularAcceleration;
    fn pitch_damping_coefficient(&self) -> AngularDamping;
    fn roll_damping_coefficient(&self) -> AngularDamping;
    fn yaw_damping_coefficient(&self) -> AngularDamping;

    // Thrust
    fn thrust_to_waight_ratio(&self) -> f64;
    fn max_thrust_acceleration(&self) -> Acceleration {
        G_EARTH * self.thrust_to_waight_ratio()
    }
    fn hover_throttle(&self) -> Throttle {
        Throttle::clamp(G_EARTH / self.max_thrust_acceleration())
    }
    fn motor_time_constant(&self) -> Seconds;

    // Drag
    fn drag_coefficient(&self) -> DragCoefficient;

    // others
    fn battery_drain_pct(&self, throttle: Throttle) -> f64;
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct DragCoefficient {
    pub vertical: Drag,
    pub horizontal: Drag,
    pub forward: Drag,
}
