use crate::{
    simulator::types::{angular_damping::AngularDamping, drag::Drag, yaw::Yaw},
    types::throttle::Throttle,
    units::{
        acceleration::Acceleration,
        angles::{AngularAcceleration, Degrees, DegreesPerSecond},
        consts::G_EARTH,
    },
};

pub trait Drone {
    fn max_heading_rate(&self) -> DegreesPerSecond;
    fn max_roll(&self) -> Degrees; // side tilt
    fn max_pitch(&self) -> Degrees; // Forward / backward tilt
    fn max_pitch_acceleration(&self) -> AngularAcceleration;
    fn max_roll_acceleration(&self) -> AngularAcceleration;
    fn thrust_to_waight_ratio(&self) -> f64;
    fn battery_drain_pct(&self, throttle: Throttle) -> f64;
    fn drag_coefficient(&self) -> DragCoefficient;
    fn pitch_damping_coefficient(&self) -> AngularDamping;
    fn roll_damping_coefficient(&self) -> AngularDamping;

    fn max_thrust_acceleration(&self) -> Acceleration {
        G_EARTH * self.thrust_to_waight_ratio()
    }

    #[allow(dead_code)] // Used for unit tests
    fn hover_throttle(&self) -> Throttle {
        Throttle::clamp(G_EARTH / self.max_thrust_acceleration())
    }

    fn heading_rate(&self, yaw: Yaw) -> DegreesPerSecond {
        self.max_heading_rate() * yaw.get()
    }
}

pub struct DragCoefficient {
    pub vertical: Drag,
    pub horizontal: Drag,
    pub forward: Drag,
}

#[cfg(test)]
mod tests {
    use crate::simulator::default_drone::DefaultDrone;

    use super::*;
    use approx::{self, assert_relative_eq};

    fn test_drone() -> impl Drone {
        DefaultDrone {}
    }

    #[test]
    fn test_heading_rate() {
        let drone = test_drone();
        assert_relative_eq!(drone.heading_rate(Yaw::clamp(0.5)).0, 22.5, epsilon = 1e-2);
    }
}
