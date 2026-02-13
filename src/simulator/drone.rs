use crate::{
    simulator::types::{
        acceleration_3d::BodyFrameAcceleration, drag::Drag, pitch::Pitch, roll::Roll, yaw::Yaw,
    },
    types::throttle::Throttle,
    units::{
        acceleration::Acceleration,
        angles::{Degrees, DegreesPerSecond},
        consts::G_EARTH,
        units::Kilograms,
    },
};

pub trait Drone {
    fn max_heading_rate(&self) -> DegreesPerSecond;
    fn max_roll(&self) -> Degrees; // side tilt
    fn max_pitch(&self) -> Degrees; // Forward / backward tilt
    fn thrust_to_waight_ratio(&self) -> f64;
    fn mass(&self) -> Kilograms;
    fn battery_drain_pct(&self, throttle: Throttle) -> f64;
    fn drag_coefficient(&self) -> DragCoefficient;

    fn max_thrust_acceleration(&self) -> Acceleration {
        G_EARTH * self.thrust_to_waight_ratio()
    }

    fn hover_throttle(&self) -> Throttle {
        Throttle::clamp(G_EARTH / self.max_thrust_acceleration())
    }

    fn heading_rate(&self, yaw: Yaw) -> DegreesPerSecond {
        self.max_heading_rate() * yaw.get()
    }

    // Right stick (typical Mode 2):
    //   ↑ Pitch forward  (nose down → drone moves forward)
    //   ↓ Pitch backward (nose up  → drone moves backward)
    //   ← Roll left      (right side down)
    //   → Roll right     (left side down)
    fn body_frame_acceleration(
        &self,
        throttle: Throttle,
        pitch: Pitch,
        roll: Roll,
    ) -> BodyFrameAcceleration {
        let pitch = (self.max_pitch() * pitch.get()).to_radians();
        let roll = (self.max_roll() * roll.get()).to_radians();

        let total_tilt = pitch.hypot(roll);
        let base_acceleration = throttle.get() * self.max_thrust_acceleration();
        let vertical = base_acceleration * total_tilt.cos();

        if total_tilt.is_zero() {
            return BodyFrameAcceleration::from_vertical(vertical);
        }
        let forward = base_acceleration * total_tilt.sin() * (pitch / total_tilt);
        let right = base_acceleration * total_tilt.sin() * (roll / total_tilt);

        return BodyFrameAcceleration::new(forward, right, vertical);
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
    fn test_hover_thurst() {
        let drone = test_drone();
        assert_relative_eq!(drone.hover_throttle().get(), 0.5, epsilon = 1e-2);
    }

    #[test]
    fn test_heading_rate() {
        let drone = test_drone();
        assert_relative_eq!(drone.heading_rate(Yaw::clamp(0.5)).0, 22.5, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_vertical_only() {
        let drone = test_drone();
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(0.0),
            Roll::clamp(0.0),
        );

        assert_relative_eq!(acceleration.vertical().0, 15.69, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward().0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right().0, 0.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_forward() {
        let drone = test_drone();
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.5),
            Pitch::clamp(0.5),
            Roll::clamp(0.0),
        );

        assert_relative_eq!(acceleration.vertical().0, 9.06, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward().0, 3.75, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right().0, 0.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_hovering_max_forward() {
        let drone = test_drone();
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.707), // Thrust needed for hover while 100% pitch.
            Pitch::clamp(1.0),
            Roll::clamp(0.0),
        );

        assert_relative_eq!(acceleration.vertical().0, G_EARTH.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward().0, 9.805, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right().0, 0.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_backwards() {
        let drone = test_drone();
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(-0.5),
            Roll::clamp(0.0),
        );

        assert_relative_eq!(acceleration.vertical().0, 14.49, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward().0, -6.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right().0, 0.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_right() {
        let drone = test_drone();
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(0.0),
            Roll::clamp(0.5),
        );

        assert_relative_eq!(acceleration.vertical().0, 15.31, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward().0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right().0, 3.39, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_left() {
        let drone = test_drone();
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(0.0),
            Roll::clamp(-0.5),
        );

        assert_relative_eq!(acceleration.vertical().0, 15.31, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward().0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right().0, -3.39, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_max_side_acceleration() {
        let drone = test_drone();
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(1.0),
            Pitch::clamp(0.0),
            Roll::clamp(1.0),
        );

        assert_relative_eq!(acceleration.vertical().0, 17.77, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward().0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right().0, 8.28, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_diagonal_acceleration() {
        let drone = test_drone();
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(1.0),
            Pitch::clamp(1.0),
            Roll::clamp(1.0),
        );

        assert_relative_eq!(acceleration.vertical().0, 12.21, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward().0, 13.41, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right().0, 7.45, epsilon = 1e-2);
    }
}
