use std::ops::{Add, Sub};

use crate::{
    simulator::types::{drag::Drag, pitch::Pitch, roll::Roll, vec2::Vec2, yaw::Yaw},
    types::throttle::Throttle,
    units::{
        acceleration::Acceleration,
        angles::{Degrees, DegreesPerSecond, Radians},
        consts::G_EARTH,
        units::{Kilograms, PerMeter},
    },
};

pub trait Drone {
    fn max_acceleration(&self) -> Acceleration;
    fn max_heading_rate(&self) -> DegreesPerSecond;
    fn max_roll(&self) -> Degrees; // side tilt
    fn max_pitch(&self) -> Degrees; // Forward / backward tilt
    fn mass(&self) -> Kilograms;
    fn battery_drain_pct(&self, throttle: Throttle) -> f64;
    fn drag_coefficient(&self) -> DragCoefficient;

    fn hover_throttle(&self) -> Throttle {
        Throttle::clamp(G_EARTH / self.max_acceleration())
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

        let base_acceleration = throttle.get() * self.max_acceleration();

        let vertical = base_acceleration * total_tilt.cos();

        if total_tilt.0 == 0.0 {
            return BodyFrameAcceleration {
                vertical,
                forward: Acceleration(0.0),
                right: Acceleration(0.0),
            };
        }
        let forward = base_acceleration * total_tilt.sin() * (pitch / total_tilt);
        let right = base_acceleration * total_tilt.sin() * (roll / total_tilt);

        return BodyFrameAcceleration {
            vertical,
            forward,
            right,
        };
    }
}

#[derive(Clone, Copy)]
pub struct BodyFrameAcceleration {
    pub vertical: Acceleration,
    pub forward: Acceleration,
    pub right: Acceleration,
}

impl BodyFrameAcceleration {
    pub fn new(ground: Vec2<Acceleration>, vertical: Acceleration) -> Self {
        Self {
            vertical,
            forward: ground.y,
            right: ground.x,
        }
    }

    pub fn ground_acceleration(&self) -> Vec2<Acceleration> {
        Vec2::new(self.right, self.forward)
    }

    pub fn rotate(&self, angle: Radians) -> BodyFrameAcceleration {
        let ground_acceleration = self.ground_acceleration();
        let rotated = ground_acceleration.rotate(angle);
        BodyFrameAcceleration::new(rotated, self.vertical)
    }
}

impl Add<BodyFrameAcceleration> for BodyFrameAcceleration {
    type Output = BodyFrameAcceleration;

    fn add(self, rhs: BodyFrameAcceleration) -> Self::Output {
        BodyFrameAcceleration {
            vertical: self.vertical + rhs.vertical,
            forward: self.forward + rhs.forward,
            right: self.right + rhs.right,
        }
    }
}

impl Sub<BodyFrameAcceleration> for BodyFrameAcceleration {
    type Output = BodyFrameAcceleration;

    fn sub(self, rhs: BodyFrameAcceleration) -> Self::Output {
        BodyFrameAcceleration {
            vertical: self.vertical - rhs.vertical,
            forward: self.forward - rhs.forward,
            right: self.right - rhs.right,
        }
    }
}

pub struct DragCoefficient {
    pub vertical: Drag,
    pub horizontal: Drag,
    pub forward: Drag,
}

#[cfg(test)]
mod tests {
    use crate::units::{acceleration::AccelerationLiteral, angles::DegreesLiteral};

    use super::*;
    use approx::{self, assert_relative_eq};

    struct TestDrone;
    impl Drone for TestDrone {
        fn max_acceleration(&self) -> Acceleration {
            20.mps2()
        }

        fn max_heading_rate(&self) -> DegreesPerSecond {
            DegreesPerSecond(20.0)
        }

        fn max_roll(&self) -> Degrees {
            45.degrees()
        }

        fn max_pitch(&self) -> Degrees {
            20.degrees()
        }

        fn mass(&self) -> Kilograms {
            Kilograms(0.5)
        }

        fn battery_drain_pct(&self, throttle: Throttle) -> f64 {
            0.1 + throttle.get() * 0.5
        }

        fn drag_coefficient(&self) -> DragCoefficient {
            DragCoefficient {
                vertical: Drag::new(0.6),
                horizontal: Drag::new(0.4),
                forward: Drag::new(0.2),
            }
        }
    }

    #[test]
    fn test_hover_thurst() {
        let drone = TestDrone {};
        assert_relative_eq!(drone.hover_throttle().get(), 0.49, epsilon = 1e-2);
    }

    #[test]
    fn test_heading_rate() {
        let drone = TestDrone {};
        assert_relative_eq!(drone.heading_rate(Yaw::clamp(0.5)).0, 10.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_vertical_only() {
        let drone = TestDrone {};
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(0.0),
            Roll::clamp(0.0),
        );

        assert_relative_eq!(acceleration.vertical.0, 6.19, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward.0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right.0, 0.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_forward() {
        let drone = TestDrone {};
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(0.5),
            Roll::clamp(0.0),
        );

        assert_relative_eq!(acceleration.vertical.0, 5.95, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward.0, 2.77, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right.0, 0.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_hovering_max_forward() {
        let drone = TestDrone {};
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.522),
            Pitch::clamp(1.0),
            Roll::clamp(0.0),
        );

        assert_relative_eq!(acceleration.vertical.0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward.0, 3.57, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right.0, 0.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_backwards() {
        let drone = TestDrone {};
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(-0.5),
            Roll::clamp(0.0),
        );

        assert_relative_eq!(acceleration.vertical.0, 5.95, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward.0, -2.77, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right.0, 0.0, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_right() {
        let drone = TestDrone {};
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(0.0),
            Roll::clamp(0.5),
        );

        assert_relative_eq!(acceleration.vertical.0, 4.97, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward.0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right.0, 6.12, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_acceleration_left() {
        let drone = TestDrone {};
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(0.8),
            Pitch::clamp(0.0),
            Roll::clamp(-0.5),
        );

        assert_relative_eq!(acceleration.vertical.0, 4.97, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward.0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right.0, -6.12, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_max_side_acceleration() {
        let drone = TestDrone {};
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(1.0),
            Pitch::clamp(0.0),
            Roll::clamp(1.0),
        );

        assert_relative_eq!(acceleration.vertical.0, 4.33, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward.0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right.0, 14.14, epsilon = 1e-2);
    }

    #[test]
    fn test_body_frame_diagonal_acceleration() {
        let drone = TestDrone {};
        let acceleration = drone.body_frame_acceleration(
            Throttle::clamp(1.0),
            Pitch::clamp(1.0),
            Roll::clamp(1.0),
        );

        assert_relative_eq!(acceleration.vertical.0, 3.25, epsilon = 1e-2);
        assert_relative_eq!(acceleration.forward.0, 6.15, epsilon = 1e-2);
        assert_relative_eq!(acceleration.right.0, 13.84, epsilon = 1e-2);
    }
}
