use crate::{
    controller::pid::AngularPID,
    simulator::types::{angular_velocity_frd::AngularVelocityFrd, quaternion::Quaternion},
    units::{
        Seconds,
        angles::{AngularVelocity, Radians},
        traits::Initializable,
    },
};

pub struct AttitudeController {
    pitch_pid: AngularPID,
    roll_pid: AngularPID,
    yaw_pid: AngularPID,
}

impl AttitudeController {
    pub fn new() -> Self {
        Self {
            pitch_pid: AngularPID::new(5, 0, 2.0).with_limits(AngularVelocity::new(1.5)),
            roll_pid: AngularPID::new(5, 0, 2.0).with_limits(AngularVelocity::new(1.5)),
            yaw_pid: AngularPID::new(4, 0, 0.5).with_limits(AngularVelocity::new(1.5)),
        }
    }
    pub fn update(
        &mut self,
        target: Quaternion,
        current: Quaternion,
        dt: Seconds,
    ) -> AngularVelocityFrd {
        let q_error = Self::calculate_error(target, current);

        AngularVelocityFrd::new(
            self.roll_pid.update(Radians(q_error.x * 2.0), dt),
            self.pitch_pid.update(Radians(q_error.y * 2.0), dt),
            self.yaw_pid.update(Radians(q_error.z * 2.0), dt),
        )
    }

    fn calculate_error(target: Quaternion, current: Quaternion) -> Quaternion {
        let q_error = current.conjugate() * target;
        if q_error.w < 0.0 { -q_error } else { q_error }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::units::{SecondsLiteral, angles::DegreesLiteral, traits::RawRepresentable};

    use super::*;

    #[test]
    fn zero_error_produces_zero_velocity() {
        let mut controller = AttitudeController::new();

        let dt = 0.01.seconds();
        let velocity = controller.update(Quaternion::identity(), Quaternion::identity(), dt);

        assert_relative_eq!(velocity.x().raw(), 0.0, epsilon = 1e-6);
        assert_relative_eq!(velocity.y().raw(), 0.0, epsilon = 1e-6);
        assert_relative_eq!(velocity.z().raw(), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn pure_pitch_error_only_affects_pitch() {
        let mut controller = AttitudeController::new();

        // Small rotation around Y axis (pitch)
        let angle: f64 = 0.1;
        let q_target = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };

        let output = controller.update(q_target, Quaternion::identity(), 0.1.seconds());

        assert!(output.y().raw() > 0.0, "pitch should be positive");
        assert_relative_eq!((output.x().raw()).abs(), 0.0, epsilon = 1e-6);
        assert_relative_eq!((output.z().raw()).abs(), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn pure_roll_error_only_affects_roll() {
        let mut controller = AttitudeController::new();

        let angle: f64 = 0.1;
        let q_target = Quaternion {
            w: (angle / 2.0).cos(),
            x: (angle / 2.0).sin(),
            y: 0.0,
            z: 0.0,
        };

        let output = controller.update(q_target, Quaternion::identity(), 0.1.seconds());

        assert!(output.x().raw() > 0.0, "roll should be positive");
        assert!((output.y().raw()).abs() < 1e-6, "pitch should be zero");
        assert!((output.z().raw()).abs() < 1e-6, "yaw should be zero");
    }

    #[test]
    fn opposite_error_produces_opposite_output() {
        let mut controller_1 = AttitudeController::new();
        let mut controller_2 = AttitudeController::new();

        let angle: f64 = 0.1;
        let q_pos = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };
        let q_neg = q_pos.conjugate();

        let out_pos = controller_1.update(Quaternion::identity(), q_pos, 0.1.seconds());
        let out_neg = controller_2.update(Quaternion::identity(), q_neg, 0.1.seconds());

        assert!((out_pos.y() + out_neg.y()).raw().abs() < 1e-6);
    }

    #[test]
    fn level_drone_with_positive_pitch_target_produces_positive_pitch_error() {
        // q_current = level (identity)
        // q_target = positive pitch (nose down)
        // q_error should be positive pitch → command nose down
        let q_current = Quaternion::identity();
        let angle = 17.degrees().to_radians();
        let q_target = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };

        let q_error = AttitudeController::calculate_error(q_target, q_current);

        assert!(
            q_error.y > 0.0,
            "positive pitch target from level should give positive pitch error"
        );
    }

    #[test]
    fn positive_pitched_drone_with_level_target_produces_negative_pitch_error() {
        // q_current = positive pitch (nose down)
        // q_target = level
        // q_error should be negative → command nose up to recover
        let angle = 17.degrees().to_radians();
        let q_current = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };
        let q_target = Quaternion::identity();
        let q_error = AttitudeController::calculate_error(q_target, q_current);

        assert!(
            q_error.y < 0.0,
            "positive pitched drone with level target should give negative pitch error"
        );
    }
}
