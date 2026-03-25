use primitives::{
    control::{Pitch, Roll, Yaw},
    frames::AngularVelocityFrd,
    units::Seconds,
};

use crate::controller::pid::{PitchRatePID, RollRatePID, YawRatePID};

pub struct RateController {
    roll_pid: RollRatePID,
    pitch_pid: PitchRatePID,
    yaw_pid: YawRatePID,
}

impl RateController {
    pub fn new() -> Self {
        Self {
            roll_pid: RollRatePID::new(3, 0, 0),
            pitch_pid: PitchRatePID::new(3, 0, 0),
            yaw_pid: YawRatePID::new(2, 0, 0),
        }
    }

    pub fn update(
        &mut self,
        target: AngularVelocityFrd,
        current: AngularVelocityFrd,
        dt: Seconds,
    ) -> (Roll, Pitch, Yaw) {
        let error = target - current;
        (
            self.roll_pid.update(error.x(), dt),
            self.pitch_pid.update(error.y(), dt),
            self.yaw_pid.update(error.z(), dt),
        )
    }
}

#[cfg(test)]
mod rate_controller_tests {

    use primitives::{traits::RawRepresentable, units::SecondsLiteral};

    use super::*;

    fn make_controller() -> RateController {
        RateController::new()
    }

    fn zero_rates() -> AngularVelocityFrd {
        AngularVelocityFrd::default()
    }

    #[test]
    fn zero_error_produces_zero_inputs() {
        let mut ctrl = make_controller();
        let (roll, pitch, yaw) = ctrl.update(zero_rates(), zero_rates(), 0.1.seconds());
        assert_eq!((roll.raw()).abs(), 0.0);
        assert_eq!((pitch.raw()).abs(), 0.0);
        assert_eq!((yaw.raw()).abs(), 0.0);
    }

    #[test]
    fn roll_rate_error_only_affects_roll_input() {
        let mut ctrl = make_controller();
        let target = AngularVelocityFrd::new(1.0, 0.0, 0.0);
        let (roll, pitch, yaw) = ctrl.update(target, zero_rates(), 0.1.seconds());

        assert!(roll.raw() > 0.0, "roll input should be positive");
        assert!((pitch.raw()).abs() < 1e-6, "pitch input should be zero");
        assert!((yaw.raw()).abs() < 1e-6, "yaw input should be zero");
    }

    #[test]
    fn output_proportional_to_rate_error() {
        let mut ctrl = make_controller();
        let target = AngularVelocityFrd::new(2.0, 0.0, 0.0);
        let (roll, ..) = ctrl.update(target, zero_rates(), 0.1.seconds());
        // kp=0.8, error=2.0 → output=0.2
        // roll = 1.6.clamped() -> 1.0
        assert_eq!(roll.raw(), 1.0);
    }

    #[test]
    fn output_is_clamped_to_valid_range() {
        let mut ctrl = make_controller();
        let target = AngularVelocityFrd::new(9999.0, 0.0, 0.0);
        let (roll, ..) = ctrl.update(target, zero_rates(), 0.1.seconds());
        assert!(roll.raw() <= 1.0, "roll input must not exceed 1.0");
        assert!(roll.raw() >= -1.0);
    }

    #[test]
    fn positive_pitch_rate_error_produces_positive_pitch_input() {
        let mut ctrl = make_controller();
        let target = AngularVelocityFrd::new(0.0, 1.0, 0.0); // positive pitch rate
        let current = AngularVelocityFrd::zero();
        let (_, pitch, _) = ctrl.update(target, current, 0.1.seconds());

        assert!(
            pitch.raw() > 0.0,
            "positive pitch rate error should produce positive pitch input"
        );
    }

    #[test]
    fn positive_roll_rate_error_produces_positive_roll_input() {
        let mut ctrl = make_controller();
        let target = AngularVelocityFrd::new(1.0, 0.0, 0.0); // positive roll rate
        let current = AngularVelocityFrd::zero();
        let (roll, ..) = ctrl.update(target, current, 0.1.seconds());

        assert!(
            roll.raw() > 0.0,
            "positive roll rate error should produce positive roll input"
        );
    }
}
