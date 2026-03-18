use primitives::{
    frames::{PositionNed, VelocityNed},
    units::Seconds,
};

use crate::controller::pid::PositionPID;

pub struct PositionController {
    north_pid: PositionPID,
    east_pid: PositionPID,
    altitude_pid: PositionPID,
}

impl PositionController {
    pub fn new(velocity_max: VelocityNed) -> Self {
        Self {
            north_pid: PositionPID::new(0.95, 0, 0).with_limits(velocity_max.north()),
            east_pid: PositionPID::new(0.95, 0, 0).with_limits(velocity_max.east()),
            altitude_pid: PositionPID::new(1, 0, 0).with_limits(velocity_max.down()),
        }
    }

    pub fn update(
        &mut self,
        current: PositionNed,
        target: PositionNed,
        dt: Seconds,
    ) -> VelocityNed {
        VelocityNed::new(
            self.north_pid.update(target.north() - current.north(), dt),
            self.east_pid.update(target.east() - current.east(), dt),
            self.altitude_pid.update(target.down() - current.down(), dt),
        )
    }
}

#[cfg(test)]
mod tests {
    use primitives::prelude::*;

    use super::*;

    const EPSILON: f64 = 1e-10;

    #[test]
    fn zero_error_produces_zero_correction() {
        let mut controller = PositionController::new(VelocityNed::new(10.mps(), 10.mps(), 5.mps()));
        let current = PositionNed::new(10.meters(), 5.meters(), -10.0.meters());
        let target = current;

        let correction = controller.update(current, target, 0.01.seconds());
        assert!(correction.north().raw().abs() < EPSILON);
        assert!(correction.east().raw().abs() < EPSILON);
        assert!(correction.down().raw().abs() < EPSILON);
    }

    #[test]
    fn positive_error_produces_positive_correction() {
        let mut controller = PositionController::new(VelocityNed::new(10.mps(), 10.mps(), 5.mps()));
        let current = PositionNed::new(8.meters(), 4.meters(), -6.0.meters());
        let target = PositionNed::new(10.meters(), 6.meters(), -8.0.meters());

        let correction = controller.update(current, target, 0.01.seconds());
        assert!((correction.north().raw() - 1.9).abs() < EPSILON);
        assert!((correction.east().raw() - 1.9).abs() < EPSILON);
        assert!((correction.down().raw() - (-2.0)).abs() < EPSILON);
    }

    #[test]
    fn negative_error_produces_negative_correction() {
        let mut controller = PositionController::new(VelocityNed::new(10.mps(), 10.mps(), 5.mps()));
        let current = PositionNed::new(12.meters(), 8.meters(), -12.0.meters());
        let target = PositionNed::new(10.meters(), 6.meters(), -10.0.meters());

        let correction = controller.update(current, target, 0.01.seconds());
        assert!((correction.north().raw() - (-1.9)).abs() < EPSILON);
        assert!((correction.east().raw() - (-1.9)).abs() < EPSILON);
        assert!((correction.down().raw() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn output_is_clamped_to_max_velocity() {
        let mut controller = PositionController::new(VelocityNed::new(3.mps(), 3.mps(), 2.mps()));
        let current = PositionNed::new(0.meters(), 0.meters(), 0.meters());
        let target = PositionNed::new(100.meters(), 100.meters(), -100.0.meters());

        let correction = controller.update(current, target, 0.01.seconds());
        assert!((correction.north().raw() - 3.0).abs() < EPSILON);
        assert!((correction.east().raw() - 3.0).abs() < EPSILON);
        assert!((correction.down().raw() - (-2.0)).abs() < EPSILON);
    }

    #[test]
    fn altitude_has_higher_gain_than_horizontal() {
        let mut controller =
            PositionController::new(VelocityNed::new(10.mps(), 10.mps(), 10.mps()));
        let current = PositionNed::new(0.meters(), 0.meters(), 0.meters());
        let target = PositionNed::new(1.meters(), 1.meters(), -1.meters());

        let correction = controller.update(current, target, 0.01.seconds());
        assert!(correction.down().raw().abs() > correction.north().raw().abs());
    }
}
