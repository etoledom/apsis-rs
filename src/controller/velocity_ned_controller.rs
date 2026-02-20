use crate::{
    controller::pid::{VelocityDownPID, VelocityEastPID, VelocityNorthPID},
    simulator::types::{acceleration_3d::WorldFrameAcceleration, velocity_ned::VelocityNED},
    units::{acceleration::Acceleration, units::Seconds},
};

pub struct VelocityNedController {
    pub target: VelocityNED,
    north_pid: VelocityNorthPID,
    east_pid: VelocityEastPID,
    down_pid: VelocityDownPID,
}

impl VelocityNedController {
    pub fn new(target: VelocityNED) -> Self {
        Self {
            target,
            north_pid: VelocityNorthPID::new(4, 0.4, 0.05),
            east_pid: VelocityEastPID::new(4, 0.4, 0.05),
            down_pid: VelocityDownPID::new(0.8, 0, 0.1),
        }
    }
    pub fn update(&mut self, current: VelocityNED, dt: Seconds) -> WorldFrameAcceleration {
        let error = self.target - current;
        // println!("v_error north: {}", error.north().0);
        WorldFrameAcceleration::new(
            Acceleration(self.north_pid.update(error.north(), dt).0.clamp(-6.0, 6.0)),
            self.east_pid.update(error.east(), dt),
            self.down_pid.update(error.down(), dt),
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::units::units::{SecondsLiteral, VelocityLiteral};

    use super::*;

    #[test]
    fn zero_acceleration_with_zero_error() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(5.mps(), 0.mps(), 0.mps()));
        let current = VelocityNED::new(5.mps(), 0.mps(), 0.mps());

        let acc_target = controller.update(current, 0.1.seconds());

        assert_eq!(acc_target.norm().0, 0.0);
    }

    #[test]
    fn positive_acceleration_with_positive_error() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(5.mps(), 5.mps(), 5.mps()));
        let current = VelocityNED::zero();

        let acc_target = controller.update(current, 0.1.seconds());

        assert!(acc_target.north().0 > 1.0);
        assert!(acc_target.east().0 > 1.0);
        assert!(acc_target.down().0 > 1.0);
    }

    #[test]
    fn negative_acceleration_with_negative_error() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(5.mps(), 5.mps(), 5.mps()));
        let current = VelocityNED::new(15.mps(), 15.mps(), 15.mps());

        let acc_target = controller.update(current, 0.1.seconds());

        assert!(acc_target.north().0 < -1.0);
        assert!(acc_target.east().0 < -1.0);
        assert!(acc_target.down().0 < -1.0);
    }

    #[test]
    fn axis_with_different_signs() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(5.mps(), 5.mps(), 5.mps()));
        let current = VelocityNED::new(15.mps(), 0.mps(), 15.mps());

        let acc_target = controller.update(current, 0.1.seconds());

        assert!(acc_target.north().0 < -1.0);
        assert!(acc_target.east().0 > 1.0);
        assert!(acc_target.down().0 < -1.0);
    }

    #[test]
    fn small_dt_stability() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(10.mps(), 0.mps(), 0.mps()));
        let current = VelocityNED::zero();

        let acc_target = controller.update(current, 0.00001.seconds());

        assert!(acc_target.north().0.is_finite());
    }

    #[test]
    fn convergence() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(10.mps(), 0.mps(), 0.mps()));
        let mut current = VelocityNED::zero();
        let dt = 0.01.seconds();

        for _ in 0..200 {
            let acc_target = controller.update(current, dt);
            current += acc_target * dt;
        }

        assert!((current.north().0 - 10.0).abs() < 0.5);
    }

    #[test]
    fn positive_north_error_produces_positive_north_acceleration() {
        // If we want to go north faster, we need positive north acceleration
        let mut controller =
            VelocityNedController::new(VelocityNED::new(1.mps(), 0.mps(), 0.mps()));

        let current = VelocityNED::new(0.mps(), 0.mps(), 0.mps()); // 1 m/s north error
        let acc = controller.update(current, 0.1.seconds());

        assert!(
            acc.north().0 > 0.0,
            "positive north velocity error should produce positive north acceleration"
        );
    }

    #[test]
    fn positive_east_error_produces_positive_east_acceleration() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(0.mps(), 1.mps(), 0.mps()));
        let current = VelocityNED::zero();
        let acc = controller.update(current, 0.1.seconds());

        assert!(
            acc.east().0 > 0.0,
            "positive east velocity error should produce positive east acceleration"
        );
    }
}
