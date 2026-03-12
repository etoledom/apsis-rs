use crate::{
    controller::pid::VelocityPID,
    simulator::types::{acceleration_3d::WorldFrameAcceleration, velocity_ned::VelocityNED},
    units::{PerSecond, units::Seconds},
};

pub struct VelocityNedController {
    pub target: VelocityNED,
    north_pid: VelocityPID,
    east_pid: VelocityPID,
    down_pid: VelocityPID,
    k_ff: PerSecond, // Feed forward gain.
}

impl VelocityNedController {
    pub fn new(target: VelocityNED) -> Self {
        Self {
            target,
            north_pid: VelocityPID::new(5.5, 0.5, 0.2),
            east_pid: VelocityPID::new(5.5, 0.5, 0.2),
            down_pid: VelocityPID::new(4.0, 0.0, 0.3),
            k_ff: PerSecond(0.2),
        }
    }
    pub fn update(&mut self, current: VelocityNED, dt: Seconds) -> WorldFrameAcceleration {
        let error = self.target - current;

        WorldFrameAcceleration::new(
            self.north_pid.update(error.north(), dt) + self.target.north() * self.k_ff,
            self.east_pid.update(error.east(), dt) + self.target.east() * self.k_ff,
            self.down_pid.update(error.down(), dt) + self.target.down() * self.k_ff,
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::units::units::{SecondsLiteral, VelocityLiteral};

    use super::*;

    #[test]
    fn feedforward_maintains_velocity_at_zero_error() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(5.mps(), 0.mps(), 0.mps()));
        let current = VelocityNED::new(5.mps(), 0.mps(), 0.mps());

        let acc_target = controller.update(current, 0.1.seconds());

        assert!(acc_target.north().0 > 0.0); // feedforward maintains velocity
        assert_eq!(acc_target.east().0, 0.0); // no east target, no feedforward
        assert_eq!(acc_target.down().0, 0.0); // no down target, no feedforward
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
        let dt = 0.1.seconds();

        for _ in 0..100 {
            let acc_target = controller.update(current, dt);
            current += acc_target * dt;
        }
        println!("integral: {}", controller.north_pid.integral.0); // if available

        assert!(
            (current.north().0 - 10.0).abs() < 0.5,
            "Shoud converge to 10.0, but result was {}",
            current.north().0
        );
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

    #[test]
    fn climb_target_produces_negative_down_acceleration() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(0.mps(), 0.mps(), -2.mps()));
        let current = VelocityNED::zero();
        let acc_target = controller.update(current, 0.1.seconds());
        assert!(acc_target.down().0 < 0.0);
    }

    #[test]
    fn descend_target_produces_positive_down_acceleration() {
        let mut controller =
            VelocityNedController::new(VelocityNED::new(0.mps(), 0.mps(), 2.mps()));
        let current = VelocityNED::zero();
        let acc_target = controller.update(current, 0.1.seconds());
        assert!(acc_target.down().0 > 0.0);
    }
}
