use crate::{
    controller::pid::VelocityPID,
    simulator::types::{acceleration_3d::AccelerationNed, velocity_ned::VelocityNed},
    units::units::Seconds,
};

pub struct VelocityNedController {
    pub target: VelocityNed,
    north_pid: VelocityPID,
    east_pid: VelocityPID,
    pub down_pid: VelocityPID,
    k_ff: f64, // Feed forward gain.
}

impl VelocityNedController {
    pub fn new(target: VelocityNed) -> Self {
        Self {
            target,
            north_pid: VelocityPID::new(6, 0, 0.2),
            east_pid: VelocityPID::new(6, 0, 0.2),
            down_pid: VelocityPID::new(4.0, 0, 0.3),
            k_ff: 1.0,
        }
    }
    pub fn update(
        &mut self,
        current: VelocityNed,
        trajectory_acceleration: AccelerationNed,
        drag_ff: AccelerationNed,
        dt: Seconds,
    ) -> AccelerationNed {
        let error = self.target - current;

        AccelerationNed::new(
            self.north_pid.update(error.north(), dt)
                + trajectory_acceleration.north() * self.k_ff
                + drag_ff.north(),
            self.east_pid.update(error.east(), dt)
                + trajectory_acceleration.east() * self.k_ff
                + drag_ff.east(),
            self.down_pid.update(error.down(), dt)
                + trajectory_acceleration.down() * self.k_ff
                + drag_ff.down(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{
        acceleration::AccelerationLiteral,
        units::{SecondsLiteral, VelocityLiteral},
    };

    #[test]
    fn feedforward_contributes_to_output() {
        // With zero velocity error, only feedforward contributes
        let mut controller =
            VelocityNedController::new(VelocityNed::new(5.mps(), 0.mps(), 0.mps()));
        let current = VelocityNed::new(5.mps(), 0.mps(), 0.mps());
        let trajectory_acc = AccelerationNed::new(2.mps2(), 0.mps2(), 0.mps2());

        let acc_target = controller.update(
            current,
            trajectory_acc,
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        assert!(
            acc_target.north().0 > 0.0,
            "feedforward should contribute positive north acceleration"
        );
        assert_eq!(acc_target.east().0, 0.0);
        assert_eq!(acc_target.down().0, 0.0);
    }

    #[test]
    fn positive_acceleration_with_positive_error() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(5.mps(), 5.mps(), 5.mps()));
        let current = VelocityNed::zero();

        let acc_target = controller.update(
            current,
            AccelerationNed::zero(),
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        assert!(acc_target.north().0 > 1.0);
        assert!(acc_target.east().0 > 1.0);
        assert!(acc_target.down().0 > 1.0);
    }

    #[test]
    fn negative_acceleration_with_negative_error() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(5.mps(), 5.mps(), 5.mps()));
        let current = VelocityNed::new(15.mps(), 15.mps(), 15.mps());

        let acc_target = controller.update(
            current,
            AccelerationNed::zero(),
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        assert!(acc_target.north().0 < -1.0);
        assert!(acc_target.east().0 < -1.0);
        assert!(acc_target.down().0 < -1.0);
    }

    #[test]
    fn axis_with_different_signs() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(5.mps(), 5.mps(), 5.mps()));
        let current = VelocityNed::new(15.mps(), 0.mps(), 15.mps());

        let acc_target = controller.update(
            current,
            AccelerationNed::zero(),
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        assert!(acc_target.north().0 < -1.0);
        assert!(acc_target.east().0 > 1.0);
        assert!(acc_target.down().0 < -1.0);
    }

    #[test]
    fn small_dt_stability() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(10.mps(), 0.mps(), 0.mps()));
        let current = VelocityNed::zero();

        let acc_target = controller.update(
            current,
            AccelerationNed::zero(),
            AccelerationNed::zero(),
            0.00001.seconds(),
        );

        assert!(acc_target.north().0.is_finite());
    }

    #[test]
    fn convergence() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(10.mps(), 0.mps(), 0.mps()));
        let mut current = VelocityNed::zero();
        let dt = 0.1.seconds();

        for _ in 0..100 {
            let acc_target = controller.update(
                current,
                AccelerationNed::zero(),
                AccelerationNed::zero(),
                dt,
            );
            current += acc_target * dt;
        }

        assert!(
            (current.north().0 - 10.0).abs() < 0.5,
            "Should converge to 10.0, but result was {}",
            current.north().0
        );
    }

    #[test]
    fn positive_north_error_produces_positive_north_acceleration() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(1.mps(), 0.mps(), 0.mps()));
        let current = VelocityNed::zero();

        let acc = controller.update(
            current,
            AccelerationNed::zero(),
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        assert!(
            acc.north().0 > 0.0,
            "positive north velocity error should produce positive north acceleration"
        );
    }

    #[test]
    fn positive_east_error_produces_positive_east_acceleration() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(0.mps(), 1.mps(), 0.mps()));
        let current = VelocityNed::zero();

        let acc = controller.update(
            current,
            AccelerationNed::zero(),
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        assert!(
            acc.east().0 > 0.0,
            "positive east velocity error should produce positive east acceleration"
        );
    }

    #[test]
    fn climb_target_produces_negative_down_acceleration() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(0.mps(), 0.mps(), -2.mps()));
        let current = VelocityNed::zero();

        let acc_target = controller.update(
            current,
            AccelerationNed::zero(),
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        assert!(acc_target.down().0 < 0.0);
    }

    #[test]
    fn descend_target_produces_positive_down_acceleration() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(0.mps(), 0.mps(), 2.mps()));
        let current = VelocityNed::zero();

        let acc_target = controller.update(
            current,
            AccelerationNed::zero(),
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        assert!(acc_target.down().0 > 0.0);
    }
    #[test]
    fn drag_feedforward_contributes_to_output() {
        // Zero velocity error, zero trajectory — only drag feedforward contributes
        let mut controller = VelocityNedController::new(VelocityNed::from_north(5.mps()));
        let current = VelocityNed::from_north(5.mps());
        let drag_ff = AccelerationNed::new(1.5.mps2(), 0.mps2(), 0.mps2());

        let acc_target =
            controller.update(current, AccelerationNed::zero(), drag_ff, 0.1.seconds());

        assert!(
            acc_target.north().0 > 1.0,
            "drag feedforward should contribute positive north acceleration, got {}",
            acc_target.north().0
        );
        assert_eq!(acc_target.east().0, 0.0);
        assert_eq!(acc_target.down().0, 0.0);
    }

    #[test]
    fn drag_feedforward_on_multiple_axes() {
        let mut controller =
            VelocityNedController::new(VelocityNed::new(3.mps(), 2.mps(), 0.mps()));
        let current = VelocityNed::new(3.mps(), 2.mps(), 0.mps());
        let drag_ff = AccelerationNed::new(1.0.mps2(), 0.5.mps2(), 0.mps2());

        let acc_target =
            controller.update(current, AccelerationNed::zero(), drag_ff, 0.1.seconds());

        assert!(
            acc_target.north().0 > 0.5,
            "north drag ff should pass through"
        );
        assert!(
            acc_target.east().0 > 0.2,
            "east drag ff should pass through"
        );
    }

    #[test]
    fn drag_and_trajectory_feedforward_combine() {
        let mut controller = VelocityNedController::new(VelocityNed::from_north(5.mps()));
        let current = VelocityNed::from_north(5.mps());
        let trajectory_acc = AccelerationNed::new(1.0.mps2(), 0.mps2(), 0.mps2());
        let drag_ff = AccelerationNed::new(0.8.mps2(), 0.mps2(), 0.mps2());

        let acc_drag_only =
            controller.update(current, AccelerationNed::zero(), drag_ff, 0.1.seconds());

        // Reset integral state
        let mut controller = VelocityNedController::new(VelocityNed::from_north(5.mps()));

        let acc_traj_only = controller.update(
            current,
            trajectory_acc,
            AccelerationNed::zero(),
            0.1.seconds(),
        );

        let mut controller = VelocityNedController::new(VelocityNed::from_north(5.mps()));

        let acc_both = controller.update(current, trajectory_acc, drag_ff, 0.1.seconds());

        let tolerance = 1e-6;
        assert!(
            (acc_both.north().0 - (acc_drag_only.north().0 + acc_traj_only.north().0)).abs()
                < tolerance,
            "feedforwards should combine additively: both={}, drag={} + traj={}",
            acc_both.north().0,
            acc_drag_only.north().0,
            acc_traj_only.north().0,
        );
    }
}
