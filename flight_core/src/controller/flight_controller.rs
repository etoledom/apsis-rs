use primitives::{
    control::Throttle,
    frames::*,
    math::{CrossMult, DotMult, Quaternion, Vec3},
    traits::{RawRepresentable, UnitsArithmetics},
    units::{consts::G_EARTH, *},
};

use crate::{
    Drone,
    controller::{
        acceleration_estimator::AccelerationEstimator,
        attitude_controller::AttitudeController,
        position_controller::PositionController,
        rate_controller::RateController,
        trajectory_generator::{
            trajectory_generator::TrajectoryGenerator, trajectory_limits::TrajectoryLimits,
        },
        velocity_controller::VelocityController,
    },
    drone::DragCoefficient,
    simulator::{inputs::Inputs, state::State},
};

const THRUST_MARGIN: f64 = 0.1;

#[derive(Default)]
pub struct AirframeLimits {
    pub max_pitch: Radians,
    pub max_roll: Radians,
    pub max_thrust: Acceleration,
    pub max_acceleration: AccelerationFrd,
    pub drag: DragCoefficient,
}

impl AirframeLimits {
    pub fn from_drone(drone: &impl Drone) -> AirframeLimits {
        let max_pitch = drone.max_pitch().to_radians();
        let max_roll = drone.max_roll().to_radians();
        let max_thrust = drone.max_thrust_acceleration();
        let max_forward_acc = G_EARTH * max_pitch.tan();
        let max_lateral_acc = G_EARTH * max_roll.tan();
        let max_vertical_acc = max_thrust * (1.0 - THRUST_MARGIN) - G_EARTH;

        AirframeLimits {
            max_pitch,
            max_roll,
            max_thrust,
            max_acceleration: AccelerationFrd::new(
                max_forward_acc,
                max_lateral_acc,
                max_vertical_acc,
            ),
            drag: drone.drag_coefficient(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AxisTarget {
    Velocity(Velocity),
    Position(Meters),
    Loiter,
}

#[derive(Debug, PartialEq)]
pub struct FlightTarget {
    pub north: AxisTarget,
    pub east: AxisTarget,
    pub down: AxisTarget,
}

pub struct TrajectoryConfig {
    pub max_horizontal_velocity: Velocity,
    pub max_vertical_velocity: Velocity,
    pub horizontal_jerk: Jerk,
    pub vertical_jerk: Jerk,
}

impl Default for TrajectoryConfig {
    fn default() -> Self {
        Self {
            max_horizontal_velocity: 10.mps(),
            max_vertical_velocity: 3.mps(),
            horizontal_jerk: 20.mps3(),
            vertical_jerk: 5.mps3(),
        }
    }
}

pub struct FlightController {
    target: FlightTarget,
    pub(crate) trajectory_generator: TrajectoryGenerator,
    position_controller: PositionController,
    velocity_controller: VelocityController,
    attitude_controller: AttitudeController,
    rate_controller: RateController,
    limits: AirframeLimits,
    yaw_rate_target: AngularVelocity,
    initialized: bool,
    acceleration_estimator: AccelerationEstimator,
    prev_saturation_error: AccelerationNed,
}

impl FlightController {
    pub fn new(
        limits: AirframeLimits,
        velocity_max: VelocityNed,
        h_trajectory_limits_auto: TrajectoryLimits,
        v_trajectory_limits_auto: TrajectoryLimits,
        h_trajectory_limits_manual: TrajectoryLimits,
        v_trajectory_limits_manual: TrajectoryLimits,
    ) -> Self {
        Self {
            target: FlightTarget {
                north: AxisTarget::Position(0.meters()),
                east: AxisTarget::Position(0.meters()),
                down: AxisTarget::Position(0.meters()),
            },
            trajectory_generator: TrajectoryGenerator::new(
                h_trajectory_limits_auto,
                v_trajectory_limits_auto,
                h_trajectory_limits_manual,
                v_trajectory_limits_manual,
                PerSecond(0.8),
            ),
            position_controller: PositionController::new(velocity_max),
            velocity_controller: VelocityController::new(),
            attitude_controller: AttitudeController::new(),
            rate_controller: RateController::new(),
            limits: limits,
            yaw_rate_target: Default::default(),
            initialized: false,
            acceleration_estimator: AccelerationEstimator::new(PerSecond(5.0)),
            prev_saturation_error: AccelerationNed::zero(),
        }
    }

    pub fn for_drone(drone: impl Drone) -> Self {
        let limits = AirframeLimits::from_drone(&drone);

        let max_forward_acc = limits.max_acceleration.forward();
        let max_lateral_acc = limits.max_acceleration.right();
        let max_vertical_acc = limits.max_acceleration.down();

        // Derive jerk limits from drone's rotational response bandwidth.
        let max_pitch_rate = drone.max_pitch_acceleration() / drone.pitch_damping_coefficient();

        let horizontal_jerk = G_EARTH * max_pitch_rate;
        let vertical_jerk = max_vertical_acc / drone.motor_time_constant();

        let max_forward_velocity = (max_forward_acc / drone.drag_coefficient().horizontal).sqrt();
        let max_lateral_velocity = (max_lateral_acc / drone.drag_coefficient().horizontal).sqrt();
        let max_vertical_velocity: Velocity =
            (max_vertical_acc / drone.drag_coefficient().vertical).sqrt();

        let trajectory_acc_factor = 0.7;
        let trajectory_jerk_factor = 0.7;

        let horizontal_limits_manual = TrajectoryLimits {
            max_velocity: 10.mps(),
            max_acceleration: max_lateral_acc,
            max_jerk: horizontal_jerk,
        };
        let vertical_limits_manual = TrajectoryLimits {
            max_velocity: max_forward_velocity,
            max_acceleration: max_vertical_acc,
            max_jerk: vertical_jerk,
        };

        let horizontal_limits_auto = TrajectoryLimits {
            max_velocity: 5.mps(),
            max_acceleration: max_lateral_acc * trajectory_acc_factor,
            max_jerk: horizontal_jerk * trajectory_jerk_factor,
        };
        let vertical_limits_auto = TrajectoryLimits {
            max_velocity: 3.mps(),
            max_acceleration: max_vertical_acc * trajectory_acc_factor,
            max_jerk: vertical_jerk * trajectory_jerk_factor,
        };

        Self::new(
            limits,
            VelocityNed::new(
                max_lateral_velocity, // Conservative limit to not mess up with Yaw
                max_lateral_velocity,
                max_vertical_velocity,
            ),
            horizontal_limits_auto,
            vertical_limits_auto,
            horizontal_limits_manual,
            vertical_limits_manual,
        )
    }

    // Cascade:
    // position_target -> velocity_target (position controller)
    // velocity_target -> acceleration_target (velocity controller + feedforward)
    // acceleration_target -> quaternion_target (thrust vector decomposition)
    // quaternion_target -> angular_rates_target (attitude controller)
    // angular_rates_target -> drone inputs: pitch, roll, yaw, throttle (rate controller)
    pub fn update(&mut self, telemetry: &State, dt: Seconds) -> Inputs {
        self.sync_if_needed(telemetry);
        // 1. Trajectory generator — open loop, produces feedforward signals
        let trajectory_setpoint = self.trajectory_generator.update(&self.target, dt);

        // 2. Position controller — feedback, compares plan vs reality
        let vel_correction = self.position_controller.update(
            telemetry.position_ned,
            trajectory_setpoint.position(),
            dt,
        );

        // 3. Combine: position correction + velocity feedforward
        let vel_ff = trajectory_setpoint.velocity();
        let vel_target = vel_correction + vel_ff;

        // 4. Acceleration feedforward from trajectory
        let acc_ff = trajectory_setpoint.acceleration();
        let acc_current = self
            .acceleration_estimator
            .update(telemetry.velocity_ned, dt);

        // 5. Velocity PID with both feedforwards
        //
        let acc_target = self.velocity_controller.update(
            telemetry.velocity_ned,
            vel_target,
            acc_current,
            acc_ff,
            self.prev_saturation_error,
            dt,
        );

        let acc_target_real = self.clamp_acceleration(acc_target, &telemetry.attitude);
        self.prev_saturation_error = acc_target - acc_target_real;

        let throttle = Self::throttle_from_acceleration(
            acc_target_real,
            &telemetry.attitude,
            self.limits.max_thrust,
        );

        let q_target = Self::q_target_from_acceleration(acc_target_real, telemetry.attitude.yaw())
            .clamped(self.limits.max_pitch, self.limits.max_roll);
        let angular_rates_target = self.compute_angular_rates(q_target, telemetry, dt);
        let (roll, pitch, yaw) =
            self.rate_controller
                .update(angular_rates_target, telemetry.angular_velocity_body, dt);

        Inputs {
            throttle,
            pitch,
            roll,
            yaw,
        }
    }

    fn sync_if_needed(&mut self, telemetry: &State) {
        if !self.initialized {
            self.sync_trajectory_to_state(telemetry);
            self.initialized = true;
        }
    }

    fn sync_trajectory_to_state(&mut self, telemetry: &State) {
        self.trajectory_generator.north.reset(
            telemetry.position_ned.north(),
            telemetry.velocity_ned.north(),
            telemetry.acceleration_ned.north(),
        );
        self.trajectory_generator.east.reset(
            telemetry.position_ned.east(),
            telemetry.velocity_ned.east(),
            telemetry.acceleration_ned.east(),
        );
        self.trajectory_generator.down.reset(
            telemetry.position_ned.down(),
            telemetry.velocity_ned.down(),
            telemetry.acceleration_ned.down(),
        );
        self.velocity_controller.reset_integral();
    }

    fn clamp_acceleration(
        &self,
        acc_unclamped: AccelerationNed,
        attitude: &Quaternion,
    ) -> AccelerationNed {
        // Horizontal: clamp in body frame (pitch/roll limits)
        let horizontal_only =
            AccelerationNed::new(acc_unclamped.north(), acc_unclamped.east(), 0.mps2());

        let clamped_horizontal = horizontal_only
            .to_frd(attitude)
            .clamping_forward(self.limits.max_acceleration.forward())
            .clamping_right(self.limits.max_acceleration.right())
            .to_ned(attitude);

        // Vertical: clamp in world frame (decoupled from attitude)
        AccelerationNed::new(
            clamped_horizontal.north(),
            clamped_horizontal.east(),
            acc_unclamped.down().clamping(
                -self.limits.max_acceleration.down(),
                self.limits.max_acceleration.down(),
            ),
        )
    }

    fn compute_angular_rates(
        &mut self,
        q_target: Quaternion,
        telemetry: &State,
        dt: Seconds,
    ) -> AngularVelocityFrd {
        let mut angular_rates_target =
            self.attitude_controller
                .update(q_target, telemetry.attitude, dt);

        angular_rates_target.set_z(self.yaw_rate_target); // Use the yaw rate directly from pilot.
        angular_rates_target
    }

    pub fn q_target_from_acceleration(
        acc_target: AccelerationNed,
        yaw_current: Radians,
    ) -> Quaternion {
        let gravity = AccelerationNed::from_down(G_EARTH);
        let thrust = acc_target + gravity;
        let thrust_dir = thrust.normalized();
        let z_body_desired = Vec3 {
            x: -thrust_dir.x,
            y: -thrust_dir.y,
            z: thrust_dir.z, // keep Z positive (down)
        };

        let x_body_desired = Vec3 {
            x: yaw_current.cos(),
            y: yaw_current.sin(),
            z: 0.0,
        }
        .normalized();

        let y_body_desired = z_body_desired.cross(x_body_desired).normalized();
        let x_body_desired =
            (x_body_desired - z_body_desired * x_body_desired.dot(z_body_desired)).normalized();

        let matrix = [
            [x_body_desired.x, y_body_desired.x, z_body_desired.x],
            [x_body_desired.y, y_body_desired.y, z_body_desired.y],
            [x_body_desired.z, y_body_desired.z, z_body_desired.z],
        ];

        Quaternion::from_rotation_matrix(matrix)
    }

    fn throttle_from_acceleration(
        acc_target: AccelerationNed,
        attitude: &Quaternion,
        max_thrust: Acceleration,
    ) -> Throttle {
        let vertical_acc_needed = acc_target.down() - G_EARTH;
        let tilt_compensation = attitude.pitch().cos() * attitude.roll().cos();

        Throttle::clamp(vertical_acc_needed.raw().abs() / (max_thrust.raw() * tilt_compensation))
    }

    pub fn set_target_velocity(&mut self, velocity: VelocityNed) {
        self.target.down = AxisTarget::Velocity(velocity.down());
        self.target.north = AxisTarget::Velocity(velocity.north());
        self.target.east = AxisTarget::Velocity(velocity.east());
    }

    pub fn set_yaw_rate_target(&mut self, rate: AngularVelocity) {
        self.yaw_rate_target = rate;
    }

    pub fn set_target_north(&mut self, north: AxisTarget) {
        if matches!(north, AxisTarget::Loiter) && !matches!(self.target.north, AxisTarget::Loiter) {
            self.initialized = false
        }

        self.target.north = north;
    }

    pub fn set_target_east(&mut self, east: AxisTarget) {
        if matches!(east, AxisTarget::Loiter) && !matches!(self.target.east, AxisTarget::Loiter) {
            self.initialized = false
        }
        self.target.east = east;
    }

    pub fn set_target_down(&mut self, down: AxisTarget) {
        if matches!(down, AxisTarget::Loiter) && !matches!(self.target.down, AxisTarget::Loiter) {
            self.initialized = false
        }
        self.target.down = down;
    }

    pub fn get_target(&self) -> &FlightTarget {
        &self.target
    }

    pub fn loiter_targets(&self) -> Vec3<Option<Meters>> {
        Vec3 {
            x: self.trajectory_generator.north.loiter_target(),
            y: self.trajectory_generator.east.loiter_target(),
            z: self.trajectory_generator.down.loiter_target(),
        }
    }
}

#[cfg(test)]
mod flight_controller_tests {
    use approx::assert_relative_eq;
    use primitives::prelude::*;

    use crate::{DefaultDrone, controller::tests_utils::TestDrone};

    use super::*;

    fn make_controller() -> FlightController {
        FlightController::for_drone(TestDrone {})
    }

    fn nominal_state() -> State {
        State {
            position_ned: PositionNed::from_altitude_ned(-10.meters()), // at target
            ..Default::default()
        }
    }

    #[test]
    fn nominal_state_produces_stable_inputs() {
        let mut ctrl = make_controller();
        let inputs = ctrl.update(&nominal_state(), 0.1.seconds());

        assert!((inputs.roll.raw()).abs() < 0.1, "roll should be near zero");
        assert!(
            (inputs.pitch.raw()).abs() < 0.1,
            "pitch should be near zero"
        );
        assert!((inputs.yaw.raw()).abs() < 0.1, "yaw should be near zero");
        // throttle near hover — exact value depends on your airframe limits
        assert!(inputs.throttle.raw() >= 0.0);
        assert!(inputs.throttle.raw() <= 1.0);
    }

    #[test]
    fn below_target_altitude_increases_throttle() {
        let mut ctrl = make_controller();

        let mut low_state = nominal_state();
        low_state.position_ned = PositionNed::from_altitude_ned(-5.meters()); // 5m below target

        ctrl.set_target_down(AxisTarget::Position(-10.meters()));

        let inputs = ctrl.update(&low_state, 0.1.seconds());

        assert!(
            inputs.throttle.raw() > 0.5,
            "should increase throttle when below target. {}",
            inputs.throttle.raw()
        );
    }

    #[test]
    fn should_increase_throttle_with_vertical_target_is_negative() {
        let mut ctrl = make_controller();

        let high_state = nominal_state();
        // high_state.position_ned = PositionNed::from_altitude_ned(-15.meters()); // 5m above target
        ctrl.set_target_velocity(VelocityNed::new(0.mps(), 0.mps(), -5.mps()));

        let inputs = ctrl.update(&high_state, 0.1.seconds());

        assert!(
            inputs.throttle.raw() > 0.5,
            "should decrease throttle when above target"
        );
    }

    #[test]
    fn should_decrease_throttle_with_vertical_target_is_positive() {
        let mut ctrl = make_controller();

        let high_state = nominal_state();
        // high_state.position_ned = PositionNed::from_altitude_ned(-15.meters()); // 5m above target
        ctrl.set_target_velocity(VelocityNed::new(0.mps(), 0.mps(), 5.mps()));

        let inputs = ctrl.update(&high_state, 0.1.seconds());

        assert!(
            inputs.throttle.raw() < 0.5,
            "should decrease throttle when above target"
        );
    }

    #[test]
    fn rolled_drone_gets_corrective_roll_input() {
        let mut ctrl = make_controller();

        let angle = 17.degrees().to_radians();
        let mut rolled_state = nominal_state();
        rolled_state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: (angle / 2.0).sin(),
            y: 0.0,
            z: 0.0,
        };

        let inputs = ctrl.update(&rolled_state, 0.1.seconds());
        assert!(
            inputs.roll.raw() < 0.0,
            "should correct against positive roll"
        );
    }

    #[test]
    fn larger_error_produces_larger_response() {
        let mut ctrl_small = make_controller();
        let mut ctrl_large = make_controller();
        let state = nominal_state(); // at -10m altitude

        // Small altitude error: target -11m (1m below current)
        ctrl_small.set_target_down(AxisTarget::Position((-11.0).meters()));
        // Large altitude error: target -15m (5m below current)
        ctrl_large.set_target_down(AxisTarget::Position((-15.0).meters()));

        let dt = 0.01.seconds();
        let mut inputs_small = ctrl_small.update(&state, dt);
        let mut inputs_large = ctrl_large.update(&state, dt);
        for _ in 0..100 {
            inputs_small = ctrl_small.update(&state, dt);
            inputs_large = ctrl_large.update(&state, dt);
        }

        assert!(
            inputs_large.throttle.raw() > inputs_small.throttle.raw(),
            "larger altitude error should produce larger throttle response, small={}, large={}",
            inputs_small.throttle.raw(),
            inputs_large.throttle.raw(),
        );
    }

    #[test]
    fn pitched_drone_gets_corrective_pitch_input() {
        let mut ctrl = make_controller();

        let angle = 17.degrees().to_radians();
        let mut pitched_state = nominal_state();
        pitched_state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };

        let inputs = ctrl.update(&pitched_state, 0.1.seconds());
        assert!(
            inputs.pitch.raw() < 0.0,
            "should correct against positive pitch"
        );
    }

    #[test]
    fn pitched_drone_gets_corrective_pitch_input_trace() {
        let mut ctrl = make_controller();

        let angle = Radians(0.3);
        let mut pitched_state = nominal_state();
        pitched_state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };

        let inputs = ctrl.update(&pitched_state, 0.1.seconds());

        assert!(inputs.pitch.raw() < 0.0);
    }

    #[test]
    fn forward_velocity_target_produces_negative_pitch_input() {
        let mut ctrl = FlightController::for_drone(DefaultDrone {});
        // target moving north
        ctrl.set_target_velocity(VelocityNed::new(1.mps(), 0.mps(), 0.mps()));

        let inputs = ctrl.update(&nominal_state(), 0.1.seconds());

        assert!(
            inputs.pitch.raw() < 0.0,
            "forward velocity target should produce forward pitch"
        );
    }

    #[test]
    fn pure_north_acceleration_produces_nose_down_pitch() {
        let acc = AccelerationNed::new(5.mps2(), 0.mps2(), 0.mps2());
        let q_target = FlightController::q_target_from_acceleration(acc, Default::default());

        assert!(
            q_target.y < 0.0,
            "north acc should produce negative pitch (aerospace convention), got {}",
            q_target.pitch().to_degrees().0
        );
    }

    #[test]
    fn pure_north_acceleration_produces_pitch() {
        let acc = AccelerationNed::new(5.mps2(), 0.mps2(), 0.mps2());

        let q_target = FlightController::q_target_from_acceleration(acc, Default::default());

        // North acceleration should produce positive pitch, no roll
        assert!(
            q_target.pitch().to_degrees().0 < 0.0,
            "north acc should produce negative pitch"
        );
        assert!(
            q_target.roll().to_degrees().0.abs() < 1e-6,
            "north acc should not affect roll"
        );
    }

    #[test]
    fn pure_east_acceleration_produces_roll() {
        // Pure east acceleration should produce a roll (rotation around X axis)
        let acc = AccelerationNed::new(0.mps2(), 5.mps2(), 0.mps2());

        let q_target = FlightController::q_target_from_acceleration(acc, Default::default());

        // East acceleration should produce positive roll, no pitch, no yaw
        assert!(
            q_target.roll().to_degrees().0 > 0.0,
            "east acc should produce positive roll"
        );
        assert!(
            q_target.pitch().to_degrees().0.abs() < 1e-6,
            "east acc should not affect pitch"
        );
    }

    #[test]
    fn level_drone_with_positive_pitch_target_produces_positive_pitch_error() {
        // q_current = level (identity)
        // q_target = positive pitch (nose up, aerospace convention)
        // q_error should be positive pitch → command nose down
        let q_current = Quaternion::identity();
        let angle = 0.3_f64;
        let q_target = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };

        let q_error = q_current.conjugate() * q_target;
        let q_error = if q_error.w < 0.0 { -q_error } else { q_error };

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
        let angle = 0.3_f64;
        let q_current = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
        };
        let q_target = Quaternion::identity();

        let q_error = q_current.conjugate() * q_target;
        let q_error = if q_error.w < 0.0 { -q_error } else { q_error };

        assert!(
            q_error.y < 0.0,
            "positive pitched drone with level target should give negative pitch error"
        );
    }

    #[test]
    fn acc_target_6_produces_31_degree_pitch() {
        let acc = AccelerationNed::new(6.0.mps2(), 0.mps2(), 0.mps2());
        let q_target = FlightController::q_target_from_acceleration(acc, Default::default());

        // atan(6 / 9.81) = 31.5°, nose down = negative in aerospace
        assert_relative_eq!(q_target.pitch().to_degrees().0, -31.5, epsilon = 0.5);
    }

    #[test]
    fn q_target_yaw_90_points_east() {
        // With zero acceleration and 90° yaw target,
        // the drone should be level and pointing east.
        let acc_target = AccelerationNed::zero();
        let q = FlightController::q_target_from_acceleration(acc_target, 90.degrees().to_radians());

        assert!(
            q.pitch().to_degrees().0.abs() < 0.1,
            "pitch: {}",
            q.pitch().to_degrees()
        );
        assert!(
            q.roll().to_degrees().0.abs() < 0.1,
            "roll: {}",
            q.roll().to_degrees()
        );
        assert!(
            (q.yaw().to_degrees().0 - 90.0).abs() < 0.1,
            "yaw: {}",
            q.yaw().to_degrees()
        );
    }

    #[test]
    fn q_target_yaw_with_north_acceleration() {
        // With north acceleration and 90° yaw target,
        // the drone should pitch forward (negative pitch in aerospace)
        // but still be pointing east (90° yaw)
        let acc_target = AccelerationNed::new(
            3.mps2(), // north acceleration
            0.mps2(),
            0.mps2(),
        );
        let q = FlightController::q_target_from_acceleration(acc_target, 90.degrees().to_radians());

        // Pitch should be non-zero due to north acceleration
        assert!(
            q.roll().to_degrees().0 < 0.0,
            "expected side tilt roll: {}",
            q.roll().to_degrees()
        );
        // Yaw should still be 90°
        assert!(
            (q.yaw().to_degrees().0 - 90.0).abs() < 1.0,
            "yaw: {}",
            q.yaw().to_degrees()
        );
    }

    #[test]
    fn throttle_from_acceleration_hover() {
        let acc_target = AccelerationNed::default();
        let throttle = FlightController::throttle_from_acceleration(
            acc_target,
            &Quaternion::identity(),
            G_EARTH * 2.0,
        );
        assert!((throttle.raw() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn throttle_from_acceleration_full_climb() {
        let acc_target = AccelerationNed::new(0.0.mps2(), 0.0.mps2(), -G_EARTH);
        let throttle = FlightController::throttle_from_acceleration(
            acc_target,
            &Quaternion::identity(),
            G_EARTH * 2.0,
        );
        assert!((throttle.raw() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn throttle_from_acceleration_freefall() {
        let acc_target = AccelerationNed::new(0.0.mps2(), 0.0.mps2(), G_EARTH);
        let throttle = FlightController::throttle_from_acceleration(
            acc_target,
            &Quaternion::identity(),
            G_EARTH * 2.0,
        );
        assert!((throttle.raw() - 0.0).abs() < 1e-6);
    }

    #[test]
    fn position_mode_uses_trajectory_and_position_controller() {
        let mut ctrl = make_controller();
        let mut state = nominal_state();
        state.position_ned = PositionNed::new(0.meters(), 0.meters(), (-5.0).meters());

        ctrl.set_target_north(AxisTarget::Position(0.meters()));
        ctrl.set_target_east(AxisTarget::Position(0.meters()));
        ctrl.set_target_down(AxisTarget::Position((-10.0).meters()));

        // Run a few ticks so the trajectory generator builds up state
        let dt = 0.1.seconds();
        let mut inputs = ctrl.update(&state, dt);
        for _ in 0..10 {
            inputs = ctrl.update(&state, dt);
        }

        // Drone is at -5m, target is -10m → should command climb (negative throttle bias)
        // Throttle should be above hover (0.5) to climb
        assert!(
            inputs.throttle.raw() > 0.5,
            "should command more than hover thrust to climb, got {}",
            inputs.throttle.raw()
        );
    }

    #[test]
    fn vertical_acceleration_is_clamped() {
        let mut ctrl = make_controller();
        let state = nominal_state();
        ctrl.set_target_down(AxisTarget::Position((-100.0).meters()));

        let dt = 0.1.seconds();
        for _ in 0..10 {
            ctrl.update(&state, dt);
        }
        let inputs = ctrl.update(&state, dt);

        let max_vertical_acc =
            TestDrone.max_thrust_acceleration() * (1.0 - THRUST_MARGIN) - G_EARTH;
        // Max throttle for clamped vertical acc:
        // throttle = (max_vertical_acc + G) / max_thrust
        let max_throttle = (max_vertical_acc + G_EARTH) / TestDrone.max_thrust_acceleration();

        assert!(
            inputs.throttle.raw() <= max_throttle + 0.01,
            "throttle should reflect clamped vertical acc, expected <= {}, got {}",
            max_throttle,
            inputs.throttle.raw()
        );
    }

    #[test]
    fn horizontal_acceleration_is_clamped_by_pitch_limit() {
        let mut ctrl = make_controller();
        let state = nominal_state();
        // Huge north velocity target → PID commands max acceleration
        ctrl.set_target_north(AxisTarget::Velocity(100.mps()));

        let dt = 0.1.seconds();
        for _ in 0..10 {
            ctrl.update(&state, dt);
        }
        let inputs = ctrl.update(&state, dt);

        // With large north velocity error, pitch should be at max
        // Pitch input of -1.0 means max nose down
        assert!(
            inputs.pitch.raw() <= -0.9,
            "pitch should be near max nose down for large north error, got {}",
            inputs.pitch.raw()
        );
    }

    #[test]
    fn yaw_rate_is_injected_directly() {
        let mut ctrl = make_controller();
        ctrl.set_yaw_rate_target(AngularVelocity::new(1.0));

        let state = nominal_state();
        let q_target = Quaternion::identity();
        let rates = ctrl.compute_angular_rates(q_target, &state, 0.1.seconds());
        assert_relative_eq!(rates.z().raw(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn pitched_drone_produces_nonzero_pitch_rate() {
        let mut ctrl = make_controller();
        let angle = 17.0.degrees().to_radians();
        let mut state = nominal_state();
        state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: -(angle / 2.0).sin(), // nose down
            z: 0.0,
        };
        let q_target = Quaternion::identity(); // level target
        let rates = ctrl.compute_angular_rates(q_target, &state, 0.1.seconds());
        assert!(rates.y().raw() > 0.0, "should command nose up to level");
    }

    #[test]
    fn rolled_drone_produces_nonzero_roll_rate() {
        let mut ctrl = make_controller();
        let angle = 15.0.degrees().to_radians();
        let mut state = nominal_state();
        state.attitude = Quaternion {
            w: (angle / 2.0).cos(),
            x: (angle / 2.0).sin(), // right wing down
            y: 0.0,
            z: 0.0,
        };
        let q_target = Quaternion::identity(); // level target
        let rates = ctrl.compute_angular_rates(q_target, &state, 0.1.seconds());
        assert!(rates.x().raw() < 0.0, "should command left roll to level");
    }

    #[test]
    fn acceleration_clamped_by_pitch_limit_at_zero_yaw() {
        let ctrl = make_controller();
        let large_north_acc = AccelerationNed::new(100.mps2(), 0.mps2(), 0.mps2());

        let clamped = ctrl.clamp_acceleration(large_north_acc, &Quaternion::identity());

        let max_forward_acc = ctrl.limits.max_acceleration.forward();
        assert!(
            clamped.north().raw() <= max_forward_acc.raw() + 0.01,
            "north should be clamped by max_pitch at 0° yaw, got {}",
            clamped.north().raw()
        );
    }

    #[test]
    fn acceleration_clamped_by_roll_limit_at_zero_yaw() {
        let ctrl = make_controller();
        let large_east_acc = AccelerationNed::new(0.mps2(), 100.mps2(), 0.mps2());

        let clamped = ctrl.clamp_acceleration(large_east_acc, &Quaternion::identity());

        let max_lateral_acc = ctrl.limits.max_acceleration.right();
        assert!(
            clamped.east().raw() <= max_lateral_acc.raw() + 0.01,
            "east should be clamped by max_roll at 0° yaw, got {}",
            clamped.east().raw()
        );
    }

    #[test]
    fn acceleration_limits_swap_at_90_degree_yaw() {
        let ctrl = make_controller();
        let large_east_acc = AccelerationNed::new(0.mps2(), 100.mps2(), 0.mps2());

        let yaw_90 = Quaternion::from_yaw(90.0.degrees().to_radians());
        let max_forward_acc = ctrl.limits.max_acceleration.forward();
        let max_lateral_acc = ctrl.limits.max_acceleration.right();

        let clamped = ctrl.clamp_acceleration(large_east_acc, &yaw_90);

        assert!(
            clamped.east().raw() > max_lateral_acc.raw(),
            "east should exceed roll limit at 90° yaw (pitch limit applies), got {}",
            clamped.east().raw()
        );
        assert!(
            clamped.east().raw() <= max_forward_acc.raw() + 0.01,
            "east should still be bounded by pitch limit, got {}",
            clamped.east().raw()
        );
    }

    #[test]
    fn north_gets_roll_limit_at_90_degree_yaw() {
        let ctrl = make_controller();
        let large_north_acc = AccelerationNed::new(100.mps2(), 0.mps2(), 0.mps2());

        let yaw_90 = Quaternion::from_yaw(90.0.degrees().to_radians());
        let max_lateral_acc = ctrl.limits.max_acceleration.right();

        let clamped = ctrl.clamp_acceleration(large_north_acc, &yaw_90);

        assert!(
            clamped.north().raw() <= max_lateral_acc.raw() + 0.01,
            "north should be clamped by max_roll at 90° yaw, got {}",
            clamped.north().raw()
        );
    }

    #[test]
    fn horizontal_clamping_does_not_affect_vertical() {
        let ctrl = make_controller();
        let acc = AccelerationNed::new(100.mps2(), 0.mps2(), (-2.0).mps2());

        let clamped_level = ctrl.clamp_acceleration(acc, &Quaternion::identity());

        let pitched = Quaternion::from_pitch((-20.0).degrees().to_radians());
        let clamped_pitched = ctrl.clamp_acceleration(acc, &pitched);

        assert!(
            (clamped_pitched.down().raw() - clamped_level.down().raw()).abs() < 0.01,
            "vertical acc should not change with pitch angle: level={}, pitched={}",
            clamped_level.down().raw(),
            clamped_pitched.down().raw(),
        );
    }
}
