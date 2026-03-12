use crate::{
    AngularVelocity3D, Drone, PositionNed, Throttle,
    controller::{
        attitude_controller::AttitudeController, position_controller::PositionController,
        rate_controller::RateController, velocity_ned_controller::VelocityNedController,
    },
    simulator::{
        inputs::Inputs,
        state::State,
        types::{
            acceleration_3d::WorldFrameAcceleration,
            quaternion::Quaternion,
            vec3::{CrossMult, DotMult, Vec3},
            velocity_ned::VelocityNED,
        },
    },
    units::{
        MettersLiteral, Velocity,
        acceleration::Acceleration,
        angles::{AngularVelocity, Radians},
        consts::G_EARTH,
        units::{Meters, Seconds},
    },
};

const THRUST_MARGIN: f64 = 0.1;

#[derive(Default)]
pub struct AirframeLimits {
    pub max_pitch: Radians,
    pub max_roll: Radians,
    pub max_thrust: Acceleration,
}

impl AirframeLimits {
    pub fn from_drone(drone: impl Drone) -> AirframeLimits {
        AirframeLimits {
            max_pitch: drone.max_pitch().to_radians(),
            max_roll: drone.max_roll().to_radians(),
            max_thrust: drone.max_thrust_acceleration(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum AxisTarget {
    Velocity(Velocity),
    Position(Meters),
}

#[derive(Debug)]
pub struct FlightTarget {
    pub north: AxisTarget,
    pub east: AxisTarget,
    pub down: AxisTarget,
}

pub struct FlightController {
    target: FlightTarget,
    position_controller: PositionController,
    velocity_ned_controller: VelocityNedController,
    attitude_controller: AttitudeController,
    rate_controller: RateController,
    limits: AirframeLimits,
    yaw_rate_target: AngularVelocity,
}

impl FlightController {
    pub fn new(limits: AirframeLimits) -> Self {
        Self {
            target: FlightTarget {
                north: AxisTarget::Position(0.meters()),
                east: AxisTarget::Position(0.meters()),
                down: AxisTarget::Position(0.meters()),
            },
            position_controller: PositionController::new(PositionNed::zero()),
            velocity_ned_controller: VelocityNedController::new(VelocityNED::zero()),
            attitude_controller: AttitudeController::new(),
            rate_controller: RateController::new(),
            limits: limits,
            yaw_rate_target: Default::default(),
        }
    }

    pub fn for_drone(drone: impl Drone) -> Self {
        Self::new(AirframeLimits::from_drone(drone))
    }

    // Cascade:
    // position_target -> velocity_target (position controller)
    // velocity_target -> acceleration_target (velocity controller + feedforward)
    // acceleration_target -> quaternion_target (thrust vector decomposition)
    // quaternion_target -> angular_rates_target (attitude controller)
    // angular_rates_target -> drone inputs: pitch, roll, yaw, throttle (rate controller)
    pub fn update(&mut self, telemetry: &State, dt: Seconds) -> Inputs {
        let vel_target = self.resolve_velocity_target(telemetry, dt);
        self.velocity_ned_controller.target = vel_target;
        let acc_target = self.compute_acceleration_target(telemetry.velocity_ned, dt);
        let throttle = Self::throttle_from_acceleration(
            acc_target,
            &telemetry.attitude,
            self.limits.max_thrust,
        );
        let q_target = Self::q_target_from_acceleration(acc_target, telemetry.attitude.yaw())
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

    fn compute_acceleration_target(
        &mut self,
        current_velocity: VelocityNED,
        dt: Seconds,
    ) -> WorldFrameAcceleration {
        // Use trhurst margin
        let max_vertical_acc = self.limits.max_thrust * (1.0 - THRUST_MARGIN) - G_EARTH;
        let max_north_acc = G_EARTH * self.limits.max_pitch.tan();
        let max_east_acc = G_EARTH * self.limits.max_roll.tan();

        let acc_target = self.velocity_ned_controller.update(current_velocity, dt);

        acc_target
            .clamping_down(max_vertical_acc)
            .clamping_north(max_north_acc)
            .clamping_east(max_east_acc)
    }

    fn compute_angular_rates(
        &mut self,
        q_target: Quaternion,
        telemetry: &State,
        dt: Seconds,
    ) -> AngularVelocity3D {
        let mut angular_rates_target =
            self.attitude_controller
                .update(q_target, telemetry.attitude, dt);

        angular_rates_target.set_z(self.yaw_rate_target); // Use the yaw rate directly from pilot.
        angular_rates_target
    }

    pub fn q_target_from_acceleration(
        acc_target: WorldFrameAcceleration,
        yaw_current: Radians,
    ) -> Quaternion {
        let gravity = WorldFrameAcceleration::from_down(G_EARTH);
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
        acc_target: WorldFrameAcceleration,
        attitude: &Quaternion,
        max_thrust: Acceleration,
    ) -> Throttle {
        let vertical_acc_needed = acc_target.down().0 - G_EARTH.raw();
        let tilt_compensation = attitude.pitch().cos() * attitude.roll().cos();

        Throttle::clamp(vertical_acc_needed.abs() / (max_thrust.raw() * tilt_compensation))
    }

    fn throttle_from_attitude(attitude: &Quaternion, max_thrust: Acceleration) -> Throttle {
        let thrust_needed = G_EARTH.raw() / attitude.pitch().cos() / attitude.roll().cos();
        Throttle::clamp(thrust_needed / max_thrust.raw())
    }

    pub fn set_target_velocity(&mut self, velocity: VelocityNED) {
        self.target.down = AxisTarget::Velocity(velocity.down());
        self.target.north = AxisTarget::Velocity(velocity.north());
        self.target.east = AxisTarget::Velocity(velocity.east());
    }

    fn update_position_targets(&mut self, telemetry: &State) {
        if let AxisTarget::Velocity(_) = self.target.north {
            self.position_controller
                .target
                .set_north(telemetry.position_ned.north());
        }
        if let AxisTarget::Velocity(_) = self.target.east {
            self.position_controller
                .target
                .set_east(telemetry.position_ned.east());
        }
        if let AxisTarget::Velocity(_) = self.target.down {
            self.position_controller
                .target
                .set_down(telemetry.position_ned.down());
        }
    }

    fn resolve_velocity_target(&mut self, telemetry: &State, dt: Seconds) -> VelocityNED {
        self.update_position_targets(telemetry);
        let velocity_target = self.compute_velocity_target(telemetry, dt);
        velocity_target
    }

    fn compute_velocity_target(&mut self, telemetry: &State, dt: Seconds) -> VelocityNED {
        let velocity_target_north = match self.target.north {
            AxisTarget::Velocity(velocity) => velocity,
            AxisTarget::Position(meters) => {
                self.position_controller.target.set_north(meters);
                self.position_controller
                    .update_north(telemetry.position_ned.north(), dt)
            }
        };
        let velocity_target_east = match self.target.east {
            AxisTarget::Velocity(velocity) => velocity,
            AxisTarget::Position(meters) => {
                self.position_controller.target.set_east(meters);
                self.position_controller
                    .update_east(telemetry.position_ned.east(), dt)
            }
        };
        let velocity_target_down = match self.target.down {
            AxisTarget::Velocity(velocity) => velocity,
            AxisTarget::Position(meters) => {
                self.position_controller.target.set_down(meters);
                self.position_controller
                    .update_down(telemetry.position_ned.down(), dt)
            }
        };
        VelocityNED::new(
            velocity_target_north,
            velocity_target_east,
            velocity_target_down,
        )
    }

    pub fn get_target_velocity(&self) -> VelocityNED {
        self.velocity_ned_controller.target
    }

    pub fn set_yaw_rate_target(&mut self, rate: AngularVelocity) {
        self.yaw_rate_target = rate;
    }

    pub fn set_target_north(&mut self, north: AxisTarget) {
        self.target.north = north;
    }

    pub fn set_target_east(&mut self, east: AxisTarget) {
        self.target.east = east;
    }

    pub fn set_target_down(&mut self, down: AxisTarget) {
        self.target.down = down;
    }
}

#[cfg(test)]
mod flight_controller_tests {
    use approx::assert_relative_eq;

    use crate::{
        DefaultDrone,
        simulator::types::{angular_velocity_3d::AngularVelocity3D, position_ned::PositionNed},
        units::{
            acceleration::AccelerationLiteral,
            angles::DegreesLiteral,
            units::{MettersLiteral, SecondsLiteral, VelocityLiteral},
        },
    };

    use super::*;

    fn make_controller() -> FlightController {
        FlightController::for_drone(DefaultDrone {})
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

        assert!((inputs.roll.get()).abs() < 0.1, "roll should be near zero");
        assert!(
            (inputs.pitch.get()).abs() < 0.1,
            "pitch should be near zero"
        );
        assert!((inputs.yaw.get()).abs() < 0.1, "yaw should be near zero");
        // throttle near hover — exact value depends on your airframe limits
        assert!(inputs.throttle.get() >= 0.0);
        assert!(inputs.throttle.get() <= 1.0);
    }

    #[test]
    fn below_target_altitude_increases_throttle() {
        let mut ctrl = make_controller();

        let mut low_state = nominal_state();
        low_state.position_ned = PositionNed::from_altitude_ned(-5.meters()); // 5m below target

        ctrl.set_target_down(AxisTarget::Position(-10.meters()));

        let inputs = ctrl.update(&low_state, Seconds(0.1));

        assert!(
            inputs.throttle.get() > 0.5,
            "should increase throttle when below target. {}",
            inputs.throttle.get()
        );
    }

    #[test]
    fn should_increase_throttle_with_vertical_target_is_negative() {
        let mut ctrl = make_controller();

        let high_state = nominal_state();
        // high_state.position_ned = PositionNed::from_altitude_ned(-15.meters()); // 5m above target
        ctrl.set_target_velocity(VelocityNED::new(0.mps(), 0.mps(), -5.mps()));

        let inputs = ctrl.update(&high_state, Seconds(0.1));

        assert!(
            inputs.throttle.get() > 0.5,
            "should decrease throttle when above target"
        );
    }

    #[test]
    fn should_decrease_throttle_with_vertical_target_is_positive() {
        let mut ctrl = make_controller();

        let high_state = nominal_state();
        // high_state.position_ned = PositionNed::from_altitude_ned(-15.meters()); // 5m above target
        ctrl.set_target_velocity(VelocityNED::new(0.mps(), 0.mps(), 5.mps()));

        let inputs = ctrl.update(&high_state, Seconds(0.1));

        assert!(
            inputs.throttle.get() < 0.5,
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

        let inputs = ctrl.update(&rolled_state, Seconds(0.1));
        assert!(
            inputs.roll.get() < 0.0,
            "should correct against positive roll"
        );
    }

    #[test]
    fn inputs_always_within_valid_range() {
        let mut ctrl = make_controller();

        // Extreme state — far from target, high rates
        let extreme_state = State {
            altitude: Meters(0.0),
            velocity_ned: VelocityNED::new(10.mps(), 10.mps(), 10.mps()),
            attitude: Quaternion {
                w: 0.5,
                x: 0.5,
                y: 0.5,
                z: 0.5,
            }, // large tilt
            angular_velocity_body: AngularVelocity3D::new(5.0, 5.0, 5.0),
            ..nominal_state()
        };

        let inputs = ctrl.update(&extreme_state, Seconds(0.1));
        assert!(inputs.throttle.get() >= 0.0 && inputs.throttle.get() <= 1.0);
        assert!(inputs.roll.get() >= -1.0 && inputs.roll.get() <= 1.0);
        assert!(inputs.pitch.get() >= -1.0 && inputs.pitch.get() <= 1.0);
        assert!(inputs.yaw.get() >= -1.0 && inputs.yaw.get() <= 1.0);
    }

    fn max_thrust_limit(limit: Acceleration) -> AirframeLimits {
        AirframeLimits {
            max_pitch: Radians(0.0),
            max_roll: Radians(0.0),
            max_thrust: limit,
        }
    }

    #[test]
    fn larger_error_produces_larger_response() {
        let mut ctrl_small = make_controller();
        let mut ctrl_large = make_controller();

        let state = nominal_state();

        ctrl_small.set_target_velocity(VelocityNED::new(0.mps(), 0.mps(), -1.mps()));
        ctrl_large.set_target_velocity(VelocityNED::new(0.mps(), 0.mps(), -5.mps()));

        let inputs_small = ctrl_small.update(&state, Seconds(0.1));
        let inputs_large = ctrl_large.update(&state, Seconds(0.1));

        assert!(
            inputs_large.throttle.get() > inputs_small.throttle.get(),
            "larger altitude error should produce larger throttle response, {}. {}",
            inputs_large.throttle.get(),
            inputs_small.throttle.get()
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
            inputs.pitch.get() < 0.0,
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

        let inputs = ctrl.update(&pitched_state, Seconds(0.1));

        assert!(inputs.pitch.get() < 0.0);
    }

    #[test]
    fn forward_velocity_target_produces_negative_pitch_input() {
        let mut ctrl = FlightController::for_drone(DefaultDrone {});
        // target moving north
        ctrl.set_target_velocity(VelocityNED::new(1.mps(), 0.mps(), 0.mps()));

        let inputs = ctrl.update(&nominal_state(), Seconds(0.1));

        assert!(
            inputs.pitch.get() < 0.0,
            "forward velocity target should produce forward pitch"
        );
    }

    #[test]
    fn pure_north_acceleration_produces_nose_down_pitch() {
        let acc = WorldFrameAcceleration::new(5.mps2(), 0.mps2(), 0.mps2());
        let q_target = FlightController::q_target_from_acceleration(acc, Default::default());

        assert!(
            q_target.y < 0.0,
            "north acc should produce negative pitch (aerospace convention), got {}",
            q_target.pitch().to_degrees().0
        );
    }

    #[test]
    fn pure_north_acceleration_produces_pitch() {
        let acc = WorldFrameAcceleration::new(5.mps2(), 0.mps2(), 0.mps2());

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
        let acc = WorldFrameAcceleration::new(0.mps2(), 5.mps2(), 0.mps2());

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
        let acc = WorldFrameAcceleration::new(6.0.mps2(), 0.mps2(), 0.mps2());
        let q_target = FlightController::q_target_from_acceleration(acc, Default::default());

        // atan(6 / 9.81) = 31.5°, nose down = negative in aerospace
        assert_relative_eq!(q_target.pitch().to_degrees().0, -31.5, epsilon = 0.5);
    }

    #[test]
    fn q_target_yaw_90_points_east() {
        // With zero acceleration and 90° yaw target,
        // the drone should be level and pointing east.
        let acc_target = WorldFrameAcceleration::zero();
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
        let acc_target = WorldFrameAcceleration::new(
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
        let acc_target = WorldFrameAcceleration::default();
        let throttle = FlightController::throttle_from_acceleration(
            acc_target,
            &Quaternion::identity(),
            G_EARTH * 2.0,
        );
        assert!((throttle.get() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn throttle_from_acceleration_full_climb() {
        let acc_target = WorldFrameAcceleration::new(0.0.mps2(), 0.0.mps2(), -G_EARTH);
        let throttle = FlightController::throttle_from_acceleration(
            acc_target,
            &Quaternion::identity(),
            G_EARTH * 2.0,
        );
        assert!((throttle.get() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn throttle_from_acceleration_freefall() {
        let acc_target = WorldFrameAcceleration::new(0.0.mps2(), 0.0.mps2(), G_EARTH);
        let throttle = FlightController::throttle_from_acceleration(
            acc_target,
            &Quaternion::identity(),
            G_EARTH * 2.0,
        );
        assert!((throttle.get() - 0.0).abs() < 1e-6);
    }

    #[test]
    fn velocity_mode_passes_through_and_updates_position_target() {
        let mut ctrl = make_controller();
        let state = nominal_state();
        ctrl.set_target_north(AxisTarget::Velocity(3.0.mps()));
        ctrl.set_target_east(AxisTarget::Velocity(2.0.mps()));
        ctrl.set_target_down(AxisTarget::Velocity(-1.0.mps()));
        let velocity_target = ctrl.resolve_velocity_target(&state, Seconds(0.1));
        // Velocity passes through
        assert_eq!(velocity_target.north().0, 3.0);
        assert_eq!(velocity_target.east().0, 2.0);
        assert_eq!(velocity_target.down().0, -1.0);
        // Position target updated to current position
        assert_relative_eq!(
            ctrl.position_controller.target.north().0,
            state.position_ned.north().0,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            ctrl.position_controller.target.east().0,
            state.position_ned.east().0,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            ctrl.position_controller.target.down().0,
            state.position_ned.down().0,
            epsilon = 1e-6
        );
    }

    #[test]
    fn position_mode_uses_position_controller_output() {
        let mut ctrl = make_controller();
        let mut state = nominal_state();
        state.position_ned = PositionNed::from_altitude_ned(-5.meters()); // 5m below target
        ctrl.set_target_north(AxisTarget::Position(0.0.meters()));
        ctrl.set_target_east(AxisTarget::Position(0.0.meters()));
        ctrl.set_target_down(AxisTarget::Position(-10.meters())); // target is 10m altitude
        let velocity_target = ctrl.resolve_velocity_target(&state, Seconds(0.1));
        // Position controller should command upward velocity (negative down) to reach target
        assert!(
            velocity_target.down().0 < 0.0,
            "should command climb to reach target altitude"
        );
    }

    #[test]
    fn vertical_acceleration_is_clamped() {
        let mut ctrl = make_controller();
        // Large downward velocity error should produce clamped vertical acc
        let velocity_target = VelocityNED::new(0.mps(), 0.mps(), 100.mps());
        let acc_target = ctrl.compute_acceleration_target(velocity_target, Seconds(0.1));
        let max_vertical_acc =
            DefaultDrone {}.max_thrust_acceleration() * (1.0 - THRUST_MARGIN) - G_EARTH;
        assert!(
            acc_target.down().0 <= max_vertical_acc.raw(),
            "vertical acc should be clamped, was {}",
            acc_target.down().0
        );
    }

    #[test]
    fn horizontal_acceleration_is_not_clamped_by_compute() {
        let mut ctrl = make_controller();
        let max_north_acc = G_EARTH * DefaultDrone {}.max_pitch().to_radians().tan();

        // Large north velocity error
        let velocity_target = VelocityNED::new(100.mps(), 0.mps(), 0.mps());
        ctrl.set_target_north(AxisTarget::Velocity(velocity_target.north()));
        ctrl.velocity_ned_controller.target = velocity_target;
        let acc_target = ctrl.compute_acceleration_target(VelocityNED::zero(), Seconds(0.1));

        assert_relative_eq!(
            acc_target.north().raw(),
            max_north_acc.raw(),
            epsilon = 1e-6
        );
    }

    #[test]
    fn yaw_rate_is_injected_directly() {
        let mut ctrl = make_controller();
        ctrl.set_yaw_rate_target(AngularVelocity(1.0));

        let state = nominal_state();
        let q_target = Quaternion::identity();
        let rates = ctrl.compute_angular_rates(q_target, &state, Seconds(0.1));
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
        let rates = ctrl.compute_angular_rates(q_target, &state, Seconds(0.1));
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
        let rates = ctrl.compute_angular_rates(q_target, &state, Seconds(0.1));
        assert!(rates.x().raw() < 0.0, "should command left roll to level");
    }
}
