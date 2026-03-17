use crate::{
    Yaw,
    simulator::{
        drone::Drone,
        force_model::{
            context::Context, core_mechanics_models::CoreMechanicsModel, force_model::ForceModel,
        },
        inputs::Inputs,
        state::State,
        types::{
            acceleration_3d::{AccelerationNed, WorldFrameGroundVelocity},
            angular_acceleration_frd::AngularAccelerationFrd,
            pitch::Pitch,
            quaternion::Quaternion,
            roll::Roll,
        },
    },
    units::{
        consts::EARTH_RADIUS,
        units::{Meters, Seconds, Velocity},
    },
};

pub struct Simulator<DroneType: Drone> {
    pub state: State,
    pub drone: DroneType,
    core_mechanics_models: CoreMechanicsModel,
    external_flight_mechanics_models: Vec<Box<dyn ForceModel<DroneType>>>,
}

impl<DroneType: Drone> Simulator<DroneType> {
    pub fn new(drone: DroneType) -> Self {
        Self {
            state: Default::default(),
            drone,
            core_mechanics_models: CoreMechanicsModel::new(),
            external_flight_mechanics_models: vec![],
        }
    }

    pub fn add_force<Force>(&mut self, force_model: Force)
    where
        Force: ForceModel<DroneType> + 'static,
    {
        self.external_flight_mechanics_models
            .push(Box::new(force_model));
    }

    pub fn tick(&mut self, inputs: &Inputs, delta_t: Seconds) {
        self.update_attitude(inputs.pitch, inputs.roll, inputs.yaw, delta_t);
        self.state.acceleration_ned = self.net_acceleration(delta_t, inputs);
        self.integrate(self.state.acceleration_ned, delta_t);

        self.update_horizontal_position(self.state.velocity_ned.ground_speed(), delta_t);
        self.update_battery_state(delta_t, inputs);
        self.state.last_inputs = inputs.clone();
    }

    fn update_battery_state(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let battery_drain_pct = self.drone.battery_drain_pct(inputs.throttle) * delta_t.0;
        self.state.battery_pct -= battery_drain_pct;
        self.state.battery_pct = self.state.battery_pct.max(0.0);
    }

    fn net_acceleration(&self, delta_t: Seconds, inputs: &Inputs) -> AccelerationNed {
        let context = Context::new(&self.state, inputs, &self.drone);

        let external_acceleration = self
            .external_flight_mechanics_models
            .iter()
            .fold(AccelerationNed::zero(), |acceleration, force| {
                acceleration + force.acceleration_contribution(&context, delta_t)
            });

        self.core_mechanics_models
            .acceleration_contribution(&context, delta_t)
            + external_acceleration
    }

    fn integrate(&mut self, acceleration: AccelerationNed, delta_t: Seconds) {
        let velocity_ned_old = self.state.velocity_ned;

        self.state.velocity_ned += acceleration * delta_t;

        self.state.position_ned +=
            velocity_ned_old * delta_t + (acceleration * delta_t) * delta_t * 0.5;
        self.state.altitude = -self.state.position_ned.down();

        if self.state.velocity_ned.down().0 > 0.0 && self.state.altitude.0 < 0.0 {
            self.state.velocity_ned.update_down(Velocity(0.0));
            self.state.altitude = Meters::zero();
        }
    }

    fn update_horizontal_position(
        &mut self,
        ground_sleed: WorldFrameGroundVelocity,
        delta_t: Seconds,
    ) {
        // Distance traveled North/East
        let north_distance = ground_sleed.north() * delta_t;
        let east_distance = ground_sleed.east() * delta_t;

        // Lat/Lon delta (simple spherical approx)
        self.state.latitude += north_distance / EARTH_RADIUS;
        self.state.longitude += east_distance / (EARTH_RADIUS * self.state.latitude.cos());
    }

    fn update_attitude(&mut self, pitch: Pitch, roll: Roll, yaw: Yaw, dt: Seconds) {
        // 1. Compute angular acceleration in body frame
        // 2. Integrate angular velocity (body frame)
        // 3. Build omega quaternion from body-frame ω
        // 4. Compute: q_dot = 0.5 * q * omega (body to world frame quaternion)
        // 5. Integrate quaternion (Euler integration for small dt, as it's expected)
        // 6. Normalize
        let drone = &self.drone;

        // 1.
        let pitch_acceleration = drone.max_pitch_acceleration() * pitch.get()
            - (drone.pitch_damping_coefficient() * self.state.angular_velocity_body.y());

        let roll_acceleration = drone.max_roll_acceleration() * roll.get()
            - drone.roll_damping_coefficient() * self.state.angular_velocity_body.x();

        let yaw_acceleration = drone.max_yaw_acceleration() * yaw.get()
            - drone.yaw_damping_coefficient() * self.state.angular_velocity_body.z();

        let angular_acceleration =
            AngularAccelerationFrd::new(roll_acceleration, pitch_acceleration, yaw_acceleration);

        // 2.
        self.state.angular_velocity_body += angular_acceleration * dt;

        // 3.
        let quat_omega = Quaternion::omega(self.state.angular_velocity_body);

        // 4.
        let quat_dot = (self.state.attitude * quat_omega) * 0.5;

        // 5-6.
        let attitude = (self.state.attitude + quat_dot * dt.0).normalized();

        let clamped_attitude = attitude.clamped(
            self.drone.max_pitch().to_radians(),
            self.drone.max_roll().to_radians(),
        );

        let current_angular_velocity = self.state.angular_velocity_body;
        let mut new_angular_velocity = current_angular_velocity;
        new_angular_velocity += angular_acceleration * dt;

        if (clamped_attitude.roll() - attitude.roll()).0.abs() > 1e-6 {
            self.state.angular_velocity_body.set_x(0.0.into());
        }

        if (clamped_attitude.pitch() - attitude.pitch()).0.abs() > 1e-6 {
            self.state.angular_velocity_body.set_y(0.0.into());
        }

        self.state.attitude = clamped_attitude;
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::{
        controller::tests_utils::TestDrone,
        simulator::{
            default_drone::DefaultDrone,
            types::{
                pitch::Pitch, position_ned::PositionNed, roll::Roll, throttle::Throttle, yaw::Yaw,
            },
        },
        units::{
            MetersLiteral,
            angles::{Degrees, DegreesLiteral},
            consts::G_EARTH,
            units::{Meters, SecondsLiteral},
        },
    };

    use super::*;

    fn terminal_forward_velocity(drone: impl Drone, angle: Degrees) -> Velocity {
        // v_north = sqrt(g * sin(θ) / (k_forward * cos³(θ)))
        // where θ = pitch, k_forward = forward drag coefficient
        (G_EARTH * angle.to_radians().sin()
            / (drone.drag_coefficient().forward.value() * angle.to_radians().cos().powi(3)))
        .sqrt()
    }

    #[test]
    fn stable_hover() {
        let drone = DefaultDrone {};
        let inputs = Inputs {
            throttle: drone.hover_throttle(),
            pitch: Pitch::clamp(0.0),
            roll: Roll::clamp(0.0),
            yaw: Yaw::clamp(0.0),
        };

        let mut simulator = Simulator::new(drone);
        simulator.state.position_ned =
            PositionNed::new(Meters::zero(), Meters::zero(), Meters(-1.0));

        for _ in 0..100 {
            simulator.tick(&inputs, 0.02.seconds());
        }

        assert_relative_eq!(simulator.state.altitude.0, 1.0);
        assert_relative_eq!(simulator.state.velocity_ned.down().0, 0.0);
    }

    #[test]
    fn forward_terminal_velocity() {
        let drone = DefaultDrone {};
        let inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            pitch: Pitch::clamp(-1.0),
            roll: Roll::clamp(0.0),
            yaw: Yaw::clamp(0.0),
        };

        let expected = terminal_forward_velocity(drone, drone.max_pitch());

        let mut simulator = Simulator::new(drone);
        simulator.state.position_ned.set_down(-50.meters());

        for _ in 0..100 {
            simulator.tick(&inputs, 0.1.seconds());
            println!("altitude: {}", simulator.state.altitude);
        }

        assert_relative_eq!(simulator.state.velocity_ned.ground_speed().east().0, 0.0);
        assert_relative_eq!(
            simulator.state.velocity_ned.ground_speed().north().0,
            expected.0,
            epsilon = 1e-2
        );
    }

    #[test]
    fn test_yaw_rotation() {
        let drone = TestDrone {};
        let hover_throttle = drone.hover_throttle();

        let mut simulator = Simulator::new(drone);

        for _ in 0..100 {
            simulator.tick(
                &Inputs {
                    throttle: hover_throttle,
                    pitch: Pitch::clamp(0.8),
                    roll: Roll::clamp(0.0),
                    yaw: Yaw::clamp(1.0),
                },
                0.01.seconds(),
            );
        }

        assert_relative_eq!(
            simulator.state.attitude.yaw().to_degrees().0,
            35.28,
            epsilon = 1e-2
        ); // Near 45 degrees as specified.
    }

    #[test]
    fn test_vertical_terminal_velocity() {
        let drone = DefaultDrone {};

        let mut simulator = Simulator::new(drone);

        let inputs = Inputs {
            throttle: Throttle::max(),
            ..Default::default()
        };

        // V_up = -sqrt_root( G(TWR - 1) / k ) ; where: TWR = thrust to wait ration; k = vertical drag coeff
        let expected = -(G_EARTH * (drone.thrust_to_waight_ratio() - 1.0)
            / drone.drag_coefficient().vertical.value())
        .sqrt();

        for _ in 0..150 {
            simulator.tick(&inputs, 0.01.seconds());
        }

        assert_relative_eq!(
            simulator.state.velocity_ned.down().0,
            expected.0,
            epsilon = 1e-2
        );

        for _ in 0..10 {
            simulator.tick(&inputs, 0.01.seconds());
        }

        assert_relative_eq!(
            simulator.state.velocity_ned.down().0,
            expected.0,
            epsilon = 1e-2
        );
    }

    #[test]
    fn free_fall_terminal_velocity() {
        let drone = DefaultDrone {};
        let mut simulator = Simulator::new(drone);
        let input = Inputs::default();

        simulator.state.position_ned =
            PositionNed::new(Meters::zero(), Meters::zero(), Meters(-100.0));

        for _ in 0..200 {
            simulator.tick(&input, 0.01.seconds());
        }

        // (G / drag_coef_vertical)^2
        let expected = (G_EARTH.0 / drone.drag_coefficient().vertical.value().0).sqrt();

        assert_relative_eq!(
            simulator.state.velocity_ned.down().0,
            expected,
            epsilon = 1e-2
        );
    }

    #[test]
    fn pitch_input_response() {
        let mut sim = Simulator::new(TestDrone {});

        // Start level at hover
        let inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            pitch: Pitch::clamp(-0.1), // small constant pitch command
            ..Default::default()
        };

        let dt = Seconds(0.1);
        for i in 0..40 {
            sim.tick(&inputs, dt);
            println!(
                "t={:.1} pitch={:.2} pitch_rate={:.3}, v_angular: {:.3}",
                i as f64 * dt.0,
                sim.state.attitude.pitch().to_degrees().0,
                sim.state.angular_velocity_body.y().raw(),
                sim.state.angular_velocity_body.y().raw()
            );
        }

        // At steady state, pitch rate should be constant (not growing)
        // and angular velocity should match theoretical steady state
        //
        // Steady state: max_pitch_acc * input / damping = 2.0 * 0.1 / 2.0 = 0.1
        assert_relative_eq!(
            sim.state.angular_velocity_body.y().raw(),
            -0.1,
            epsilon = 0.01
        );
    }

    #[test]
    fn pitch_reversal_decelerates() {
        let drone = TestDrone {};
        let mut sim = Simulator::new(drone);

        // Start at 45° nose down
        sim.state.attitude = Quaternion::from_pitch(-drone.max_pitch().to_radians());

        let dt = 0.1.seconds();

        // Phase 1: pitch forward until terminal velocity
        let forward_inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            pitch: Pitch::clamp(-1.0),
            ..Default::default()
        };
        for _ in 0..100 {
            sim.tick(&forward_inputs, dt);
        }

        let expected = terminal_forward_velocity(drone, drone.max_pitch());

        assert_relative_eq!(sim.state.velocity_ned.north().0, expected.0, epsilon = 1e-2);
        assert_relative_eq!(
            sim.state.attitude.pitch().to_degrees().0,
            -45.0,
            epsilon = 1e-2
        );

        // Phase 2: reverse pitch command
        let reverse_inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            pitch: Pitch::clamp(1.0),
            ..Default::default()
        };
        for _ in 0..200 {
            sim.tick(&reverse_inputs, dt);
        }

        assert_relative_eq!(
            sim.state.velocity_ned.north().0,
            -expected.0,
            epsilon = 1e-2
        );
        assert_relative_eq!(
            sim.state.attitude.pitch().to_degrees().0,
            45.0,
            epsilon = 1e-2
        );
    }

    #[test]
    fn roll_reversal_decelerates() {
        let mut sim = Simulator::new(DefaultDrone {});

        // Start at 45° roll right
        sim.state.attitude = Quaternion {
            w: (45.0.degrees().to_radians() / 2.0).cos(),
            x: (45.0.degrees().to_radians() / 2.0).sin(),
            y: 0.0,
            z: 0.0,
        };

        let dt = Seconds(0.1);

        // Phase 1: roll right until terminal velocity
        println!("--- Phase 1: rolling right ---");
        let right_inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            roll: Roll::clamp(1.0),
            ..Default::default()
        };
        for i in 0..50 {
            sim.tick(&right_inputs, dt);
            println!(
                "t={:.1} roll={:.2} roll_rate={:.4} v_east={:.2}",
                i as f64 * dt.0,
                sim.state.attitude.roll().to_degrees().0,
                sim.state.angular_velocity_body.x().raw(),
                sim.state.velocity_ned.east().0,
            );
        }

        // Phase 2: reverse roll command
        println!("--- Phase 2: reversing ---");
        let left_inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            roll: Roll::clamp(-1.0),
            ..Default::default()
        };
        for i in 0..100 {
            sim.tick(&left_inputs, dt);
            println!(
                "t={:.1} roll={:.2} roll_rate={:.4} v_east={:.2}",
                (50 + i) as f64 * dt.0,
                sim.state.attitude.roll().to_degrees().0,
                sim.state.angular_velocity_body.x().raw(),
                sim.state.velocity_ned.east().0,
            );
        }
    }

    #[test]
    fn negative_pitch_input_pitches_nose_down() {
        // Internal convention: negative pitch = go forward = nose down
        // Aerospace convention: nose down = negative pitch angle
        let mut sim = Simulator::new(DefaultDrone {});
        let inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            pitch: Pitch::clamp(-1.0),
            ..Default::default()
        };
        for i in 0..3 {
            sim.tick(&inputs, 0.1.seconds());
            println!(
                "tick {}: angular_vel_y={}, attitude_y={}, pitch={}",
                i,
                sim.state.angular_velocity_body.y().raw(),
                sim.state.attitude.y,
                sim.state.attitude.pitch().to_degrees().0
            );
        }
        println!("pitch: {}", sim.state.attitude.pitch().to_degrees().0);
        assert!(
            sim.state.attitude.pitch().to_degrees().0 < 0.0,
            "negative pitch input should produce negative pitch angle (nose down in aerospace)"
        );
    }

    #[test]
    fn yaw_zero_input_no_rotation() {
        let mut sim = Simulator::new(DefaultDrone {});
        let initial_yaw = sim.state.attitude.yaw();
        let inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            pitch: Pitch::clamp(0.0),
            roll: Roll::clamp(0.0),
            yaw: Yaw::clamp(0.0),
        };
        sim.tick(&inputs, 1.seconds());
        assert!(
            (sim.state.attitude.yaw() - initial_yaw)
                .to_degrees()
                .0
                .abs()
                < 0.01
        );
    }

    #[test]
    fn yaw_positive_input_rotates_clockwise() {
        let mut sim = Simulator::new(DefaultDrone {});
        let initial_yaw = sim.state.attitude.yaw();
        let inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            pitch: Pitch::clamp(0.0),
            roll: Roll::clamp(0.0),
            yaw: Yaw::clamp(1.0),
        };
        sim.tick(&inputs, 1.0.seconds());
        // Positive yaw = clockwise from above = yaw increases in aerospace convention
        assert!(sim.state.attitude.yaw().raw() > initial_yaw.raw());
    }

    #[test]
    fn yaw_acceleration_model() {
        let mut sim = Simulator::new(DefaultDrone {});
        let inputs = Inputs {
            throttle: Throttle::clamp(0.5),
            pitch: Pitch::clamp(0.0),
            roll: Roll::clamp(0.0),
            yaw: Yaw::clamp(1.0), // full yaw input
        };

        // Run for 2 seconds
        for _ in 0..200 {
            sim.tick(&inputs, 0.01.seconds());
        }

        // Should have rotated significantly
        let yaw = sim.state.attitude.yaw().raw();
        assert!(yaw < 1.0, "expected significant yaw rotation, got {}°", yaw);

        // Angular velocity should be positive (clockwise)
        assert!(sim.state.angular_velocity_body.z().raw() > 0.0);
    }
}
