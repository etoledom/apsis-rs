use crate::{
    simulator::{
        drone::Drone,
        force_model::{
            context::Context, core_mechanics_models::CoreMechanicsModel, force_model::ForceModel,
        },
        inputs::Inputs,
        state::State,
        types::acceleration_3d::{WorldFrameAcceleration, WorldFrameGroundSpeed},
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

    pub fn tick(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let acceleration = self.net_acceleration(delta_t, inputs);
        self.integrate(acceleration, delta_t);
        self.update_heading(delta_t, inputs);
        self.update_horizontal_position(self.state.velocity_ned.ground_speed(), delta_t);
        self.update_battery_state(delta_t, inputs);
    }

    fn update_heading(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let new_heading_rate = self.drone.heading_rate(inputs.yaw);
        let average = (self.state.heading_rate + new_heading_rate) / 2.0;
        self.state.heading_rate = new_heading_rate;

        self.state.heading = (average * delta_t) + self.state.heading;
    }

    fn update_battery_state(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let battery_drain_pct = self.drone.battery_drain_pct(inputs.throttle) * delta_t.0;
        self.state.battery_pct -= battery_drain_pct;
        self.state.battery_pct = self.state.battery_pct.max(0.0);
    }

    fn net_acceleration(&self, delta_t: Seconds, inputs: &Inputs) -> WorldFrameAcceleration {
        let context = Context::new(&self.state, inputs, &self.drone);

        let external_acceleration = self
            .external_flight_mechanics_models
            .iter()
            .fold(WorldFrameAcceleration::zero(), |acceleration, force| {
                acceleration + force.acceleration_contribution(&context, delta_t)
            });

        self.core_mechanics_models
            .acceleration_contribution(&context, delta_t)
            + external_acceleration
    }

    fn integrate(&mut self, acceleration: WorldFrameAcceleration, delta_t: Seconds) {
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
        ground_sleed: WorldFrameGroundSpeed,
        delta_t: Seconds,
    ) {
        // Distance traveled North/East
        let north_distance = ground_sleed.north() * delta_t;
        let east_distance = ground_sleed.east() * delta_t;

        // Lat/Lon delta (simple spherical approx)
        self.state.latitude += north_distance / EARTH_RADIUS;
        self.state.longitude += east_distance / (EARTH_RADIUS * self.state.latitude.cos());
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::{
        simulator::{
            default_drone::DefaultDrone,
            types::{
                pitch::Pitch, position_ned::PositionNed, roll::Roll, throttle::Throttle, yaw::Yaw,
            },
        },
        units::{consts::G_EARTH, units::Meters},
    };

    use super::*;

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
            simulator.tick(Seconds(0.02), &inputs);
        }

        assert_relative_eq!(simulator.state.altitude.0, 1.0);
        assert_relative_eq!(simulator.state.velocity_ned.down().0, 0.0);
    }

    #[test]
    fn forward_terminal_velocity() {
        let drone = DefaultDrone {};
        let inputs = Inputs {
            throttle: drone.hover_throttle(),
            pitch: Pitch::clamp(1.0),
            roll: Roll::clamp(0.0),
            yaw: Yaw::clamp(0.0),
        };

        // V_up = -sqrt_root( G*(TWR - 1)*sin(pitch) / k ) ; where: TWR = thrust to wait ration; k = vertical drag coeff
        let expected = (G_EARTH
            * (drone.thrust_to_waight_ratio() - 1.0)
            * drone.max_pitch().to_radians().sin()
            / drone.drag_coefficient().forward.value())
        .sqrt();

        let mut simulator = Simulator::new(drone);
        simulator.state = State {
            altitude: Meters(1.0),
            ..Default::default()
        };

        for _ in 0..400 {
            simulator.tick(Seconds(0.01), &inputs);
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
        let drone = DefaultDrone {};
        let hover_throttle = drone.hover_throttle();

        let mut simulator = Simulator::new(drone);
        simulator.state.heading = crate::units::angles::Degrees(0.0); // North

        for _ in 0..100 {
            simulator.tick(
                Seconds(0.01),
                &Inputs {
                    throttle: hover_throttle,
                    pitch: Pitch::clamp(0.8),
                    roll: Roll::clamp(0.0),
                    yaw: Yaw::clamp(1.0),
                },
            );
        }

        assert_relative_eq!(simulator.state.heading.0, 44.77, epsilon = 1e-2); // Near 45 degrees as specified.
    }

    #[test]
    fn test_vertical_terminal_velocity() {
        let drone = DefaultDrone {};

        let mut simulator = Simulator::new(drone);
        simulator.state.heading = crate::units::angles::Degrees(0.0); // North

        let inputs = Inputs {
            throttle: Throttle::max(),
            ..Default::default()
        };

        // V_up = -sqrt_root( G(TWR - 1) / k ) ; where: TWR = thrust to wait ration; k = vertical drag coeff
        let expected = -(G_EARTH * (drone.thrust_to_waight_ratio() - 1.0)
            / drone.drag_coefficient().vertical.value())
        .sqrt();

        for _ in 0..150 {
            simulator.tick(Seconds(0.01), &inputs);
        }

        assert_relative_eq!(
            simulator.state.velocity_ned.down().0,
            expected.0,
            epsilon = 1e-2
        );

        for _ in 0..10 {
            simulator.tick(Seconds(0.01), &inputs);
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
            simulator.tick(Seconds(0.01), &input);
        }

        // (G / drag_coef_vertical)^2
        let expected = (G_EARTH.0 / drone.drag_coefficient().vertical.value().0).sqrt();

        assert_relative_eq!(
            simulator.state.velocity_ned.down().0,
            expected,
            epsilon = 1e-2
        );
    }
}
