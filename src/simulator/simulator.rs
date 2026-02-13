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
        units::{MettersLiteral, Seconds, Velocity},
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
        self.update_heading(delta_t, inputs);
        self.update_airframe_velocity(delta_t, inputs);
        self.update_altitude(self.state.vertical_velocity, delta_t);
        self.update_horizontal_position(self.state.ground_speed_ned, delta_t);
        self.update_battery_state(delta_t, inputs);
    }

    fn update_heading(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let new_heading_rate = self.drone.heading_rate(inputs.yaw);
        let average = (self.state.heading_rate + new_heading_rate) / 2.0;
        self.state.heading_rate = new_heading_rate;

        self.state.heading = (average * delta_t) + self.state.heading;
    }

    fn update_altitude(&mut self, vertical_velocity: Velocity, delta_t: Seconds) {
        let final_gained_altitude = vertical_velocity * delta_t;
        self.state.altitude = self.state.altitude + final_gained_altitude.max(0.meters())
    }

    fn update_battery_state(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let battery_drain_pct = self.drone.battery_drain_pct(inputs.throttle) * delta_t.0;
        self.state.battery_pct -= battery_drain_pct;
        self.state.battery_pct = self.state.battery_pct.max(0.0);
    }

    fn update_airframe_velocity(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let context = Context::new(&self.state, inputs, &self.drone);

        let external_acceleration = self
            .external_flight_mechanics_models
            .iter()
            .fold(WorldFrameAcceleration::zero(), |acceleration, force| {
                acceleration + force.acceleration_contribution(&context, delta_t)
            });

        let net_acceleration = self
            .core_mechanics_models
            .acceleration_contribution(&context, delta_t)
            + external_acceleration;

        self.state.vertical_velocity =
            net_acceleration.vertical() * delta_t + self.state.vertical_velocity;

        if self.state.vertical_velocity.0 < 0.0 && self.state.altitude.0 <= 0.0 {
            self.state.vertical_velocity = Velocity(0.0);
        }

        self.state.ground_speed_ned =
            net_acceleration.ground_acceleration() * delta_t + self.state.ground_speed_ned;
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
            types::{pitch::Pitch, roll::Roll, throttle::Throttle, yaw::Yaw},
        },
        units::units::Meters,
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
        simulator.state.altitude = Meters(1.0);

        for _ in 0..100 {
            simulator.tick(Seconds(0.02), &inputs);
        }

        assert_relative_eq!(simulator.state.altitude.0, 1.0);
        assert_relative_eq!(simulator.state.vertical_velocity.0, 0.0);
    }

    #[test]
    fn move_forward() {
        let drone = DefaultDrone {};
        let inputs = Inputs {
            throttle: Throttle::clamp(drone.hover_throttle().get()),
            pitch: Pitch::clamp(1.0),
            roll: Roll::clamp(0.0),
            yaw: Yaw::clamp(0.0),
        };

        let mut simulator = Simulator::new(drone);
        simulator.state = State {
            altitude: Meters(1.0),
            ..Default::default()
        };

        for _ in 0..100 {
            simulator.tick(Seconds(0.02), &inputs);
        }

        assert_relative_eq!(simulator.state.ground_speed_ned.east().0, 0.0);
        assert_relative_eq!(
            simulator.state.ground_speed_ned.north().0,
            4.15,
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

        for _ in 0..100 {
            simulator.tick(
                Seconds(0.01),
                &Inputs {
                    throttle: Throttle::max(),
                    ..Default::default()
                },
            );
        }

        assert_relative_eq!(simulator.state.vertical_velocity.0, 3.98, epsilon = 1e-2); // Near 45 degrees as specified.

        for _ in 0..10 {
            simulator.tick(
                Seconds(0.01),
                &Inputs {
                    throttle: Throttle::max(),
                    ..Default::default()
                },
            );
        }

        assert_relative_eq!(simulator.state.vertical_velocity.0, 4.0, epsilon = 1e-2); // Near 45 degrees as specified.
    }
}
