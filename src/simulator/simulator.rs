use crate::{
    simulator::{
        drone::{BodyFrameAcceleration, Drone},
        inputs::Inputs,
        state::State,
        types::vec2::Vec2,
    },
    units::{
        consts::{EARTH_RADIUS, G_EARTH},
        units::{MettersLiteral, Seconds, Velocity},
    },
};

pub struct Simulator<DroneType: Drone> {
    pub state: State,
    pub drone: DroneType,
}

impl<DroneType: Drone> Simulator<DroneType> {
    pub fn new(drone: DroneType) -> Self {
        Self {
            state: Default::default(),
            drone,
        }
    }

    pub fn tick(&mut self, delta_t: Seconds, inputs: &Inputs) {
        self.update_heading(delta_t, inputs);
        self.update_altitude(delta_t, inputs);
        self.update_airframe_velocity(delta_t, inputs);
        self.update_horizontal_position(delta_t);
        self.update_battery_state(delta_t, inputs);
    }

    fn update_heading(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let new_heading_rate = self.drone.heading_rate(inputs.yaw);
        let average = (self.state.heading_rate + new_heading_rate) / 2.0;
        self.state.heading_rate = new_heading_rate;

        self.state.heading = (average * delta_t) + self.state.heading;
    }

    fn update_altitude(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let vertical_acceleration = self
            .drone
            .body_frame_acceleration(inputs.throttle, inputs.pitch, inputs.roll)
            .vertical;

        let new_vertical_velocity = self.state.vertical_velocity + vertical_acceleration * delta_t;
        let average_vertical_velocity =
            (self.state.vertical_velocity + new_vertical_velocity) / 2.0;

        let final_gained_altitude = average_vertical_velocity * delta_t;
        let final_total_altitude = final_gained_altitude + self.state.altitude;
        self.state.altitude = final_total_altitude.max(0.meters())
    }

    fn update_battery_state(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let battery_drain_pct = self.drone.battery_drain_pct(inputs.throttle) * delta_t.0;
        self.state.battery_pct -= battery_drain_pct;
        self.state.battery_pct = self.state.battery_pct.max(0.0);
    }

    fn update_airframe_velocity(&mut self, delta_t: Seconds, inputs: &Inputs) {
        let body_frame_acceleration =
            self.drone
                .body_frame_acceleration(inputs.throttle, inputs.pitch, inputs.roll);

        let world_frame_acceleration =
            body_frame_acceleration.rotate(self.state.heading.to_radians());

        let drag_acceleration = BodyFrameAcceleration {
            vertical: -(self.state.vertical_velocity.abs() * self.state.vertical_velocity)
                * self.drone.drag_coefficient().vertical.value(),
            forward: -(self.state.ground_speed_ned.y.abs() * self.state.ground_speed_ned.y)
                * self.drone.drag_coefficient().horizontal.value(),
            right: -(self.state.ground_speed_ned.x.abs() * self.state.ground_speed_ned.x)
                * self.drone.drag_coefficient().horizontal.value(),
        };

        let mut net_acceleration = world_frame_acceleration + drag_acceleration;
        net_acceleration.vertical = net_acceleration.vertical - G_EARTH;

        // let ground_acceleration = Vec2::new(net_acceleration.right, net_acceleration.forward)
        //     .rotate(self.state.heading.to_radians());

        self.state.vertical_velocity =
            net_acceleration.vertical * delta_t + self.state.vertical_velocity;

        if self.state.vertical_velocity.0 < 0.0 && self.state.altitude.0 <= 0.0 {
            self.state.vertical_velocity = Velocity(0.0);
        }

        self.state.ground_speed_ned =
            net_acceleration.ground_acceleration() * delta_t + self.state.ground_speed_ned;
    }

    fn update_horizontal_position(&mut self, delta_t: Seconds) {
        // Distance traveled North/East
        let north_distance = self.state.ground_speed_ned.y * delta_t;
        let east_distance = self.state.ground_speed_ned.x * delta_t;

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
        units::units::{Meters, VelocityLiteral},
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
    fn stable_move_forward() {
        let drone = DefaultDrone {};
        let inputs = Inputs {
            throttle: Throttle::clamp(drone.hover_throttle().get() + 0.095),
            pitch: Pitch::clamp(0.8),
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

        assert_relative_eq!(simulator.state.vertical_velocity.0, 0.0, epsilon = 1e-2);
        assert_relative_eq!(simulator.state.ground_speed_ned.x.0, 0.0);
        assert_relative_eq!(simulator.state.ground_speed_ned.y.0, 14.24, epsilon = 1e-2);
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

        assert_relative_eq!(simulator.state.vertical_velocity.0, 3.16, epsilon = 1e-2); // Near 45 degrees as specified.

        for _ in 0..10 {
            simulator.tick(
                Seconds(0.01),
                &Inputs {
                    throttle: Throttle::max(),
                    ..Default::default()
                },
            );
        }

        assert_relative_eq!(simulator.state.vertical_velocity.0, 3.16, epsilon = 1e-2); // Near 45 degrees as specified.
    }
}
