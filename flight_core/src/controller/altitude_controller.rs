use crate::{
    controller::{
        gain::LinearGain,
        pid::{AltitudePID, ClimbPID},
    },
    simulator::{state::State, types::throttle::Throttle},
    units::units::{Meters, Seconds},
};

pub struct AltitudeController {
    altitude_target: Meters,
    altitude_pid: AltitudePID,
    climb_pid: ClimbPID,
}

impl AltitudeController {
    pub fn new(altitude_target: Meters) -> Self {
        Self {
            altitude_target,
            altitude_pid: AltitudePID::new(
                LinearGain::new(1.0),
                LinearGain::zero(),
                LinearGain::zero(),
            ),
            climb_pid: ClimbPID::new(
                LinearGain::new(0.4),
                LinearGain::new(0.6),
                LinearGain::zero(),
            ),
        }
    }

    pub fn update(&mut self, telemetry: &State, dt: Seconds) -> Throttle {
        let altitude = -telemetry.position_ned.down();
        let velocity = -telemetry.velocity_ned.down();

        let error_altitude = self.altitude_target - altitude;

        let climb_target = self
            .altitude_pid
            .update(error_altitude, dt)
            .clamp(-4.0, 4.0);

        let climb_error = climb_target - velocity;

        self.climb_pid.update(climb_error, dt)
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        simulator::types::position_ned::PositionNed,
        units::units::{MettersLiteral, SecondsLiteral},
    };

    use super::*;

    #[test]
    fn test_controller_throttles_up() {
        let state = State {
            ..Default::default()
        };
        let mut controller = AltitudeController::new(10.meters());
        let throttle = controller.update(&state, 1.seconds());

        assert_eq!(throttle.get(), 1.0);
    }

    #[test]
    fn test_controller_throttles_down() {
        let state = State {
            position_ned: PositionNed::new(Meters::zero(), Meters::zero(), Meters(-10.0)),
            ..Default::default()
        };
        let mut controller = AltitudeController::new(0.meters());
        let throttle = controller.update(&state, 1.seconds());

        assert_eq!(throttle.get(), 0.0);
    }
}
