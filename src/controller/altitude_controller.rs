use crate::{
    controller::pid::{LinearGain, PID},
    simulator::{state::State, types::throttle::Throttle},
    units::units::{Meters, Seconds, Velocity},
};

pub struct AltitudeController {
    altitude_target: Meters,
    altitude_pid: PID<Meters, Velocity, LinearGain, LinearGain, LinearGain>,
    climb_pid: PID<Velocity, Throttle, LinearGain, LinearGain, LinearGain>,
}

impl AltitudeController {
    pub fn new(altitude_target: Meters) -> Self {
        Self {
            altitude_target,
            altitude_pid: PID::new(LinearGain::new(1.0), LinearGain::zero(), LinearGain::zero()),
            climb_pid: PID::new(
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
