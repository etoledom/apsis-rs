use crate::{
    PositionNed,
    controller::pid::PositionPID,
    units::{Meters, Velocity, VelocityLiteral, units::Seconds},
};

pub struct PositionController {
    pub target: PositionNed,
    north_pid: PositionPID,
    east_pid: PositionPID,
    altitude_pid: PositionPID,
}

impl PositionController {
    pub fn new(target: PositionNed) -> Self {
        Self {
            target,
            north_pid: PositionPID::new(1, 0, 0.1).with_limits(3.mps()),
            east_pid: PositionPID::new(1, 0, 0.1).with_limits(3.mps()),
            altitude_pid: PositionPID::new(5, 0, 0.1).with_limits(5.mps()),
        }
    }

    pub fn update_north(&mut self, current: Meters, dt: Seconds) -> Velocity {
        let error = self.target.north() - current;
        self.north_pid.update(error, dt).clamping(-5.mps(), 5.mps())
    }

    pub fn update_east(&mut self, current: Meters, dt: Seconds) -> Velocity {
        let error = self.target.east() - current;
        self.east_pid.update(error, dt).clamping(-5.mps(), 5.mps())
    }

    pub fn update_down(&mut self, current: Meters, dt: Seconds) -> Velocity {
        let error = self.target.down() - current;

        let target = self
            .altitude_pid
            .update(error, dt)
            .clamping(-5.mps(), 5.mps());

        target
    }
}

// #[cfg(test)]
// mod tests {
//     use crate::{
//         simulator::types::position_ned::PositionNed,
//         units::units::{MettersLiteral, SecondsLiteral},
//     };

//     use super::*;

//     #[test]
//     fn test_controller_throttles_up() {
//         let state = State {
//             ..Default::default()
//         };
//         let mut controller = PositionController::new(10.meters());
//         let throttle = controller.update(&state, 1.seconds());

//         assert_eq!(throttle.get(), 1.0);
//     }

//     #[test]
//     fn test_controller_throttles_down() {
//         let state = State {
//             position_ned: PositionNed::new(Meters::zero(), Meters::zero(), Meters(-10.0)),
//             ..Default::default()
//         };
//         let mut controller = AltitudeController::new(0.meters());
//         let throttle = controller.update(&state, 1.seconds());

//         assert_eq!(throttle.get(), 0.0);
//     }
// }
