use flight_core::controller::{AxisTarget, FlightTarget};
use primitives::{frames::PositionNed, traits::UnitsArithmetics, units::MetersLiteral};

pub enum ReturnToLaunchPhase {
    SafeAltitude,
    Return,
    Decend,
    Completed,
}

pub struct ReturnToLaunch {
    state: ReturnToLaunchPhase,
}

impl ReturnToLaunch {
    pub fn new() -> Self {
        Self {
            state: ReturnToLaunchPhase::SafeAltitude,
        }
    }
    pub fn update(&mut self, position: PositionNed) -> FlightTarget {
        let target = match self.state {
            ReturnToLaunchPhase::SafeAltitude => {
                if position.down().abs() - 10.meters() < 0.5.meters() {
                    self.state = ReturnToLaunchPhase::Return;
                }
                FlightTarget {
                    north: AxisTarget::Loiter,
                    east: AxisTarget::Loiter,
                    down: AxisTarget::Position(-10.meters()),
                }
            }
            ReturnToLaunchPhase::Return => {
                let horizontal_dist = (position.north().pow_2() + position.east().pow_2()).sqrt();
                if horizontal_dist.abs() < 0.5.meters() {
                    self.state = ReturnToLaunchPhase::Decend;
                }
                FlightTarget {
                    north: AxisTarget::Position(0.meters()),
                    east: AxisTarget::Position(0.meters()),
                    down: AxisTarget::Position(-10.meters()),
                }
            }
            ReturnToLaunchPhase::Decend => {
                if position.down().abs() < 0.1.meters() {
                    self.state = ReturnToLaunchPhase::Completed;
                }
                FlightTarget {
                    north: AxisTarget::Position(0.meters()),
                    east: AxisTarget::Position(0.meters()),
                    down: AxisTarget::Position(0.meters()),
                }
            }
            ReturnToLaunchPhase::Completed => todo!(),
        };

        target
    }

    pub fn reset(&mut self) {
        self.state = ReturnToLaunchPhase::SafeAltitude;
    }

    pub fn completed(&self) -> bool {
        matches!(self.state, ReturnToLaunchPhase::Completed)
    }
}
