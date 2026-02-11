use std::fmt::Display;

use crate::{
    mission_controller::setpoint::SetPoint,
    simulator::state::State,
    units::units::{Meters, Seconds, Velocity},
};

#[derive(Debug)]
pub enum SegmentDef {
    Idle {
        duration: Seconds,
    },
    Climb {
        target_altitude: Meters,
        climb_rate: Velocity,
    },
}

impl Display for SegmentDef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SegmentDef::Idle { duration } => write!(f, "Idle ({}s)", duration.0),
            SegmentDef::Climb {
                target_altitude,
                climb_rate,
            } => write!(f, "Climb ({}m)", target_altitude.0),
        }
    }
}

impl SegmentDef {
    pub fn to_setpoint(&self, state: &State) -> SetPoint {
        match self {
            SegmentDef::Idle { duration } => SetPoint {
                target_altitude: state.altitude,
                target_heading: state.heading,
            },
            SegmentDef::Climb {
                target_altitude,
                climb_rate,
            } => SetPoint {
                target_altitude: target_altitude.clone(),
                target_heading: state.heading,
            },
        }
    }

    pub fn conditions_met(&self, segment_time: Seconds, state: &State) -> bool {
        match self {
            SegmentDef::Idle { duration } => check_idle(duration, &segment_time),
            SegmentDef::Climb {
                target_altitude,
                climb_rate,
            } => check_climb(target_altitude, climb_rate, state),
        }
    }
}

fn check_idle(duration: &Seconds, segment_duration: &Seconds) -> bool {
    duration <= segment_duration
}

fn check_climb(target_altitude: &Meters, climb_rate: &Velocity, state: &State) -> bool {
    let altitude_diff = target_altitude.0 - state.altitude.0;
    altitude_diff < 0.1 && altitude_diff > -0.1
}
