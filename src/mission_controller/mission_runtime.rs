use crate::{
    mission_controller::{
        mission_def::{MissionDef, SegmentDef},
        setpoint::SetPoint,
    },
    simulator::state::State,
    units::units::{MettersLiteral, Seconds, SecondsLiteral, VelocityLiteral},
};

pub struct MissionRuntime {
    def: MissionDef,
    current_segment_index: usize,
    segment_start_time: Seconds,
    current_setpoint: Option<SetPoint>,
}

impl MissionRuntime {
    pub fn new() -> Self {
        let def = MissionDef::new(vec![
            SegmentDef::Idle {
                duration: 2.seconds(),
            },
            SegmentDef::Climb {
                target_altitude: 10.meters(),
                climb_rate: 5.mps(),
            },
            SegmentDef::Idle {
                duration: 10.seconds(),
            },
        ]);
        MissionRuntime {
            def,
            current_segment_index: 0,
            segment_start_time: 0.seconds(),
            current_setpoint: None,
        }
    }

    pub fn update(&mut self, time_now: Seconds, state: &State) -> Option<SetPoint> {
        if let Some(current_segment) = self.def.segment_at(self.current_segment_index) {
            if current_segment.conditions_met(time_now - self.segment_start_time, state) {
                self.current_segment_index += 1;
                self.segment_start_time = time_now;
                self.current_setpoint = None;
                return self.update(time_now, state);
            } else {
                if self.current_setpoint.is_none() {
                    self.current_setpoint = Some(current_segment.to_setpoint(state));
                }
                self.current_setpoint
            }
        } else {
            None
        }
    }

    pub fn current_segment(&self) -> Option<&SegmentDef> {
        self.def.segment_at(self.current_segment_index)
    }
}
