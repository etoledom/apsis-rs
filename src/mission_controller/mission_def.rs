use crate::mission_controller::segment_def::SegmentDef;

pub struct MissionDef {
    segments: Vec<SegmentDef>,
}

impl MissionDef {
    pub fn new(segments: Vec<SegmentDef>) -> Self {
        Self { segments }
    }

    pub fn segment_at(&self, index: usize) -> Option<&SegmentDef> {
        if index < self.segments.len() {
            Some(&self.segments[index])
        } else {
            None
        }
    }
}
