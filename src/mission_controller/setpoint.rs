use crate::units::{angles::Degrees, units::Meters};

#[derive(Clone, Copy)]
pub struct SetPoint {
    pub target_altitude: Meters,
    pub target_heading: Degrees,
}
