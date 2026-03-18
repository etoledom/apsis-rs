use primitives::control::*;

#[derive(Default, Clone, Copy)]
pub struct Inputs {
    pub throttle: Throttle,
    pub pitch: Pitch, // Front / back tilt
    pub roll: Roll,   // Side tilt
    pub yaw: Yaw,
}
