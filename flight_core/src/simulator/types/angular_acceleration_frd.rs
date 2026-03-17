use std::ops::Mul;

use crate::{
    simulator::types::{angular_velocity_frd::AngularVelocityFrd, vec3::Vec3},
    units::{angles::AngularAcceleration, units::Seconds},
};

#[derive(Debug, Clone, Copy, Default)]
pub struct AngularAccelerationFrd(Vec3<AngularAcceleration>);

impl AngularAccelerationFrd {
    pub fn new(x: AngularAcceleration, y: AngularAcceleration, z: AngularAcceleration) -> Self {
        Self(Vec3 { x, y, z })
    }

    pub fn x(&self) -> AngularAcceleration {
        self.0.x
    }
    pub fn y(&self) -> AngularAcceleration {
        self.0.y
    }
    pub fn z(&self) -> AngularAcceleration {
        self.0.z
    }
}

impl Mul<Seconds> for AngularAccelerationFrd {
    type Output = AngularVelocityFrd;

    fn mul(self, rhs: Seconds) -> Self::Output {
        AngularVelocityFrd::new(self.x() * rhs, self.y() * rhs, self.z() * rhs)
    }
}
