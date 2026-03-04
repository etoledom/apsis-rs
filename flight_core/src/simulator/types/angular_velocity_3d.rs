use std::ops::{AddAssign, Sub};

use crate::{simulator::types::vec3::Vec3, units::angles::AngularVelocity};

#[derive(Debug, Clone, Copy, Default)]
pub struct AngularVelocity3D(Vec3<AngularVelocity>);

impl AngularVelocity3D {
    pub fn new(
        x: impl Into<AngularVelocity>,
        y: impl Into<AngularVelocity>,
        z: impl Into<AngularVelocity>,
    ) -> Self {
        Self(Vec3 {
            x: x.into(),
            y: y.into(),
            z: z.into(),
        })
    }
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
    pub fn x(&self) -> AngularVelocity {
        self.0.x
    }
    pub fn y(&self) -> AngularVelocity {
        self.0.y
    }
    pub fn z(&self) -> AngularVelocity {
        self.0.z
    }
    pub fn set_x(&mut self, x: AngularVelocity) {
        self.0.x = x;
    }
    pub fn set_y(&mut self, y: AngularVelocity) {
        self.0.y = y;
    }
    pub fn set_z(&mut self, z: AngularVelocity) {
        self.0.z = z;
    }
}

impl AddAssign for AngularVelocity3D {
    fn add_assign(&mut self, rhs: Self) {
        self.0.x += rhs.x();
        self.0.y += rhs.y();
        self.0.z += rhs.z();
    }
}

impl Sub for AngularVelocity3D {
    type Output = AngularVelocity3D;

    fn sub(self, rhs: Self) -> Self::Output {
        AngularVelocity3D(self.0 - rhs.0)
    }
}
