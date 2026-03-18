use std::ops::{AddAssign, Sub};

use crate::{math::Vec3, units::AngularVelocity};

#[derive(Debug, Clone, Copy, Default)]
pub struct AngularVelocityFrd(Vec3<AngularVelocity>);

impl AngularVelocityFrd {
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

impl AddAssign for AngularVelocityFrd {
    fn add_assign(&mut self, rhs: Self) {
        self.0.x += rhs.x();
        self.0.y += rhs.y();
        self.0.z += rhs.z();
    }
}

impl Sub for AngularVelocityFrd {
    type Output = AngularVelocityFrd;

    fn sub(self, rhs: Self) -> Self::Output {
        AngularVelocityFrd(self.0 - rhs.0)
    }
}
