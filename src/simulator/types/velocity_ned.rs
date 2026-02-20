use std::ops::{AddAssign, Mul, Sub};

use crate::{
    simulator::types::{
        acceleration_3d::WorldFrameGroundSpeed, position_ned::PositionNed, vec3::Vec3,
    },
    units::units::{Seconds, Velocity, VelocityLiteral},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct VelocityNED(Vec3<Velocity>);

impl VelocityNED {
    pub fn new(north: Velocity, east: Velocity, down: Velocity) -> Self {
        Self(Vec3 {
            x: north,
            y: east,
            z: down,
        })
    }
    pub fn zero() -> VelocityNED {
        Self::new(0.mps(), 0.mps(), 0.mps())
    }
    pub fn north(&self) -> Velocity {
        self.0.x
    }
    pub fn east(&self) -> Velocity {
        self.0.y
    }
    pub fn down(&self) -> Velocity {
        self.0.z
    }

    pub fn update_down(&mut self, new_value: Velocity) {
        self.0.z = new_value;
    }
    pub fn ground_speed(&self) -> WorldFrameGroundSpeed {
        WorldFrameGroundSpeed::new(self.north(), self.east())
    }
}

impl Mul<Seconds> for VelocityNED {
    type Output = PositionNed;

    fn mul(self, time: Seconds) -> Self::Output {
        PositionNed::new(self.north() * time, self.east() * time, self.down() * time)
    }
}

impl AddAssign for VelocityNED {
    fn add_assign(&mut self, rhs: Self) {
        self.0.x.0 += rhs.0.x.0;
        self.0.y.0 += rhs.0.y.0;
        self.0.z.0 += rhs.0.z.0;
    }
}

impl Sub for VelocityNED {
    type Output = VelocityNED;

    fn sub(self, rhs: Self) -> Self::Output {
        VelocityNED(self.0 - rhs.0)
    }
}
