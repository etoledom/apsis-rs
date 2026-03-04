use std::ops::{AddAssign, Mul, Sub};

use crate::{
    simulator::types::{
        acceleration_3d::WorldFrameGroundVelocity, position_ned::PositionNed, vec3::Vec3,
    },
    units::units::{Seconds, Velocity, VelocityLiteral},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq)]
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
    pub fn update_north(&mut self, new_value: Velocity) {
        self.0.x = new_value;
    }
    pub fn update_east(&mut self, new_value: Velocity) {
        self.0.y = new_value;
    }
    pub fn add_down(&mut self, added_value: Velocity) {
        self.0.z += added_value;
    }
    pub fn add_north(&mut self, added_value: Velocity) {
        self.0.x += added_value;
    }
    pub fn add_east(&mut self, added_value: Velocity) {
        self.0.y += added_value;
    }
    pub fn clamp_down(&mut self, min: Velocity, max: Velocity) {
        self.0.z.clamp(min, max);
    }
    pub fn clamp_north(&mut self, min: Velocity, max: Velocity) {
        self.0.x.clamp(min, max);
    }
    pub fn clamp_east(&mut self, min: Velocity, max: Velocity) {
        self.0.y.clamp(min, max);
    }
    pub fn ground_speed(&self) -> WorldFrameGroundVelocity {
        WorldFrameGroundVelocity::new(self.north(), self.east())
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
