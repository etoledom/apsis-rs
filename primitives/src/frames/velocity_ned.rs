use std::ops::Mul;

use crate::{
    frames::{GroundVelocityNed, Ned, PositionNed, VelocityFrd},
    impl_ned_vec3, impl_units_arithmetics,
    math::{Quaternion, Vec3},
    traits::RawRepresentable,
    units::{Seconds, Velocity, VelocityLiteral},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct VelocityNed(Vec3<Velocity>);

impl VelocityNed {
    pub fn zero() -> Self {
        Self::default()
    }
    pub fn from_north(north: Velocity) -> Self {
        Self::new(north, 0.mps(), 0.mps())
    }
    pub fn from_east(east: Velocity) -> Self {
        Self::new(0.mps(), east, 0.mps())
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
    pub fn ground_velocity(&self) -> GroundVelocityNed {
        GroundVelocityNed::new(self.north(), self.east())
    }
    pub fn to_frd(self, rotation: &Quaternion) -> VelocityFrd {
        let qv = Quaternion::pure(self.north().raw(), self.east().raw(), self.down().raw());
        let rotated = rotation.conjugate().rotate(qv);
        VelocityFrd::new(rotated.x.mps(), rotated.y.mps(), rotated.z.mps())
    }

    // pub fn dot<T>(self, other: Vec3<T>) {}
}

impl_ned_vec3!(VelocityNed, Velocity);
impl_units_arithmetics!(VelocityNed);

impl Mul<Seconds> for VelocityNed {
    type Output = PositionNed;

    fn mul(self, time: Seconds) -> Self::Output {
        PositionNed::new(self.north() * time, self.east() * time, self.down() * time)
    }
}
