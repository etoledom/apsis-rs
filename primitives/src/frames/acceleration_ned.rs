use std::ops::Mul;

use crate::{
    frames::{AccelerationFrd, Ned, VelocityNed},
    impl_ned_vec3, impl_units_arithmetics,
    math::{Quaternion, Vec3},
    traits::{Initializable, RawRepresentable},
    units::{Acceleration, AccelerationLiteral, Seconds},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct AccelerationNed(Vec3<Acceleration>);

impl AccelerationNed {
    pub fn zero() -> Self {
        Self::default()
    }

    pub fn from_down(down: Acceleration) -> Self {
        Self(Vec3 {
            z: down,
            ..Default::default()
        })
    }

    pub fn norm(&self) -> Acceleration {
        Acceleration::new(self.to_raw_vec3().norm())
    }

    pub fn normalized(self) -> Vec3<f64> {
        self.to_raw_vec3().normalized()
    }

    pub fn to_raw_vec3(&self) -> Vec3<f64> {
        Vec3 {
            x: self.north().raw(),
            y: self.east().raw(),
            z: self.down().raw(),
        }
    }

    pub fn setting_down(self, down: Acceleration) -> Self {
        Self(Vec3 {
            x: self.north(),
            y: self.east(),
            z: down,
        })
    }

    pub fn clamping_north(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_x(-clamp, clamp))
    }

    pub fn clamping_east(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_y(-clamp, clamp))
    }

    pub fn clamping_down(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_z(-clamp, clamp))
    }

    pub fn to_frd(self, rotation: &Quaternion) -> AccelerationFrd {
        let qv = Quaternion::pure(self.north().raw(), self.east().raw(), self.down().raw());
        let rotated = rotation.conjugate().rotate(qv);
        AccelerationFrd::new(rotated.x.mps2(), rotated.y.mps2(), rotated.z.mps2())
    }
}

impl_units_arithmetics!(AccelerationNed);
impl_ned_vec3!(AccelerationNed, Acceleration);

impl Mul<Seconds> for AccelerationNed {
    type Output = VelocityNed;

    fn mul(self, time: Seconds) -> Self::Output {
        VelocityNed::new(self.north() * time, self.east() * time, self.down() * time)
    }
}

impl From<AccelerationNed> for Vec3<Acceleration> {
    fn from(value: AccelerationNed) -> Self {
        Vec3 {
            x: value.north(),
            y: value.east(),
            z: value.down(),
        }
    }
}
