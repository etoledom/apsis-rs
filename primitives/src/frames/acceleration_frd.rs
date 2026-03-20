use crate::{
    frames::{AccelerationNed, Ned},
    impl_units_arithmetics,
    math::{Quaternion, Vec3},
    traits::RawRepresentable,
    units::{Acceleration, AccelerationLiteral},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct AccelerationFrd(Vec3<Acceleration>);

impl AccelerationFrd {
    pub fn new(forward: Acceleration, right: Acceleration, down: Acceleration) -> Self {
        Self(Vec3 {
            x: forward,
            y: right,
            z: down,
        })
    }
    pub fn forward(&self) -> Acceleration {
        self.0.x
    }
    pub fn right(&self) -> Acceleration {
        self.0.y
    }
    pub fn down(&self) -> Acceleration {
        self.0.z
    }
    pub fn from_down(down: Acceleration) -> Self {
        Self(Vec3 {
            z: down,
            ..Default::default()
        })
    }
    pub fn clamping_forward(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_x(-clamp, clamp))
    }
    pub fn clamping_right(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_y(-clamp, clamp))
    }
    pub fn clamping_down(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_z(-clamp, clamp))
    }

    pub fn to_ned(self, rotation: &Quaternion) -> AccelerationNed {
        let qv = Quaternion::pure(self.forward().raw(), self.right().raw(), self.down().raw());
        let rotated = rotation.rotate(qv);
        AccelerationNed::new(rotated.x.mps2(), rotated.y.mps2(), rotated.z.mps2())
    }
}

impl_units_arithmetics!(AccelerationFrd);
