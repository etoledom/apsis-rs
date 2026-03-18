use crate::{
    frames::VelocityNed,
    impl_units_arithmetics,
    math::{Quaternion, Vec3},
    traits::RawRepresentable,
    units::{Velocity, VelocityLiteral},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct VelocityFrd(Vec3<Velocity>);

impl VelocityFrd {
    pub fn new(forward: Velocity, right: Velocity, down: Velocity) -> Self {
        Self(Vec3 {
            x: forward,
            y: right,
            z: down,
        })
    }
    pub fn forward(&self) -> Velocity {
        self.0.x
    }
    pub fn right(&self) -> Velocity {
        self.0.y
    }
    pub fn down(&self) -> Velocity {
        self.0.z
    }
    pub fn to_world_frame(self, rotation: &Quaternion) -> VelocityNed {
        let qv = Quaternion::pure(self.forward().raw(), self.right().raw(), self.down().raw());
        let rotated = rotation.rotate(qv);
        VelocityNed::new(rotated.x.mps(), rotated.y.mps(), rotated.z.mps())
    }
}

impl_units_arithmetics!(VelocityFrd);
