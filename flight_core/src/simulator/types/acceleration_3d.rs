use std::ops::{Add, Mul, Sub};

use crate::{
    Quaternion,
    simulator::types::{vec2::Vec2, vec3::Vec3, velocity_ned::VelocityNed},
    units::{
        Acceleration, AccelerationLiteral, Seconds, Velocity,
        angles::Radians,
        traits::{Initializable, RawRepresentable, UnitsArithmetics},
    },
};

#[derive(Copy, Clone, Debug, Default)]
pub struct Acceleration3D(Vec3<Acceleration>);

impl Acceleration3D {
    pub fn new(x: Acceleration, y: Acceleration, z: Acceleration) -> Self {
        Self(Vec3 { x, y, z })
    }

    pub fn down(z: Acceleration) -> Self {
        Self(Vec3 {
            x: 0.mps2(),
            y: 0.mps2(),
            z,
        })
    }

    fn ground_acceleration(&self) -> Vec2<Acceleration> {
        Vec2::new(self.0.x, self.0.y)
    }

    fn rotate(&self, angle: Radians) -> Acceleration3D {
        let ground_acceleration = self.ground_acceleration();
        let rotated = ground_acceleration.rotate(angle);
        Acceleration3D::new(rotated.y, rotated.x, self.0.z)
    }

    fn x(&self) -> Acceleration {
        self.0.x
    }

    fn y(&self) -> Acceleration {
        self.0.y
    }

    fn z(&self) -> Acceleration {
        self.0.z
    }
    pub fn clamping_x(self, clamp: Acceleration) -> Self {
        Self::new(self.x().clamping(-clamp, clamp), self.y(), self.z())
    }
    pub fn clamping_y(self, clamp: Acceleration) -> Self {
        Self::new(self.x(), self.y().clamping(-clamp, clamp), self.z())
    }
    pub fn clamping_z(self, clamp: Acceleration) -> Self {
        Self::new(self.x(), self.y(), self.z().clamping(-clamp, clamp))
    }

    pub fn clamping_by(self, x: Acceleration, y: Acceleration, z: Acceleration) -> Self {
        Self::new(
            self.x().clamping(-x, x),
            self.y().clamping(-y, y),
            self.z().clamping(-z, z),
        )
    }
}

impl Add for Acceleration3D {
    type Output = Acceleration3D;

    fn add(self, rhs: Acceleration3D) -> Self::Output {
        Acceleration3D(Vec3 {
            x: self.0.x + rhs.0.x,
            y: self.0.y + rhs.0.y,
            z: self.0.z + rhs.0.z,
        })
    }
}

impl Sub for Acceleration3D {
    type Output = Acceleration3D;

    fn sub(self, rhs: Acceleration3D) -> Self::Output {
        Acceleration3D(Vec3 {
            x: self.0.x - rhs.0.x,
            y: self.0.y - rhs.0.y,
            z: self.0.z - rhs.0.z,
        })
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct AccelerationFrd(Acceleration3D);

impl AccelerationFrd {
    pub fn new(forward: Acceleration, right: Acceleration, down: Acceleration) -> Self {
        Self(Acceleration3D::new(forward, right, down))
    }
    pub fn forward(&self) -> Acceleration {
        self.0.x()
    }
    pub fn right(&self) -> Acceleration {
        self.0.y()
    }
    pub fn down(&self) -> Acceleration {
        self.0.z()
    }
    pub fn from_down(down: Acceleration) -> Self {
        Self(Acceleration3D::down(down))
    }
    pub fn rotate_to_world_frame_by(self, angle: Radians) -> AccelerationNed {
        let rotated = self.0.rotate(angle);
        AccelerationNed::new(rotated.x(), rotated.y(), rotated.z())
    }
    pub fn clamping_forward(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_x(clamp))
    }
    pub fn clamping_right(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_y(clamp))
    }
    pub fn clamping_down(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_z(clamp))
    }
    pub fn clamping_by(
        self,
        forward: Acceleration,
        right: Acceleration,
        down: Acceleration,
    ) -> Self {
        Self(self.0.clamping_by(forward, right, down))
    }
    pub fn to_world_frame(self, rotation: &Quaternion) -> AccelerationNed {
        let qv = Quaternion::pure(self.forward().raw(), self.right().raw(), self.down().raw());
        let rotated = rotation.rotate(qv);
        AccelerationNed::new(rotated.x.mps2(), rotated.y.mps2(), rotated.z.mps2())
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct AccelerationNed(Acceleration3D);

impl AccelerationNed {
    pub fn new(north: Acceleration, east: Acceleration, down: Acceleration) -> Self {
        Self(Acceleration3D::new(north, east, down))
    }
    pub fn zero() -> Self {
        Self::new(
            Acceleration::zero(),
            Acceleration::zero(),
            Acceleration::zero(),
        )
    }
    pub fn north(&self) -> Acceleration {
        self.0.x()
    }
    pub fn east(&self) -> Acceleration {
        self.0.y()
    }
    pub fn down(&self) -> Acceleration {
        self.0.z()
    }

    pub fn from_down(down: Acceleration) -> Self {
        Self(Acceleration3D::down(down))
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
        Self(Acceleration3D::new(self.0.x(), self.0.y(), down))
    }
    pub fn clamping_north(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_x(clamp))
    }
    pub fn clamping_east(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_y(clamp))
    }
    pub fn clamping_down(self, clamp: Acceleration) -> Self {
        Self(self.0.clamping_z(clamp))
    }
    pub fn clamping_by(self, north: Acceleration, east: Acceleration, down: Acceleration) -> Self {
        Self(self.0.clamping_by(north, east, down))
    }
    pub fn to_body_frame(self, rotation: &Quaternion) -> AccelerationFrd {
        let qv = Quaternion::pure(self.north().raw(), self.east().raw(), self.down().raw());
        let rotated = rotation.conjugate().rotate(qv);
        AccelerationFrd::new(rotated.x.mps2(), rotated.y.mps2(), rotated.z.mps2())
    }
}

impl Add for AccelerationNed {
    type Output = AccelerationNed;

    fn add(self, rhs: AccelerationNed) -> Self::Output {
        AccelerationNed(Acceleration3D::new(
            self.north() + rhs.north(),
            self.east() + rhs.east(),
            self.down() + rhs.down(),
        ))
    }
}

impl Sub for AccelerationNed {
    type Output = AccelerationNed;

    fn sub(self, rhs: Self) -> Self::Output {
        AccelerationNed(self.0 - rhs.0)
    }
}

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

#[derive(Default, Clone, Copy)]
pub struct WorldFrameGroundVelocity(Vec2<Velocity>);
impl WorldFrameGroundVelocity {
    pub fn new(north: Velocity, east: Velocity) -> Self {
        Self(Vec2 { x: east, y: north })
    }
    pub fn north(&self) -> Velocity {
        self.0.y
    }

    pub fn east(&self) -> Velocity {
        self.0.x
    }
}

impl Add for WorldFrameGroundVelocity {
    type Output = WorldFrameGroundVelocity;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.north() + rhs.north(), self.east() + rhs.east())
    }
}
