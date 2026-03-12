use std::ops::{Add, Mul, Sub};

use crate::{
    simulator::types::{vec2::Vec2, vec3::Vec3, velocity_ned::VelocityNED},
    units::{
        acceleration::{Acceleration, AccelerationLiteral},
        angles::Radians,
        units::{Seconds, Velocity},
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
#[derive(Copy, Clone, Debug)]
pub struct BodyFrameAcceleration(Acceleration3D);

impl BodyFrameAcceleration {
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
    pub fn rotate_to_world_frame_by(self, angle: Radians) -> WorldFrameAcceleration {
        let rotated = self.0.rotate(angle);
        WorldFrameAcceleration::new(rotated.x(), rotated.y(), rotated.z())
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct WorldFrameAcceleration(Acceleration3D);

impl WorldFrameAcceleration {
    pub fn new(north: Acceleration, east: Acceleration, down: Acceleration) -> Self {
        Self(Acceleration3D::new(north, east, down))
    }
    pub fn zero() -> Self {
        Self::new(Acceleration(0.0), Acceleration(0.0), Acceleration(0.0))
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
        Acceleration(self.to_raw_vec3().norm())
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
        Self(Acceleration3D::new(
            self.north().clamping(-clamp, clamp),
            self.east(),
            self.down(),
        ))
    }
    pub fn clamping_east(self, clamp: Acceleration) -> Self {
        Self(Acceleration3D::new(
            self.north(),
            self.east().clamping(-clamp, clamp),
            self.down(),
        ))
    }
    pub fn clamping_down(self, clamp: Acceleration) -> Self {
        Self(Acceleration3D::new(
            self.north(),
            self.east(),
            self.down().clamping(-clamp, clamp),
        ))
    }
}

impl Add for WorldFrameAcceleration {
    type Output = WorldFrameAcceleration;

    fn add(self, rhs: WorldFrameAcceleration) -> Self::Output {
        WorldFrameAcceleration(Acceleration3D::new(
            self.north() + rhs.north(),
            self.east() + rhs.east(),
            self.down() + rhs.down(),
        ))
    }
}

impl Sub for WorldFrameAcceleration {
    type Output = WorldFrameAcceleration;

    fn sub(self, rhs: Self) -> Self::Output {
        WorldFrameAcceleration(self.0 - rhs.0)
    }
}

impl Mul<Seconds> for WorldFrameAcceleration {
    type Output = VelocityNED;

    fn mul(self, time: Seconds) -> Self::Output {
        VelocityNED::new(self.north() * time, self.east() * time, self.down() * time)
    }
}

impl From<WorldFrameAcceleration> for Vec3<Acceleration> {
    fn from(value: WorldFrameAcceleration) -> Self {
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
