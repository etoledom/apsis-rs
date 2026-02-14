use std::ops::{Add, Mul, Sub};

use crate::{
    simulator::types::{vec2::Vec2, velocity_ned::VelocityNed},
    units::{
        acceleration::{Acceleration, AccelerationLiteral},
        angles::Radians,
        units::{Seconds, Velocity},
    },
};

#[derive(Copy, Clone, Debug)]
pub struct Acceleration3D {
    pub vertical: Acceleration,
    pub forward: Acceleration,
    pub right: Acceleration,
}

impl Acceleration3D {
    pub fn new(forward: Acceleration, right: Acceleration, vertical: Acceleration) -> Self {
        Self {
            vertical,
            forward,
            right,
        }
    }

    pub fn vertical(vertical: Acceleration) -> Self {
        Self {
            vertical,
            forward: 0.mps2(),
            right: 0.mps2(),
        }
    }

    pub fn ground_acceleration(&self) -> Vec2<Acceleration> {
        Vec2::new(self.right, self.forward)
    }

    fn rotate(&self, angle: Radians) -> Acceleration3D {
        let ground_acceleration = self.ground_acceleration();
        let rotated = ground_acceleration.rotate(angle);
        Acceleration3D::new(rotated.y, rotated.x, self.vertical)
    }

    fn to_world_frame(self) -> WorldFrameAcceleration {
        WorldFrameAcceleration(self)
    }
}

impl Add for Acceleration3D {
    type Output = Acceleration3D;

    fn add(self, rhs: Acceleration3D) -> Self::Output {
        Acceleration3D {
            vertical: self.vertical + rhs.vertical,
            forward: self.forward + rhs.forward,
            right: self.right + rhs.right,
        }
    }
}

impl Sub for Acceleration3D {
    type Output = Acceleration3D;

    fn sub(self, rhs: Acceleration3D) -> Self::Output {
        Acceleration3D {
            vertical: self.vertical - rhs.vertical,
            forward: self.forward - rhs.forward,
            right: self.right - rhs.right,
        }
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub struct BodyFrameAcceleration(Acceleration3D);

impl BodyFrameAcceleration {
    pub fn new(forward: Acceleration, right: Acceleration, vertical: Acceleration) -> Self {
        Self(Acceleration3D {
            vertical,
            forward,
            right,
        })
    }
    pub fn forward(&self) -> Acceleration {
        self.0.forward
    }
    pub fn right(&self) -> Acceleration {
        self.0.right
    }
    pub fn vertical(&self) -> Acceleration {
        self.0.vertical
    }
    pub fn from_vertical(vertical: Acceleration) -> Self {
        Self(Acceleration3D::vertical(vertical))
    }
    pub fn rotate_to_world_frame_by(self, angle: Radians) -> WorldFrameAcceleration {
        let rotated = self.0.rotate(angle);
        WorldFrameAcceleration::new(rotated.forward, rotated.right, rotated.vertical)
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub struct WorldFrameAcceleration(Acceleration3D);

impl WorldFrameAcceleration {
    pub fn new(north: Acceleration, east: Acceleration, vertical: Acceleration) -> Self {
        Self(Acceleration3D {
            vertical,
            forward: north,
            right: east,
        })
    }
    pub fn zero() -> Self {
        Self::new(Acceleration(0.0), Acceleration(0.0), Acceleration(0.0))
    }
    pub fn north(&self) -> Acceleration {
        self.0.forward
    }
    pub fn east(&self) -> Acceleration {
        self.0.right
    }
    pub fn vertical(&self) -> Acceleration {
        self.0.vertical
    }
    pub fn update_vertical(&mut self, new_value: Acceleration) {
        self.0.vertical = new_value;
    }
    pub fn from_vertical(vertical: Acceleration) -> Self {
        Self(Acceleration3D::vertical(vertical))
    }
}

impl Add for WorldFrameAcceleration {
    type Output = WorldFrameAcceleration;

    fn add(self, rhs: WorldFrameAcceleration) -> Self::Output {
        WorldFrameAcceleration(Acceleration3D {
            vertical: self.vertical() + rhs.vertical(),
            forward: self.north() + rhs.north(),
            right: self.east() + rhs.east(),
        })
    }
}

impl Mul<Seconds> for WorldFrameAcceleration {
    type Output = VelocityNed;

    fn mul(self, time: Seconds) -> Self::Output {
        VelocityNed::new(
            self.north() * time,
            self.east() * time,
            self.vertical() * time,
        )
    }
}

impl WorldFrameAcceleration {
    pub fn ground_acceleration(&self) -> WorldFrameGroundAcceleration {
        WorldFrameGroundAcceleration::new(self.north(), self.east())
    }
}

pub struct WorldFrameGroundAcceleration(Vec2<Acceleration>);
impl WorldFrameGroundAcceleration {
    pub fn new(north: Acceleration, east: Acceleration) -> Self {
        Self(Vec2 { x: east, y: north })
    }
    pub fn north(&self) -> Acceleration {
        self.0.y
    }

    pub fn east(&self) -> Acceleration {
        self.0.x
    }
}

#[derive(Default, Clone, Copy)]
pub struct WorldFrameGroundSpeed(Vec2<Velocity>);
impl WorldFrameGroundSpeed {
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

impl Mul<Seconds> for WorldFrameGroundAcceleration {
    type Output = WorldFrameGroundSpeed;

    fn mul(self, rhs: Seconds) -> Self::Output {
        WorldFrameGroundSpeed::new(self.north() * rhs, self.east() * rhs)
    }
}

impl Add for WorldFrameGroundSpeed {
    type Output = WorldFrameGroundSpeed;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.north() + rhs.north(), self.east() + rhs.east())
    }
}
