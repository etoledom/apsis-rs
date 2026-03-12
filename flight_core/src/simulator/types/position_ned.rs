use std::ops::{Add, AddAssign, Mul, Sub};

use crate::{
    simulator::types::vec3::Vec3,
    units::units::{Meters, MettersLiteral},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct PositionNed(Vec3<Meters>);

impl PositionNed {
    pub fn new(north: Meters, east: Meters, down: Meters) -> Self {
        Self(Vec3 {
            x: north,
            y: east,
            z: down,
        })
    }
    pub fn zero() -> Self {
        Self::new(0.meters(), 0.meters(), 0.meters())
    }
    pub fn from_altitude_ned(down: Meters) -> Self {
        Self::new(0.meters(), 0.meters(), down)
    }
    pub fn north(&self) -> Meters {
        self.0.x
    }
    pub fn east(&self) -> Meters {
        self.0.y
    }
    pub fn down(&self) -> Meters {
        self.0.z
    }
    pub fn set_north(&mut self, value: Meters) {
        self.0.x = value;
    }
    pub fn set_east(&mut self, value: Meters) {
        self.0.y = value;
    }
    pub fn set_down(&mut self, value: Meters) {
        self.0.z = value;
    }
}

impl Mul<f64> for PositionNed {
    type Output = PositionNed;

    fn mul(self, rhs: f64) -> Self::Output {
        PositionNed::new(self.north() * rhs, self.east() * rhs, self.down() * rhs)
    }
}

impl Add for PositionNed {
    type Output = PositionNed;

    fn add(self, rhs: Self) -> Self::Output {
        PositionNed(self.0 + rhs.0)
    }
}

impl Sub for PositionNed {
    type Output = PositionNed;

    fn sub(self, rhs: Self) -> Self::Output {
        PositionNed(self.0 - rhs.0)
    }
}

impl AddAssign for PositionNed {
    fn add_assign(&mut self, rhs: Self) {
        self.0.x.0 += rhs.0.x.0;
        self.0.y.0 += rhs.0.y.0;
        self.0.z.0 += rhs.0.z.0;
    }
}
