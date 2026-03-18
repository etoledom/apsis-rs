use std::ops::Add;

use crate::{math::Vec2, units::Velocity};

#[derive(Default, Clone, Copy)]
pub struct GroundVelocityNed(Vec2<Velocity>);

impl GroundVelocityNed {
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

impl Add for GroundVelocityNed {
    type Output = GroundVelocityNed;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}
