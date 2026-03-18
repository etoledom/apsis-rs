use std::ops::{Add, Mul, Sub};

use crate::units::{Acceleration, Seconds, Velocity};

#[derive(Clone, Copy, Default)]
pub struct Vec2<T> {
    pub x: T,
    pub y: T,
}

impl<T: Mul<f64, Output = T> + Add<T, Output = T> + Sub<T, Output = T> + Copy> Vec2<T> {
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

impl Mul<Seconds> for Vec2<Acceleration> {
    type Output = Vec2<Velocity>;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Vec2::new(self.x * rhs, self.y * rhs)
    }
}

impl<T: Add<T, Output = T> + Mul<f64, Output = T> + Sub<T, Output = T> + Copy> Add<Vec2<T>>
    for Vec2<T>
{
    type Output = Vec2<T>;

    fn add(self, rhs: Vec2<T>) -> Self::Output {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}
