use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Debug, Clone, Copy, Default)]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

pub trait SquareRootable {
    fn sqrt(self) -> Self;
}

impl SquareRootable for f64 {
    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }
}

impl<T: Mul<T, Output = T> + Add<T, Output = T> + Div<T, Output = T> + SquareRootable + Copy>
    Vec3<T>
{
    pub fn norm(&self) -> T {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalized(self) -> Self {
        let n = self.norm();
        Self {
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        }
    }
}

impl<T: Neg<Output = T>> Neg for Vec3<T> {
    type Output = Vec3<T>;

    fn neg(self) -> Self::Output {
        Vec3 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl<T: Sub<T, Output = T>> Sub for Vec3<T> {
    type Output = Vec3<T>;

    fn sub(self, rhs: Self) -> Self::Output {
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<T: Mul<f64, Output = T>> Mul<f64> for Vec3<T> {
    type Output = Vec3<T>;

    fn mul(self, rhs: f64) -> Self::Output {
        Vec3 {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

// Dot multiplication

pub trait DotMult {
    type Output;

    fn dot(self, rhs: Self) -> Self::Output;
}

impl<T: Mul<T, Output = T> + Add<T, Output = T>> DotMult for Vec3<T> {
    type Output = T;

    fn dot(self, rhs: Self) -> Self::Output {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

// Cross multiplication

pub trait CrossMult {
    fn cross(self, rhs: Self) -> Self;
}

impl<T: Mul<T, Output = T> + Sub<T, Output = T> + Copy> CrossMult for Vec3<T> {
    fn cross(self, rhs: Self) -> Self {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }
}
