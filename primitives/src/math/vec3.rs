use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

use crate::traits::UnitsArithmetics;

#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
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

impl<
    T: UnitsArithmetics, // for clamping()
> Vec3<T>
{
    pub fn clamping_x(self, min: T, max: T) -> Vec3<T> {
        Vec3 {
            x: self.x.clamping(min, max),
            ..self
        }
    }

    pub fn clamping_y(self, min: T, max: T) -> Vec3<T> {
        Vec3 {
            y: self.y.clamping(min, max),
            ..self
        }
    }

    pub fn clamping_z(self, min: T, max: T) -> Vec3<T> {
        Vec3 {
            z: self.z.clamping(min, max),
            ..self
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

impl<T: Add<T, Output = T>> Add for Vec3<T> {
    type Output = Vec3<T>;

    fn add(self, rhs: Self) -> Self::Output {
        Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl<T: AddAssign<T>> AddAssign for Vec3<T> {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl<T: SubAssign<T>> SubAssign for Vec3<T> {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
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

impl<T: Mul<f64, Output = T>> Mul<Vec3<T>> for f64 {
    type Output = Vec3<T>;

    fn mul(self, rhs: Vec3<T>) -> Self::Output {
        Vec3 {
            x: rhs.x * self,
            y: rhs.y * self,
            z: rhs.z * self,
        }
    }
}

impl<T: Div<f64, Output = T>> Div<f64> for Vec3<T> {
    type Output = Vec3<T>;

    fn div(self, rhs: f64) -> Self::Output {
        Vec3 {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
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
