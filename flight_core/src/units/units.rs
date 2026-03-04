use std::{
    cmp::Ordering,
    fmt,
    ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign},
};

use crate::units::{
    acceleration::Acceleration,
    angles::{Degrees, Radians},
};

#[repr(transparent)]
#[derive(Copy, Clone, PartialEq)]
pub struct Seconds(pub f64);

impl Seconds {
    pub fn zero() -> Self {
        Self(0.0)
    }
}

impl Default for Seconds {
    fn default() -> Self {
        Self::zero()
    }
}

pub trait SecondsLiteral {
    fn seconds(self) -> Seconds;
}

impl SecondsLiteral for f64 {
    fn seconds(self) -> Seconds {
        Seconds(self)
    }
}

impl SecondsLiteral for u32 {
    fn seconds(self) -> Seconds {
        Seconds(self as f64)
    }
}

impl PartialOrd for Seconds {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Default, PartialEq, Debug)]
pub struct Meters(pub f64);

impl Meters {
    pub fn zero() -> Self {
        Self(0.0)
    }
    pub fn max(self, max_value: Meters) -> Meters {
        Meters(self.0.max(max_value.0))
    }
    pub fn raw(&self) -> f64 {
        self.0
    }
}

impl Neg for Meters {
    type Output = Meters;

    fn neg(self) -> Self::Output {
        Meters(-self.0)
    }
}

pub trait MettersLiteral {
    fn meters(self) -> Meters;
}

impl MettersLiteral for i32 {
    fn meters(self) -> Meters {
        Meters(self as f64)
    }
}

impl MettersLiteral for f64 {
    fn meters(self) -> Meters {
        Meters(self)
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct PerMeter(pub f64); // 1/m

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct PerSecond(pub f64); // 1/m

#[repr(transparent)]
#[derive(Copy, Clone, Default, PartialEq, Debug)]
pub struct Velocity(pub f64); // m/s

impl Velocity {
    fn zero() -> Self {
        Self(0.0)
    }

    pub fn abs(&self) -> Self {
        Velocity(self.0.abs())
    }

    pub fn clamp(&self, min: Velocity, max: Velocity) -> Self {
        Self(self.0.clamp(min.into(), max.into()))
    }
}

impl From<Velocity> for f64 {
    fn from(value: Velocity) -> Self {
        value.0
    }
}

pub trait VelocityLiteral {
    fn mps(self) -> Velocity;
}

impl VelocityLiteral for i32 {
    fn mps(self) -> Velocity {
        Velocity(self as f64)
    }
}

impl VelocityLiteral for f64 {
    fn mps(self) -> Velocity {
        Velocity(self)
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default)]
pub struct VelocitySquare(pub f64); // m2/s2 or (m/s)2

impl Add for VelocitySquare {
    type Output = VelocitySquare;

    fn add(self, rhs: Self) -> Self::Output {
        VelocitySquare(self.0 + rhs.0)
    }
}

impl Sub<Seconds> for Seconds {
    type Output = Seconds;

    fn sub(self, rhs: Seconds) -> Self::Output {
        Seconds(self.0 - rhs.0)
    }
}

impl Add<Seconds> for Seconds {
    type Output = Seconds;

    fn add(self, rhs: Seconds) -> Self::Output {
        Seconds(self.0 + rhs.0)
    }
}

impl Div<Seconds> for Meters {
    type Output = Velocity;

    fn div(self, rhs: Seconds) -> Self::Output {
        Velocity(self.0 / rhs.0)
    }
}

impl Mul<Seconds> for Velocity {
    type Output = Meters;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Meters(self.0 * rhs.0)
    }
}

impl Add<Meters> for Meters {
    type Output = Meters;

    fn add(self, rhs: Meters) -> Self::Output {
        Meters(self.0 + rhs.0)
    }
}

impl Add<Velocity> for Velocity {
    type Output = Velocity;

    fn add(self, rhs: Velocity) -> Self::Output {
        Velocity(self.0 + rhs.0)
    }
}

impl Sub<Velocity> for Velocity {
    type Output = Velocity;

    fn sub(self, rhs: Velocity) -> Self::Output {
        Velocity(self.0 - rhs.0)
    }
}

impl Div<f64> for Velocity {
    type Output = Velocity;

    fn div(self, rhs: f64) -> Self::Output {
        Velocity(self.0 / rhs)
    }
}

impl Mul<f64> for Velocity {
    type Output = Velocity;

    fn mul(self, rhs: f64) -> Self::Output {
        Velocity(self.0 * rhs)
    }
}

impl Div<Meters> for Meters {
    type Output = f64;

    fn div(self, rhs: Meters) -> Self::Output {
        self.0 / rhs.0
    }
}

impl Mul<f64> for Meters {
    type Output = Meters;

    fn mul(self, rhs: f64) -> Self::Output {
        Meters(self.0 * rhs)
    }
}

impl Sub<Meters> for Meters {
    type Output = Meters;

    fn sub(self, rhs: Meters) -> Self::Output {
        Meters(self.0 - rhs.0)
    }
}

impl PartialOrd for Meters {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

impl Mul<Velocity> for Velocity {
    type Output = VelocitySquare;

    fn mul(self, rhs: Velocity) -> Self::Output {
        VelocitySquare(self.0 * rhs.0)
    }
}

impl Mul<PerMeter> for VelocitySquare {
    // (m/s)2 * 1/m = m/s2
    type Output = Acceleration;

    fn mul(self, rhs: PerMeter) -> Self::Output {
        Acceleration(self.0 * rhs.0)
    }
}

impl Neg for VelocitySquare {
    type Output = VelocitySquare;

    fn neg(self) -> Self::Output {
        VelocitySquare(-self.0)
    }
}

impl VelocitySquare {
    pub fn sqrt(self) -> Velocity {
        Velocity(self.0.sqrt())
    }
}

impl Neg for Velocity {
    type Output = Velocity;

    fn neg(self) -> Self::Output {
        Velocity(-self.0)
    }
}

impl AddAssign for Meters {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Div<f64> for Meters {
    type Output = Meters;

    fn div(self, rhs: f64) -> Self::Output {
        Meters(self.0 / rhs)
    }
}

impl AddAssign for Velocity {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0
    }
}

impl SubAssign for Velocity {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0
    }
}

#[macro_export]
macro_rules! impl_debug_unit {
    ($ty:ty, $suffix:expr) => {
        impl fmt::Display for $ty {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{:.2}{}", self.0, $suffix)
            }
        }
    };
}

impl_debug_unit!(Seconds, "s");
impl_debug_unit!(Meters, "m");
impl_debug_unit!(Velocity, "m/s");
impl_debug_unit!(Acceleration, "m/s2");
impl_debug_unit!(Degrees, "°");
impl_debug_unit!(Radians, "");
