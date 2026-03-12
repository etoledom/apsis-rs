use crate::units::{
    Meters, Seconds, Velocity,
    acceleration::Acceleration,
    angles::{AngularVelocity, Radians},
};

pub trait RawRepresentable {
    fn raw(&self) -> f64;
}

pub trait Initializable {
    fn new(value: f64) -> Self;
}

macro_rules! impl_raw_representable {
    ($($t:ty)*) => {
        $(
            impl RawRepresentable for $t {
                fn raw(&self) -> f64 {
                    self.0
                }
            }
        )*
    };
}

impl_raw_representable! { Meters Seconds Radians AngularVelocity Velocity Acceleration }

macro_rules! impl_initializable {
    ($($t:ty)*) => {
        $(
            impl Initializable for $t {
                fn new(value: f64) -> Self {
                    Self(value)
                }
            }
        )*
    };
    (clamped: $($t:ty)*) => {
        $(
            impl Initializable for $t {
                fn new(value: f64) -> Self {
                    Self::clamp(value)
                }
            }
        )*
    };
}

impl_initializable! { Meters Seconds Radians AngularVelocity Velocity }
