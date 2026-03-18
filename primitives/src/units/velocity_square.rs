use std::ops::{Div, Mul};

use crate::{
    impl_initializable, impl_raw_representable, impl_units_arithmetics,
    traits::{Initializable, RawRepresentable},
    units::{Acceleration, Meters, PerMeter, Velocity},
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct VelocitySquare(pub f64); // m^2/s^2 or (m/s)^2

impl VelocitySquare {
    pub fn sqrt(self) -> Velocity {
        Velocity::new(self.0.sqrt())
    }
}

impl_initializable!(VelocitySquare);
impl_raw_representable!(VelocitySquare);
impl_units_arithmetics!(VelocitySquare);

/// (m/s)^2 / m/s = m/s
impl Div<Velocity> for VelocitySquare {
    type Output = Velocity;

    fn div(self, rhs: Velocity) -> Self::Output {
        Velocity::new(self.0 / rhs.raw())
    }
}

/// (m/s)^2 / m/s^2 = m
impl Div<Acceleration> for VelocitySquare {
    type Output = Meters;

    fn div(self, rhs: Acceleration) -> Self::Output {
        Meters::new(self.0 / rhs.raw())
    }
}

/// (m/s)^2 * 1/m = m/s^2
impl Mul<PerMeter> for VelocitySquare {
    type Output = Acceleration;

    fn mul(self, rhs: PerMeter) -> Self::Output {
        Acceleration::new(self.0 * rhs.raw())
    }
}
