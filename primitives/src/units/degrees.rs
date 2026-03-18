use crate::{
    impl_debug_unit, impl_initializable, impl_literal, impl_raw_representable,
    impl_units_arithmetics, units::Radians,
};

#[repr(transparent)]
#[derive(Copy, Clone, Default, PartialEq, PartialOrd)]
pub struct Degrees(pub f64);

impl Degrees {
    pub fn to_radians(&self) -> Radians {
        Radians(self.0.to_radians())
    }

    pub fn sin(self) -> f64 {
        self.0.sin()
    }

    pub fn cos(self) -> f64 {
        self.0.cos()
    }
}

impl_initializable!(Degrees);
impl_raw_representable!(Degrees);
impl_units_arithmetics!(Degrees);
impl_literal!(Degrees, degrees, DegreesLiteral);
impl_debug_unit!(Degrees, "°");
