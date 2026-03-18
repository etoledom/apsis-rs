use crate::{
    impl_initializable, impl_raw_representable, impl_units_arithmetics, traits::Initializable,
    units::Acceleration,
};

#[repr(transparent)]
#[derive(Copy, Clone, Default, Debug, PartialEq, PartialOrd)]
pub struct AccelerationSquare(f64); // m²/s⁴

impl AccelerationSquare {
    pub fn sqrt(self) -> Acceleration {
        Acceleration::new(self.0.sqrt())
    }
}

impl_raw_representable!(AccelerationSquare);
impl_initializable!(AccelerationSquare);
impl_units_arithmetics!(AccelerationSquare);
