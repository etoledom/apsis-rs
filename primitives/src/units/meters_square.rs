use crate::{
    impl_initializable, impl_raw_representable, impl_units_arithmetics, traits::Initializable,
    units::Meters,
};

#[repr(transparent)]
#[derive(Copy, Clone, Default, PartialEq, PartialOrd, Debug)]
pub struct MetersSquare(f64);

impl MetersSquare {
    pub fn sqrt(self) -> Meters {
        Meters::new(self.0.sqrt())
    }
}

impl_initializable!(MetersSquare);
impl_raw_representable!(MetersSquare);
impl_units_arithmetics!(MetersSquare);
