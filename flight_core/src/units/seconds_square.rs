use crate::{impl_debug_unit, impl_initializable, impl_raw_representable, impl_units_arithmetics};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct SecondsSquare(f64);

impl_initializable!(SecondsSquare);
impl_raw_representable!(SecondsSquare);
impl_units_arithmetics!(SecondsSquare);
impl_debug_unit!(SecondsSquare, "s^2");
