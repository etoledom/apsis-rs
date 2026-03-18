use crate::{impl_debug_unit, impl_initializable, impl_raw_representable, impl_units_arithmetics};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct SecondsCube(f64);

impl_initializable!(SecondsCube);
impl_raw_representable!(SecondsCube);
impl_units_arithmetics!(SecondsCube);
impl_debug_unit!(SecondsCube, "s^3");
