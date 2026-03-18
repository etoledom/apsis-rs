use crate::{impl_debug_unit, impl_initializable, impl_raw_representable, impl_units_arithmetics};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct PerSecond(pub f64); // 1/s

impl_initializable!(PerSecond);
impl_raw_representable!(PerSecond);
impl_units_arithmetics!(PerSecond);
impl_debug_unit!(PerSecond, "1/s");
