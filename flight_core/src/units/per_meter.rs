use crate::{impl_debug_unit, impl_initializable, impl_raw_representable, impl_units_arithmetics};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct PerMeter(f64); // 1/m

impl_initializable!(PerMeter);
impl_raw_representable!(PerMeter);
impl_units_arithmetics!(PerMeter);
impl_debug_unit!(PerMeter, "1/s");
