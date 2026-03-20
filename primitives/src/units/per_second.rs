use std::ops::Div;

use crate::{
    impl_debug_unit, impl_initializable, impl_raw_representable, impl_units_arithmetics,
    traits::Initializable, units::Seconds,
};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct PerSecond(pub f64); // 1/s

impl_initializable!(PerSecond);
impl_raw_representable!(PerSecond);
impl_units_arithmetics!(PerSecond);
impl_debug_unit!(PerSecond, "1/s");

// 1 / 1/s  = s
impl Div<PerSecond> for f64 {
    type Output = Seconds;

    fn div(self, rhs: PerSecond) -> Self::Output {
        Seconds::new(self / rhs.0)
    }
}
