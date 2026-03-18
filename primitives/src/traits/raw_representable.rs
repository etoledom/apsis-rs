pub trait RawRepresentable {
    fn raw(&self) -> f64;
}

#[macro_export]
macro_rules! impl_raw_representable {
    ($($t:ty)*) => {
        $(
            impl crate::traits::RawRepresentable for $t {
                #[inline(always)]
                fn raw(&self) -> f64 {
                    self.0
                }
            }
        )*
    };
}
