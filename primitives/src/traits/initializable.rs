pub trait Initializable {
    fn new(value: impl Into<f64>) -> Self;
}

#[macro_export]
macro_rules! impl_initializable {
    ($($t:ty)*) => {
        $(
            impl crate::traits::Initializable for $t {
                #[inline(always)]
                fn new(value: impl Into<f64>) -> Self {
                    Self(value.into())
                }
            }
        )*
    };
}
