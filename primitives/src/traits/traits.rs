#[macro_export]
macro_rules! impl_literal {
    ($target:ident, $fn_name:ident, $trait_name:ident) => {
        pub trait $trait_name {
            fn $fn_name(self) -> $target;
        }

        impl $trait_name for f64 {
            fn $fn_name(self) -> $target {
                $target(self)
            }
        }

        impl $trait_name for i32 {
            fn $fn_name(self) -> $target {
                $target(self as f64)
            }
        }
    };
}

#[macro_export]
macro_rules! impl_debug_unit {
    ($ty:ty, $suffix:expr) => {
        impl core::fmt::Display for $ty {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(f, "{:.2}{}", self.0, $suffix)
            }
        }
    };
}
