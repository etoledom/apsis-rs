pub trait RawRepresentable {
    fn raw(&self) -> f64;
}

pub trait Initializable {
    fn new(value: impl Into<f64>) -> Self;
}

#[macro_export]
macro_rules! impl_raw_representable {
    ($($t:ty)*) => {
        $(
            impl crate::units::traits::RawRepresentable for $t {
                fn raw(&self) -> f64 {
                    self.0
                }
            }
        )*
    };
}

#[macro_export]
macro_rules! impl_initializable {
    ($($t:ty)*) => {
        $(
            impl crate::units::traits::Initializable for $t {
                fn new(value: impl Into<f64>) -> Self {
                    Self(value.into())
                }
            }
        )*
    };
    (clamped: $($t:ty)*) => {
        $(
            impl crate::units::traits::Initializable for $t {
                fn new(value: f64) -> Self {
                    Self::clamp(value)
                }
            }
        )*
    };
}

#[macro_export]
macro_rules! impl_units_arithmetics {
    ($type:ident) => {
        impl std::ops::Add for $type {
            type Output = $type;

            fn add(self, rhs: $type) -> Self::Output {
                $type(self.0 + rhs.0)
            }
        }

        impl std::ops::Sub for $type {
            type Output = $type;

            fn sub(self, rhs: $type) -> Self::Output {
                $type(self.0 - rhs.0)
            }
        }

        impl std::ops::Mul<f64> for $type {
            type Output = $type;

            fn mul(self, rhs: f64) -> Self::Output {
                $type(self.0 * rhs)
            }
        }

        impl std::ops::Mul<$type> for f64 {
            type Output = $type;

            fn mul(self, rhs: $type) -> Self::Output {
                $type(self * rhs.0)
            }
        }

        impl std::ops::Div<f64> for $type {
            type Output = $type;

            fn div(self, rhs: f64) -> Self::Output {
                $type(self.0 / rhs)
            }
        }

        impl std::ops::AddAssign for $type {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }

        impl std::ops::SubAssign for $type {
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0
            }
        }

        impl std::ops::Neg for $type {
            type Output = $type;

            fn neg(self) -> Self::Output {
                Self(-self.0)
            }
        }
    };
}

pub trait UnitsArithmetics {
    fn zero() -> Self;
    fn max(self, max_value: Self) -> Self;
    fn min(&self, min_value: Self) -> Self;
    fn clamping(self, min: Self, max: Self) -> Self;
    fn abs(self) -> Self;
    fn sin(&self) -> f64;
    fn cos(&self) -> f64;
    fn tan(&self) -> f64;
}

impl<T> UnitsArithmetics for T
where
    T: Initializable + RawRepresentable,
{
    fn zero() -> Self {
        Self::new(0.0)
    }
    fn max(self, max_value: Self) -> Self {
        Self::new(self.raw().max(max_value.raw()))
    }
    fn min(&self, min_value: Self) -> Self {
        Self::new(self.raw().min(min_value.raw()))
    }
    fn clamping(self, min: Self, max: Self) -> Self {
        Self::new(self.raw().clamp(min.raw(), max.raw()))
    }
    fn abs(self) -> Self {
        Self::new(self.raw().abs())
    }
    fn sin(&self) -> f64 {
        self.raw().sin()
    }

    fn cos(&self) -> f64 {
        self.raw().cos()
    }

    fn tan(&self) -> f64 {
        self.raw().tan()
    }
}

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
