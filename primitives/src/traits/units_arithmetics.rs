use crate::traits::{initializable::Initializable, raw_representable::RawRepresentable};

pub trait UnitsArithmetics {
    fn zero() -> Self;
    fn max(self, max_value: Self) -> Self;
    fn min(&self, min_value: Self) -> Self;
    fn abs(self) -> Self;
    fn sin(&self) -> f64;
    fn cos(&self) -> f64;
    fn tan(&self) -> f64;
    fn clamping(self, min: Self, max: Self) -> Self;
}

impl<T> UnitsArithmetics for T
where
    T: Initializable + RawRepresentable,
{
    #[inline(always)]
    fn zero() -> Self {
        Self::new(0.0)
    }
    #[inline(always)]
    fn max(self, max_value: Self) -> Self {
        Self::new(self.raw().max(max_value.raw()))
    }
    #[inline(always)]
    fn min(&self, min_value: Self) -> Self {
        Self::new(self.raw().min(min_value.raw()))
    }
    #[inline(always)]
    fn clamping(self, min: Self, max: Self) -> Self {
        Self::new(self.raw().clamp(min.raw(), max.raw()))
    }
    #[inline(always)]
    fn abs(self) -> Self {
        Self::new(self.raw().abs())
    }
    #[inline(always)]
    fn sin(&self) -> f64 {
        self.raw().sin()
    }
    #[inline(always)]
    fn cos(&self) -> f64 {
        self.raw().cos()
    }
    #[inline(always)]
    fn tan(&self) -> f64 {
        self.raw().tan()
    }
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
