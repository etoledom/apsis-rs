use std::{
    f64::consts::FRAC_PI_2,
    ops::{Add, Mul, Neg},
};

use crate::units::{
    angles::{Degrees, Radians},
    traits::RawRepresentable,
};
use crate::{
    simulator::types::angular_velocity_frd::AngularVelocityFrd, units::traits::UnitsArithmetics,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn pure(x: f64, y: f64, z: f64) -> Self {
        Quaternion { w: 0.0, x, y, z }
    }
    pub fn identity() -> Self {
        Default::default()
    }

    // ||q|| -> square_root( w^2 + x^2 + y^2 + z^2 )
    fn n(&self) -> f64 {
        (self.w.powi(2) + self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    /// q / ||q||
    pub fn normalized(self) -> Self {
        let n = self.n();

        Self {
            w: self.w / n,
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        }
    }

    pub fn omega(velocity: AngularVelocityFrd) -> Self {
        Self {
            w: 0.0,
            x: velocity.x().raw(),
            y: velocity.y().raw(),
            z: velocity.z().raw(),
        }
    }

    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn rotate(&self, q_v: Quaternion) -> Self {
        *self * q_v * self.conjugate()
    }

    /// Returns pitch in radians. Positive nose-down.
    pub fn pitch(&self) -> Radians {
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        // Clamp for numerical stability
        if sinp.abs() >= 1.0 {
            Radians(sinp.signum() * FRAC_PI_2)
        } else {
            Radians(sinp.asin())
        }
    }

    pub fn roll(&self) -> Radians {
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        Radians(sinr_cosp.atan2(cosr_cosp))
    }

    pub fn yaw(&self) -> Radians {
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        Radians(siny_cosp.atan2(cosy_cosp))
    }

    pub fn yaw_normalized(&self) -> Degrees {
        let yaw = self.yaw().to_degrees();
        Degrees((yaw.0 % 360.0 + 360.0) % 360.0)
    }

    pub fn to_euler(&self) -> (Radians, Radians, Radians) {
        let roll = self.roll();
        let pitch = self.pitch();
        let yaw = self.yaw();

        (roll, pitch, yaw)
    }

    /// Build quaternion from roll, pitch, yaw in radians
    pub fn from_euler(roll: Radians, pitch: Radians, yaw: Radians) -> Self {
        let (sr, cr) = (roll.0 * 0.5).sin_cos();
        let (sp, cp) = (pitch.0 * 0.5).sin_cos();
        let (sy, cy) = (yaw.0 * 0.5).sin_cos();

        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    pub fn from_yaw(yaw: Radians) -> Self {
        Self::from_euler(Radians(0.0), Radians(0.0), yaw)
    }

    pub fn from_pitch(pitch: Radians) -> Self {
        Self::from_euler(Radians(0.0), pitch, Radians(0.0))
    }

    pub fn clamped(self, max_pitch: Radians, max_roll: Radians) -> Self {
        let (roll, pitch, yaw) = self.to_euler();
        Self::from_euler(
            roll.clamping(-max_roll, max_roll),
            pitch.clamping(-max_pitch, max_pitch),
            yaw,
        )
    }

    pub fn from_rotation_matrix(r: [[f64; 3]; 3]) -> Quaternion {
        let trace = r[0][0] + r[1][1] + r[2][2];

        if trace > 0.0 {
            let s = 0.5 / (trace + 1.0).sqrt();
            Quaternion {
                w: 0.25 / s,
                x: (r[2][1] - r[1][2]) * s,
                y: (r[0][2] - r[2][0]) * s,
                z: (r[1][0] - r[0][1]) * s,
            }
        } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
            let s = 2.0 * (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt();
            Quaternion {
                w: (r[2][1] - r[1][2]) / s,
                x: 0.25 * s,
                y: (r[0][1] + r[1][0]) / s,
                z: (r[0][2] + r[2][0]) / s,
            }
        } else if r[1][1] > r[2][2] {
            let s = 2.0 * (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt();
            Quaternion {
                w: (r[0][2] - r[2][0]) / s,
                x: (r[0][1] + r[1][0]) / s,
                y: 0.25 * s,
                z: (r[1][2] + r[2][1]) / s,
            }
        } else {
            let s = 2.0 * (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt();
            Quaternion {
                w: (r[1][0] - r[0][1]) / s,
                x: (r[0][2] + r[2][0]) / s,
                y: (r[1][2] + r[2][1]) / s,
                z: 0.25 * s,
            }
        }
    }

    pub fn to_rotation_matrix(&self) -> [[f64; 3]; 3] {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }
}

impl Default for Quaternion {
    /// Default is a Quaternion identity.
    fn default() -> Self {
        Self {
            w: 1.0,
            x: Default::default(),
            y: Default::default(),
            z: Default::default(),
        }
    }
}

impl Mul for Quaternion {
    type Output = Quaternion;

    /// Hamilton product
    fn mul(self, rhs: Self) -> Self::Output {
        Quaternion {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl Mul<f64> for Quaternion {
    type Output = Quaternion;

    fn mul(self, rhs: f64) -> Self::Output {
        Quaternion {
            w: self.w * rhs,
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl Add for Quaternion {
    type Output = Quaternion;

    fn add(self, rhs: Self) -> Self::Output {
        Quaternion {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Neg for Quaternion {
    type Output = Quaternion;

    fn neg(self) -> Self::Output {
        Quaternion {
            w: -self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hamilton_product() {
        let theta: f64 = 0.1;
        let q = Quaternion {
            w: (theta / 2.0).cos(),
            x: 0.0,
            y: (theta / 2.0).sin(),
            z: 0.0,
        };
        let v = Quaternion::pure(0.0, 0.0, -1.0);
        let result = q * v * q.conjugate();
        println!("result: {:?}", result);

        assert!(result.x < 0.0);
    }
}
