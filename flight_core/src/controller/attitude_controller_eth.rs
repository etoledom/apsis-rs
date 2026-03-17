use crate::{AngularVelocityFrd, Quaternion, UnitInterval, units::angles::AngularVelocity};

/// Attitude controller using reduced-attitude (tilt-priority) approach.
/// Based on "Nonlinear Quadrocopter Attitude Control" (Brescianini et al., ETH 2013)
/// and PX4's AttitudeControl implementation.
///
/// Key difference from separate pitch/roll/yaw PIDs:
/// - Computes a single quaternion error → single 3D angular velocity command
/// - Separates tilt (pitch+roll) from yaw, giving tilt priority
/// - At non-zero yaw, a body-right command produces pure body-frame roll
///   with no cross-coupling into pitch or yaw
pub struct AttitudeController {
    /// Proportional gains applied element-wise to the attitude error vector.
    /// [roll_gain, pitch_gain, yaw_gain]
    /// Roll and pitch typically have the same gain, yaw is handled via yaw_weight.
    tilt_gain: f64,

    /// Maximum angular rate per axis [roll, pitch, yaw] in rad/s
    rate_limit: AngularVelocityFrd,

    /// Yaw priority weight ∈ [0, 1].
    /// 0 = pure tilt control (no yaw correction)
    /// 1 = full attitude control (equal yaw priority)
    /// Typical: 0.4 (from τ / τ_yaw)
    yaw_weight: UnitInterval,
}

impl AttitudeController {
    pub fn new(tilt_gain: f64, yaw_weight: UnitInterval, rate_limit: AngularVelocityFrd) -> Self {
        Self {
            tilt_gain,
            rate_limit,
            yaw_weight,
        }
    }

    /// Compute angular rate setpoint from current and target attitude.
    ///
    /// 1. Compute reduced (tilt-only) target — shortest rotation to align thrust axes
    /// 2. Blend yaw correction with reduced weight
    /// 3. Convert quaternion error to angular rate setpoint
    /// 4. Add pilot yaw rate feedforward in body frame
    pub fn update(
        &self,
        q_current: &Quaternion,
        q_target: &Quaternion,
        yaw_rate_feedforward: AngularVelocity,
    ) -> AngularVelocityFrd {
        // --- Step 1: Reduced attitude (tilt only) ---
        // Current and desired thrust directions in world frame
        let thrust_axis_current = q_current.thrust_axis_world();
        let thrust_axis_target = q_target.thrust_axis_world();

        // Quaternion that rotates ONLY the thrust axis from current to target.
        // This is a pure tilt — rotation axis lies in the horizontal plane,
        // so it has zero yaw component. This is the key insight from the ETH paper.
        let tilt_rotation =
            Quaternion::from_two_unit_vectors(thrust_axis_current, thrust_axis_target);

        // Apply tilt rotation to current attitude → reduced target (correct tilt, current yaw)
        let q_target_tilt_only = tilt_rotation * *q_current;

        // --- Step 2: Mix in yaw correction with lower priority ---
        // q_yaw_correction is the pure-yaw rotation between tilt-only and full target
        let q_yaw_correction = q_target_tilt_only.conjugate() * *q_target;
        let q_yaw_correction = if q_yaw_correction.w < 0.0 {
            -q_yaw_correction
        } else {
            q_yaw_correction
        };

        // Scale the yaw component by yaw_weight.
        // q_yaw_correction is a pure Z rotation: (cos(α/2), 0, 0, sin(α/2))
        // Scaling α by yaw_weight gives partial yaw correction.
        let w_clamped = q_yaw_correction.w.clamp(-1.0, 1.0);
        let z_clamped = q_yaw_correction.z.clamp(-1.0, 1.0);
        let yaw_half_angle_scaled = z_clamped.asin() * self.yaw_weight.get();
        let w_scaled = (w_clamped.acos() * self.yaw_weight.get()).cos();
        let q_yaw_scaled = Quaternion {
            w: w_scaled,
            x: 0.0,
            y: 0.0,
            z: yaw_half_angle_scaled.sin(),
        }
        .normalized();

        let q_target_mixed = q_target_tilt_only * q_yaw_scaled;

        // --- Step 3: Quaternion error → angular rate setpoint ---
        // Error quaternion: rotation from current to mixed target
        let q_error = q_current.conjugate() * q_target_mixed;

        // Ensure shortest path (avoid unwinding)
        let sign = if q_error.w >= 0.0 { 1.0 } else { -1.0 };

        // Convert to angular rate using sin(α/2) ≈ α/2 for small angles
        // This is equation (23) from the ETH paper: Ω_cmd = (2/τ) * sign(qe.w) * qe.xyz
        // With our gain structure: rate = gain * 2 * sign * qe.xyz
        let roll_rate = AngularVelocity(self.tilt_gain * 2.0 * sign * q_error.x)
            .clamping(-self.rate_limit.roll(), self.rate_limit.roll());
        let pitch_rate = AngularVelocity(self.tilt_gain * 2.0 * sign * q_error.y)
            .clamping(-self.rate_limit.pitch(), self.rate_limit.pitch());
        let yaw_rate = AngularVelocity(self.tilt_gain * 2.0 * sign * q_error.z)
            .clamping(-self.rate_limit.yaw(), self.rate_limit.yaw());

        // --- Step 4: Add yaw rate feedforward in body frame ---
        // Pilot yaw rate is rotation around world-Z, but rate setpoint is in body frame.
        // Project world-Z onto body frame to get the correct body-frame yaw component.
        let world_z_body = q_current.world_z_in_body();

        let rates = AngularVelocityFrd::new(
            roll_rate + yaw_rate_feedforward * world_z_body.x,
            pitch_rate + yaw_rate_feedforward * world_z_body.y,
            yaw_rate + yaw_rate_feedforward * world_z_body.z,
        );

        println!(
            "ATT_CTRL: error_roll={:.4}, error_pitch={:.4}, error_yaw={:.4}, rate_roll={:.4}, rate_pitch={:.4}, rate_yaw={:.4}",
            q_error.x,
            q_error.y,
            q_error.z,
            rates.roll().raw(),
            rates.pitch().raw(),
            rates.yaw().raw(),
        );

        rates
    }
}

#[cfg(test)]
mod tests {
    use crate::Vec3;

    use super::*;

    fn rate_limit() -> AngularVelocityFrd {
        AngularVelocityFrd::new(
            AngularVelocity(1.5),
            AngularVelocity(1.5),
            AngularVelocity(1.5),
        )
    }

    #[test]
    fn pure_tilt_produces_no_yaw_rate() {
        let controller = AttitudeController::new(5.0, UnitInterval::clamp(0.4), rate_limit());
        let q_current = Quaternion::identity();
        // Target: 10° nose down
        let angle = 10.0_f64.to_radians();
        let q_target = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: -(angle / 2.0).sin(),
            z: 0.0,
        };
        let rates = controller.update(&q_current, &q_target, AngularVelocity(0.0));
        assert!(
            rates.z().raw().abs() < 1e-6,
            "pure pitch should produce no yaw rate"
        );
    }

    #[test]
    fn body_right_at_yaw_30_produces_pure_roll_no_pitch() {
        let controller = AttitudeController::new(5.0, UnitInterval::clamp(0.4), rate_limit());

        // Current attitude: 30° yaw, level
        let yaw = 30.0_f64.to_radians();
        let q_current = Quaternion {
            w: (yaw / 2.0).cos(),
            x: 0.0,
            y: 0.0,
            z: (yaw / 2.0).sin(),
        };

        // Target: tilted in the body-right direction at 30° yaw
        // Body-right at 30° yaw = world direction (sin30, cos30, 0) = (0.5, 0.866, 0)
        // So thrust axis tilts toward (-0.5, -0.866, z).normalized
        let tilt = 10.0_f64.to_radians();
        let thrust_target = Vec3 {
            x: -tilt.sin() * yaw.sin(), // south-east component
            y: tilt.sin() * yaw.cos(),  //
            z: tilt.cos(),
        }
        .normalized();
        // Build q_target from this thrust direction + same yaw
        // (using q_target_from_acceleration would do this, but we can test directly)
        let tilt_from_down = Quaternion::from_two_unit_vectors(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            thrust_target,
        );
        let q_yaw = Quaternion {
            w: (yaw / 2.0).cos(),
            x: 0.0,
            y: 0.0,
            z: (yaw / 2.0).sin(),
        };
        let q_target = tilt_from_down * q_yaw;

        let rates = controller.update(&q_current, &q_target, AngularVelocity(0.0));

        // Should produce mostly roll, very little pitch
        assert!(
            rates.x().raw().abs() > rates.y().raw().abs() * 5.0,
            "body-right tilt at 30° yaw should produce mostly roll, got roll={:.4}, pitch={:.4}",
            rates.x().raw(),
            rates.y().raw(),
        );
    }

    #[test]
    fn yaw_feedforward_projects_to_body_frame() {
        let controller = AttitudeController::new(5.0, UnitInterval::clamp(0.4), rate_limit());

        // Pitched 30° nose down — world-Z is not aligned with body-Z
        let angle = 30.0_f64.to_radians();
        let q_current = Quaternion {
            w: (angle / 2.0).cos(),
            x: 0.0,
            y: -(angle / 2.0).sin(),
            z: 0.0,
        };
        // Target = current (no attitude error), but with yaw feedforward
        let rates = controller.update(&q_current, &q_current, AngularVelocity(1.0));

        // World-Z projected onto body frame of a pitched drone:
        // x (roll) = sin(pitch) ≈ 0.5, y (pitch) ≈ 0, z (yaw) = cos(pitch) ≈ 0.866
        assert!(
            rates.roll().raw().abs() > 0.1,
            "pitched drone should get roll rate from yaw ff"
        );
        assert!(
            rates.pitch().raw().abs() < 1e-6,
            "pure pitch shouldn't project yaw ff onto pitch"
        );
        assert!(
            rates.yaw().raw().abs() > 0.1,
            "pitched drone should get yaw rate from yaw ff"
        );
    }
}
