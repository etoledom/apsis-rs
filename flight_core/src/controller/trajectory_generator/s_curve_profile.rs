use crate::{
    Jerk, JerkLiteral,
    units::{
        Meters, Seconds, SecondsLiteral, Velocity, VelocityLiteral,
        acceleration::{Acceleration, AccelerationLiteral},
        traits::RawRepresentable,
    },
};

pub struct SCurveSetpoint {
    pub position: Meters,
    pub velocity: Velocity,
    pub acceleration: Acceleration,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SCurveProfile {
    // kinematic state
    position: Meters,
    velocity: Velocity,
    acceleration: Acceleration,

    // durations
    max_jerk: Jerk,
    max_acceleration: Acceleration,
    max_velocity: Velocity,
}

impl SCurveProfile {
    pub fn new(max_jerk: Jerk, max_acceleration: Acceleration, max_velocity: Velocity) -> Self {
        Self {
            max_jerk,
            max_acceleration,
            max_velocity,
            ..Default::default()
        }
    }

    /// Compute the distance needed to decelerate to zero velocity
    /// from current state using current limits.
    pub fn stopping_distance(&self) -> Meters {
        self.stopping_position() - self.position
    }

    /// Compute the position at which the profile will reach zero velocity.
    pub fn stopping_position(&self) -> Meters {
        let (t1, t2, t3, direction) = self.compute_durations(0.mps());
        let jerk_t1 = direction * self.max_jerk;

        // Walk through all three phases analytically
        let (a1, v1, x1) =
            Self::evaluate_poly(jerk_t1, self.acceleration, self.velocity, self.position, t1);
        let (a2, v2, x2) = Self::evaluate_poly(0.mps3(), a1, v1, x1, t2);
        let (_a3, _v3, x3) = Self::evaluate_poly(-jerk_t1, a2, v2, x2, t3);
        x3
    }

    /// Advance the profile toward target_velocity by dt.
    ///
    /// Recomputes T1/T2/T3 from current state each tick, then evaluates
    /// the polynomial through each phase, handling phase transitions
    /// within a single dt.
    pub fn update(&mut self, target_velocity: Velocity, dt: Seconds) -> SCurveSetpoint {
        let target_velocity = target_velocity.clamping(-self.max_velocity, self.max_velocity);
        let (t1, t2, t3, direction) = self.compute_durations(target_velocity);
        let jerk_t1 = direction * self.max_jerk;

        let mut t_remain = dt;

        // Phase T1: acceleration ramps up
        let t1_actual = t_remain.min(t1);
        let (a, v, x) = Self::evaluate_poly(
            jerk_t1,
            self.acceleration,
            self.velocity,
            self.position,
            t1_actual,
        );
        self.acceleration = a;
        self.velocity = v;
        self.position = x;
        t_remain = t_remain - t1_actual;

        // Phase T2: constant acceleration (jerk = 0)
        if t_remain > 0.seconds() {
            let t2_actual = t_remain.min(t2);
            let (a, v, x) = Self::evaluate_poly(
                0.mps3(),
                self.acceleration,
                self.velocity,
                self.position,
                t2_actual,
            );
            self.acceleration = a;
            self.velocity = v;
            self.position = x;
            t_remain = t_remain - t2_actual;
        }

        // Phase T3: acceleration ramps back to zero (opposite jerk)
        if t_remain > 0.seconds() {
            let t3_actual = t_remain.min(t3);
            let (a, v, x) = Self::evaluate_poly(
                -jerk_t1,
                self.acceleration,
                self.velocity,
                self.position,
                t3_actual,
            );
            self.acceleration = a;
            self.velocity = v;
            self.position = x;
            t_remain = t_remain - t3_actual;
        }

        // Past T3: coast at constant velocity (no jerk, no acceleration)
        if t_remain > 0.seconds() {
            let (a, v, x) =
                Self::evaluate_poly(0.mps3(), 0.mps2(), self.velocity, self.position, t_remain);
            self.acceleration = a;
            self.velocity = v;
            self.position = x;
        }

        SCurveSetpoint {
            position: self.position,
            velocity: self.velocity,
            acceleration: self.acceleration,
        }
    }

    /// Compute T1, T2, T3 phase durations to reach target_velocity
    /// from current state, minimizing total time.
    ///
    /// Returns (t1, t2, t3, direction) where direction is the jerk sign.
    fn compute_durations(&self, target_velocity: Velocity) -> (Seconds, Seconds, Seconds, f64) {
        let direction = self.compute_direction(target_velocity);

        if direction == 0.0 {
            return (0.seconds(), 0.seconds(), 0.seconds(), 0.0);
        }

        let jerk = direction * self.max_jerk;
        let delta_v = target_velocity - self.velocity;

        let t1 = Self::compute_t1(self.acceleration, delta_v, jerk, self.max_acceleration);
        let t3 = Self::compute_t3(t1, self.acceleration, jerk);
        let t2 = Self::compute_t2(t1, t3, self.acceleration, delta_v, jerk);

        (t1, t2, t3, direction)
    }

    /// Determine the jerk direction needed to reach the target velocity.
    ///
    /// Computes the velocity the profile would reach if current acceleration
    /// were cancelled immediately. If target is above that velocity,
    /// direction is +1 (accelerate). If below, -1 (decelerate).
    /// If equal, direction follows current acceleration sign (brake to zero).
    /// Returns 0 if already at target with zero acceleration.
    fn compute_direction(&self, target_velocity: Velocity) -> f64 {
        let vel_at_zero_acc = self.compute_vel_at_zero_acc();
        let direction = (target_velocity - vel_at_zero_acc).raw().signum();
        if direction == 0.0 {
            self.acceleration.raw().signum()
        } else {
            direction
        }
    }

    /// Compute the duration of the constant acceleration phase (T2).
    ///
    /// T2 absorbs the remaining velocity change that T1 and T3 don't cover.
    /// Acceleration during T2 is constant at a0 + jerk * T1.
    /// If T1 and T3 alone overshoot delta_v, T2 is zero.
    fn compute_t2(
        t1: Seconds,
        t3: Seconds,
        a0: Acceleration,
        delta_v: Velocity,
        jerk: Jerk,
    ) -> Seconds {
        let acc_cruise = a0 + jerk * t1;
        if acc_cruise.raw().abs() < f64::EPSILON {
            return 0.seconds();
        }
        let vel_t1 = a0 * t1 + 0.5 * jerk * t1.squared();
        let vel_t3 = acc_cruise * t3 - 0.5 * jerk * t3.squared();
        let t2 = (delta_v - vel_t1 - vel_t3) / acc_cruise;
        t2.max(0.seconds())
    }

    /// Compute the duration of the decreasing acceleration phase (T3).
    ///
    /// T3 ramps acceleration back to zero. Since T3 applies opposite
    /// jerk to T1: T3 = a0/j + T1
    fn compute_t3(t1: Seconds, a0: Acceleration, jerk: Jerk) -> Seconds {
        (a0 / jerk + t1).max(0.seconds())
    }

    /// Compute the duration of the increasing acceleration phase (T1).
    ///
    /// Solves for T1 such that a jerk-limited profile starting from
    /// acceleration a0 can achieve velocity change delta_v.
    /// The solution is clamped to respect max_acceleration via saturate_t1_for_acc.
    fn compute_t1(a0: Acceleration, delta_v: Velocity, jerk: Jerk, a_max: Acceleration) -> Seconds {
        let delta = 2.0 * a0 * a0 + 4.0 * jerk * delta_v;

        if delta.raw() < 0.0 {
            return 0.seconds();
        }

        let sqrt_delta = delta.sqrt();

        let t1_plus = (-a0 + 0.5 * sqrt_delta) / jerk;
        let t1_minus = (-a0 - 0.5 * sqrt_delta) / jerk;

        let t3_plus = a0 / jerk + t1_plus;
        let t3_minus = a0 / jerk + t1_minus;

        let t1 = if t1_plus >= 0.seconds() && t3_plus >= 0.seconds() {
            t1_plus
        } else if t1_minus >= 0.seconds() && t3_minus >= 0.seconds() {
            t1_minus
        } else {
            0.seconds()
        };

        Self::saturate_t1_for_acc(a0, jerk, t1, a_max)
    }

    /// Clamp T1 so that acceleration doesn't exceed max_acceleration.
    ///
    /// Shorten T1 if needed so that acceleration at the end of T1
    /// does not exceed max_acceleration.
    fn saturate_t1_for_acc(
        a0: Acceleration,
        jerk: Jerk,
        t1: Seconds,
        a_max: Acceleration,
    ) -> Seconds {
        let accel_at_t1 = a0 + jerk * t1;

        if accel_at_t1 > a_max {
            ((a_max - a0) / jerk).max(0.seconds())
        } else if accel_at_t1 < -a_max {
            ((-a_max - a0) / jerk).max(0.seconds())
        } else {
            t1
        }
    }

    /// Compute the velocity the profile will reach if maximum jerk
    /// is applied to cancel the current acceleration.
    ///
    /// When acceleration is already ~zero, returns current velocity.
    /// Used to determine jerk direction — if vel_target is above this
    /// velocity, accelerate; if below, decelerate.
    fn compute_vel_at_zero_acc(&self) -> Velocity {
        if self.acceleration.raw().abs() < f64::EPSILON {
            return self.velocity;
        }
        let j_cancel = -self.acceleration.raw().signum() * self.max_jerk;
        let t_cancel = -self.acceleration / j_cancel;

        // v_at_zero = v0 + a0*t_cancel + 0.5*j_cancel*t_cancel²
        self.velocity + self.acceleration * t_cancel + j_cancel * t_cancel.squared() * 0.5
    }

    /// Evaluate constant-jerk polynomial over time interval t.
    ///
    /// Given initial acceleration a0, velocity v0, position x0,
    /// and constant jerk j applied for duration t:
    ///
    ///   a(t) = a0 + j·t
    ///   v(t) = v0 + a0·t + ½·j·t²
    ///   x(t) = x0 + v0·t + ½·a0·t² + ⅙·j·t³
    ///
    /// Returns (acceleration, velocity, position) at time t.
    fn evaluate_poly(
        jerk: Jerk,
        a0: Acceleration,
        v0: Velocity,
        x0: Meters,
        t: Seconds,
    ) -> (Acceleration, Velocity, Meters) {
        let t2 = t.squared();
        let t3 = t.cubed();
        (
            a0 + jerk * t,
            v0 + a0 * t + jerk * t2 * 0.5,
            x0 + v0 * t + a0 * t2 * 0.5 + jerk * t3 / 6.0,
        )
    }

    pub fn with_state(
        self,
        position: Meters,
        velocity: Velocity,
        acceleration: Acceleration,
    ) -> Self {
        Self {
            position,
            velocity,
            acceleration,
            ..self
        }
    }

    pub fn with_limits(self, max_jerk: Jerk, max_acceleration: Acceleration) -> Self {
        Self {
            max_jerk,
            max_acceleration,
            ..self
        }
    }
    pub fn position(&self) -> Meters {
        self.position
    }
    pub fn velocity(&self) -> Velocity {
        self.velocity
    }
    pub fn acceleration(&self) -> Acceleration {
        self.acceleration
    }

    pub fn max_velocity(&self) -> Velocity {
        self.max_velocity
    }

    pub fn reset(&mut self, position: Meters, velocity: Velocity, acceleration: Acceleration) {
        self.position = position;
        self.velocity = velocity;
        self.acceleration = acceleration;
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        JerkLiteral,
        units::{MetersLiteral, SecondsLiteral, traits::RawRepresentable},
    };

    use super::*;

    const EPSILON: f64 = 1e-10;

    //============= EVALUATE POLY =============//

    #[test]
    fn evaluate_poly_zero_everything() {
        let (a, v, x) =
            SCurveProfile::evaluate_poly(0.mps3(), 0.mps2(), 0.mps(), 0.meters(), 1.seconds());
        assert!(a.raw().abs() < EPSILON);
        assert!(v.raw().abs() < EPSILON);
        assert!(x.raw().abs() < EPSILON);
    }

    #[test]
    fn evaluate_poly_constant_velocity() {
        // No jerk, no acceleration, v=5 m/s for 2s → x = 10m
        let (a, v, x) =
            SCurveProfile::evaluate_poly(0.mps3(), 0.mps2(), 5.mps(), 0.meters(), 2.seconds());
        assert!(a.raw().abs() < EPSILON);
        assert!((v.raw() - 5.0).abs() < EPSILON);
        assert!((x.raw() - 10.0).abs() < EPSILON);
    }

    #[test]
    fn evaluate_poly_constant_acceleration() {
        // No jerk, a=3 m/s², from rest, 2s
        // v = 0 + 3*2 = 6
        // x = 0 + 0 + 0.5*3*4 = 6
        let (a, v, x) =
            SCurveProfile::evaluate_poly(0.mps3(), 3.mps2(), 0.mps(), 0.meters(), 2.seconds());
        assert!((a.raw() - 3.0).abs() < EPSILON);
        assert!((v.raw() - 6.0).abs() < EPSILON);
        assert!((x.raw() - 6.0).abs() < EPSILON);
    }

    #[test]
    fn evaluate_poly_constant_jerk_from_rest() {
        // j=2 m/s³, from rest, 3s
        // a = 0 + 2*3 = 6
        // v = 0 + 0 + 0.5*2*9 = 9
        // x = 0 + 0 + 0 + (1/6)*2*27 = 9
        let (a, v, x) =
            SCurveProfile::evaluate_poly(2.0.mps3(), 0.mps2(), 0.mps(), 0.meters(), 3.0.seconds());
        assert!((a.raw() - 6.0).abs() < EPSILON);
        assert!((v.raw() - 9.0).abs() < EPSILON);
        assert!((x.raw() - 9.0).abs() < EPSILON);
    }

    #[test]
    fn evaluate_poly_negative_jerk_braking() {
        // j=-2, a0=6, v0=10, x0=0, t=3
        // a = 6 + (-2)*3 = 0
        // v = 10 + 6*3 + 0.5*(-2)*9 = 19
        // x = 0 + 10*3 + 0.5*6*9 + (1/6)*(-2)*27 = 48
        let (a, v, x) =
            SCurveProfile::evaluate_poly(-2.mps3(), 6.mps2(), 10.mps(), 0.meters(), 3.seconds());
        assert!(a.raw().abs() < EPSILON);
        assert!((v.raw() - 19.0).abs() < EPSILON);
        assert!((x.raw() - 48.0).abs() < EPSILON);
    }

    #[test]
    fn evaluate_poly_preserves_initial_position() {
        let (_, _, x) =
            SCurveProfile::evaluate_poly(0.mps3(), 0.mps2(), 0.mps(), 100.meters(), 5.seconds());
        assert!((x.raw() - 100.0).abs() < EPSILON);
    }

    #[test]
    fn evaluate_poly_zero_dt_returns_initial_state() {
        // Jerk is intentionally extreme — with dt=0, no input affects the output
        let (a, v, x) =
            SCurveProfile::evaluate_poly(999.mps3(), 1.mps2(), 2.mps(), 3.meters(), 0.seconds());
        assert!((a.raw() - 1.0).abs() < EPSILON);
        assert!((v.raw() - 2.0).abs() < EPSILON);
        assert!((x.raw() - 3.0).abs() < EPSILON);
    }

    //============= COMPUTE VEL AT ZERO ACC =============//

    #[test]
    fn vel_at_zero_acc_when_already_zero() {
        // No acceleration — returns current velocity unchanged
        let profile = SCurveProfile::new(10.mps3(), 5.mps2(), 10.mps()).with_state(
            0.meters(),
            3.mps(),
            0.mps2(),
        );
        assert!((profile.compute_vel_at_zero_acc().raw() - 3.0).abs() < EPSILON);
    }

    #[test]
    fn vel_at_zero_acc_positive_acceleration() {
        // a0=4 m/s², v0=2 m/s, max_jerk=8 m/s³
        // j_cancel = -1 * 8 = -8 m/s³
        // t_cancel = -4 / -8 = 0.5s
        // v = 2 + 4*0.5 + 0.5*(-8)*0.25 = 2 + 2 - 1 = 3 m/s
        let profile = SCurveProfile::new(8.mps3(), 5.mps2(), 10.mps()).with_state(
            0.meters(),
            2.mps(),
            4.mps2(),
        );
        assert!((profile.compute_vel_at_zero_acc().raw() - 3.0).abs() < EPSILON);
    }

    #[test]
    fn vel_at_zero_acc_negative_acceleration() {
        // a0=-6 m/s², v0=10 m/s, max_jerk=12 m/s³
        // j_cancel = -(-1) * 12 = +12 m/s³
        // t_cancel = -(-6) / 12 = 0.5s
        // v = 10 + (-6)*0.5 + 0.5*12*0.25 = 10 - 3 + 1.5 = 8.5 m/s
        let profile = SCurveProfile::new(12.mps3(), 5.mps2(), 10.mps()).with_state(
            0.meters(),
            10.mps(),
            (-6.0).mps2(),
        );
        assert!((profile.compute_vel_at_zero_acc().raw() - 8.5).abs() < EPSILON);
    }

    //============= SATURATE T1 FOR ACCEL =============//

    #[test]
    fn saturate_t1_no_saturation_needed() {
        // a0=0, j=5, t1=1.0 → a(T1)=5, a_max=10 → no clamp
        let t1 = SCurveProfile::saturate_t1_for_acc(0.mps2(), 5.mps3(), 1.0.seconds(), 10.mps2());
        assert!((t1.raw() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn saturate_t1_positive_overshoot() {
        // a0=2, j=4, t1=3.0 → a(T1)=14, a_max=10
        // clamped: t1 = (10 - 2) / 4 = 2.0
        let t1 = SCurveProfile::saturate_t1_for_acc(2.mps2(), 4.mps3(), 3.0.seconds(), 10.mps2());
        assert!((t1.raw() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn saturate_t1_negative_overshoot() {
        // a0=-1, j=-6, t1=2.0 → a(T1)=-13, a_max=10
        // clamped: t1 = (-10 - (-1)) / -6 = -9 / -6 = 1.5
        let t1 = SCurveProfile::saturate_t1_for_acc(
            (-1.0).mps2(),
            (-6.0).mps3(),
            2.0.seconds(),
            10.mps2(),
        );
        assert!((t1.raw() - 1.5).abs() < EPSILON);
    }

    #[test]
    fn saturate_t1_exactly_at_limit() {
        // a0=0, j=5, t1=2.0 → a(T1)=10, a_max=10 → exactly at limit, no clamp
        let t1 = SCurveProfile::saturate_t1_for_acc(0.mps2(), 5.mps3(), 2.0.seconds(), 10.mps2());
        assert!((t1.raw() - 2.0).abs() < EPSILON);
    }

    //============= COMPUTE T1 =============//

    #[test]
    fn compute_t1_from_rest_symmetric() {
        // a0=0, delta_v=8, j=2, a_max=10
        // delta = 0 + 4*2*8 = 64, sqrt=8
        // t1_plus = (0 + 4) / 2 = 2.0
        // t3_plus = 0 + 2.0 = 2.0
        // Both positive → t1 = 2.0
        // accel at T1 = 0 + 2*2 = 4 < 10 → no saturation
        let t1 = SCurveProfile::compute_t1(0.mps2(), 8.mps(), 2.mps3(), 10.mps2());
        assert!((t1.raw() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn compute_t1_saturated_by_max_acc() {
        // a0=0, delta_v=50, j=2, a_max=4
        // delta = 0 + 4*2*50 = 400, sqrt=20
        // t1_plus = (0 + 10) / 2 = 5.0
        // accel at T1 = 0 + 2*5 = 10 > 4 → saturated
        // saturated t1 = (4 - 0) / 2 = 2.0
        let t1 = SCurveProfile::compute_t1(0.mps2(), 50.mps(), 2.mps3(), 4.mps2());
        assert!((t1.raw() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn compute_t1_no_real_solution() {
        // a0=0, delta_v=-5, j=2 → delta = 0 + 4*2*(-5) = -40 < 0
        // No real solution → returns 0
        let t1 = SCurveProfile::compute_t1(0.mps2(), (-5.0).mps(), 2.mps3(), 10.mps2());
        assert!(t1.raw().abs() < EPSILON);
    }

    #[test]
    fn compute_t1_with_initial_acceleration() {
        // a0=3, delta_v=10, j=2, a_max=20
        // delta = 2*9 + 4*2*10 = 18 + 80 = 98, sqrt=9.8995
        // t1_plus = (-3 + 4.9497) / 2 = 0.9749
        // t3_plus = 3/2 + 0.9749 = 2.4749
        // Both positive → t1 ≈ 0.9749
        let t1 = SCurveProfile::compute_t1(3.mps2(), 10.mps(), 2.mps3(), 20.mps2());
        assert!((t1.raw() - 0.97487).abs() < 1e-4);
    }

    //============= COMPUTE T3 =============//

    #[test]
    fn compute_t3_from_rest() {
        // a0=0, j=5, t1=2.0 → T3 = 0/5 + 2 = 2.0
        // Symmetric: T1 ramps up, T3 ramps down by same amount
        let t3 = SCurveProfile::compute_t3(2.0.seconds(), 0.mps2(), 5.mps3());
        assert!((t3.raw() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn compute_t3_with_initial_acceleration() {
        // a0=3, j=2, t1=1.0 → T3 = 3/2 + 1 = 2.5
        // T3 > T1 because it also has to cancel the initial acceleration
        let t3 = SCurveProfile::compute_t3(1.0.seconds(), 3.mps2(), 2.mps3());
        assert!((t3.raw() - 2.5).abs() < EPSILON);
    }

    #[test]
    fn compute_t3_clamped_to_zero() {
        // a0=-10, j=2, t1=1.0 → T3 = -10/2 + 1 = -4 → clamped to 0
        let t3 = SCurveProfile::compute_t3(1.0.seconds(), (-10.0).mps2(), 2.mps3());
        assert!(t3.raw().abs() < EPSILON);
    }

    //============= COMPUTE T2 =============//

    #[test]
    fn compute_t2_no_constant_phase_needed() {
        // T1 and T3 exactly consume delta_v → T2 = 0
        // a0=0, jerk=2, t1=2, t3=2, delta_v = vel_t1 + vel_t3
        // vel_t1 = 0*2 + 0.5*2*4 = 4
        // acc_cruise = 0 + 2*2 = 4
        // vel_t3 = 4*2 - 0.5*2*4 = 8 - 4 = 4
        // delta_v = 4 + 4 = 8
        let t2 = SCurveProfile::compute_t2(2.seconds(), 2.seconds(), 0.mps2(), 8.mps(), 2.mps3());
        assert!(t2.raw().abs() < EPSILON);
    }

    #[test]
    fn compute_t2_with_constant_phase() {
        // a0=0, jerk=2, t1=1, t3=1, delta_v=10
        // vel_t1 = 0 + 0.5*2*1 = 1
        // acc_cruise = 0 + 2*1 = 2
        // vel_t3 = 2*1 - 0.5*2*1 = 2 - 1 = 1
        // remaining = 10 - 1 - 1 = 8
        // t2 = 8 / 2 = 4.0
        let t2 = SCurveProfile::compute_t2(1.seconds(), 1.seconds(), 0.mps2(), 10.mps(), 2.mps3());
        assert!((t2.raw() - 4.0).abs() < EPSILON);
    }

    #[test]
    fn compute_t2_with_initial_acceleration() {
        // a0=3, jerk=2, t1=1, t3=2.5, delta_v=20
        // vel_t1 = 3*1 + 0.5*2*1 = 3 + 1 = 4
        // acc_cruise = 3 + 2*1 = 5
        // vel_t3 = 5*2.5 - 0.5*2*6.25 = 12.5 - 6.25 = 6.25
        // remaining = 20 - 4 - 6.25 = 9.75
        // t2 = 9.75 / 5 = 1.95
        let t2 =
            SCurveProfile::compute_t2(1.seconds(), 2.5.seconds(), 3.mps2(), 20.mps(), 2.mps3());
        assert!((t2.raw() - 1.95).abs() < EPSILON);
    }

    #[test]
    fn compute_t2_zero_acc_cruise() {
        // acc_cruise = a0 + jerk*t1 ≈ 0 → returns 0 (avoid division by zero)
        let t2 =
            SCurveProfile::compute_t2(1.seconds(), 1.seconds(), (-2.0).mps2(), 5.mps(), 2.mps3());
        assert!(t2.raw().abs() < EPSILON);
    }

    //============= COMPUTE DIRECTION =============//

    #[test]
    fn compute_direction_target_above() {
        // v=2, a=0, target=5 → vel_at_zero_acc=2, target above → +1
        let profile = SCurveProfile::new(10.mps3(), 5.mps2(), 10.mps()).with_state(
            0.meters(),
            2.mps(),
            0.mps2(),
        );
        assert!((profile.compute_direction(5.mps()) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn compute_direction_target_below() {
        // v=8, a=0, target=3 → vel_at_zero_acc=8, target below → -1
        let profile = SCurveProfile::new(10.mps3(), 5.mps2(), 10.mps()).with_state(
            0.meters(),
            8.mps(),
            0.mps2(),
        );
        assert!((profile.compute_direction(3.mps()) - (-1.0)).abs() < EPSILON);
    }

    #[test]
    fn compute_direction_overshooting_needs_reversal() {
        // v=5, a=6, max_jerk=12 → vel_at_zero_acc = 5 + 6*0.5 + 0.5*(-12)*0.25 = 6.5
        // target=4 → 4 < 6.5 → direction = -1
        // Even though v=5 > target=4, acceleration is carrying us further away
        let profile = SCurveProfile::new(12.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            5.mps(),
            6.mps2(),
        );
        assert!((profile.compute_direction(4.mps()) - (-1.0)).abs() < EPSILON);
    }

    #[test]
    fn compute_direction_exact_match_brakes() {
        // vel_at_zero_acc exactly equals target → direction = sign(a0)
        // a0=4, max_jerk=8 → vel_at_zero_acc = 2 + 4*0.5 + 0.5*(-8)*0.25 = 3.0
        // target=3 → exact match → direction = sign(4) = +1 (brake positive acc)
        let profile = SCurveProfile::new(8.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            2.mps(),
            4.mps2(),
        );
        assert!((profile.compute_direction(3.mps()) - 1.0).abs() < EPSILON);
    }

    //============= COMPUTE DURATIONS =============//

    #[test]
    fn compute_durations_from_rest_to_target() {
        // v=0, a=0, target=8, jerk=2, a_max=10
        // direction = +1, j=2, delta_v=8
        // t1 = compute_t1(0, 8, 2, 10) = 2.0 (from earlier test)
        // t3 = 0/2 + 2 = 2.0
        // vel_t1 = 0 + 0.5*2*4 = 4, vel_t3 = 4*2 - 0.5*2*4 = 4
        // remaining = 8 - 4 - 4 = 0 → t2 = 0
        let profile = SCurveProfile::new(2.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        let (t1, t2, t3, dir) = profile.compute_durations(8.mps());
        assert!((dir - 1.0).abs() < EPSILON);
        assert!((t1.raw() - 2.0).abs() < EPSILON);
        assert!(t2.raw().abs() < EPSILON);
        assert!((t3.raw() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn compute_durations_needs_cruise_phase() {
        // v=0, a=0, target=10, jerk=2, a_max=4
        // direction = +1, j=2, delta_v=10
        // t1 = compute_t1(0, 10, 2, 4) → unsaturated t1=sqrt(10/2)=2.236
        //   but accel at t1 = 2*2.236 = 4.47 > 4 → saturated to (4-0)/2 = 2.0
        // t3 = 0/2 + 2 = 2.0
        // vel_t1 = 0 + 0.5*2*4 = 4, a_plateau = 4
        // vel_t3 = 4*2 - 0.5*2*4 = 4
        // remaining = 10 - 4 - 4 = 2, t2 = 2/4 = 0.5
        let profile = SCurveProfile::new(2.mps3(), 4.mps2(), 10.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        let (t1, t2, t3, dir) = profile.compute_durations(10.mps());
        assert!((dir - 1.0).abs() < EPSILON);
        assert!((t1.raw() - 2.0).abs() < EPSILON);
        assert!((t2.raw() - 0.5).abs() < EPSILON);
        assert!((t3.raw() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn compute_durations_deceleration() {
        // v=8, a=0, target=0, jerk=2, a_max=10
        // direction = -1, j=-2, delta_v=-8
        // t1 = compute_t1(0, -8, -2, 10)
        //   delta = 0 + 4*(-2)*(-8) = 64, sqrt=8
        //   t1_plus = (0 + 4) / -2 = -2 (negative)
        //   t1_minus = (0 - 4) / -2 = 2.0
        //   t3_minus = 0/-2 + 2 = 2.0
        //   Both positive → t1 = 2.0
        // t3 = 0/-2 + 2 = 2.0
        // t2 = 0 (symmetric)
        let profile = SCurveProfile::new(2.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            8.mps(),
            0.mps2(),
        );
        let (t1, t2, t3, dir) = profile.compute_durations(0.mps());
        assert!((dir - (-1.0)).abs() < EPSILON);
        assert!((t1.raw() - 2.0).abs() < EPSILON);
        assert!(t2.raw().abs() < EPSILON);
        assert!((t3.raw() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn compute_durations_already_at_target() {
        // v=5, a=0, target=5 → direction=0, all durations zero
        let profile = SCurveProfile::new(2.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            5.mps(),
            0.mps2(),
        );
        let (t1, t2, t3, _dir) = profile.compute_durations(5.mps());
        assert!(t1.raw().abs() < EPSILON);
        assert!(t2.raw().abs() < EPSILON);
        assert!(t3.raw().abs() < EPSILON);
    }

    //============= UPDATE =============//

    #[test]
    fn update_from_rest_single_tick() {
        // v=0, a=0, target=10, jerk=2, a_max=10, dt=0.5
        // direction=+1, j=2, delta_v=10
        // T1 large, so entire dt is in T1 phase
        // a = 0 + 2*0.5 = 1.0
        // v = 0 + 0 + 0.5*2*0.25 = 0.25
        // x = 0 + 0 + 0 + (1/6)*2*0.125 = 0.04167
        let mut profile = SCurveProfile::new(2.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        let sp = profile.update(10.mps(), 0.5.seconds());
        assert!((sp.acceleration.raw() - 1.0).abs() < 1e-9);
        assert!((sp.velocity.raw() - 0.25).abs() < 1e-9);
        assert!((sp.acceleration.raw() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn update_reaches_target_velocity() {
        // Run enough ticks to reach target, verify convergence
        let mut profile = SCurveProfile::new(5.mps3(), 3.mps2(), 10.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        let dt = 0.01.seconds();
        let mut sp = SCurveSetpoint {
            position: 0.meters(),
            velocity: 0.mps(),
            acceleration: 0.mps2(),
        };
        for _ in 0..1000 {
            sp = profile.update(4.mps(), dt);
        }
        assert!((sp.velocity.raw() - 4.0).abs() < 0.01);
        assert!(sp.acceleration.raw().abs() < 0.01);
    }

    #[test]
    fn update_deceleration_to_zero() {
        // Start at v=6, decelerate to 0
        let mut profile = SCurveProfile::new(5.mps3(), 3.mps2(), 10.mps()).with_state(
            0.meters(),
            6.mps(),
            0.mps2(),
        );
        let dt = 0.01.seconds();
        let mut sp = SCurveSetpoint {
            position: 0.meters(),
            velocity: 0.mps(),
            acceleration: 0.mps2(),
        };
        for _ in 0..1000 {
            sp = profile.update(0.mps(), dt);
        }
        assert!(sp.velocity.raw().abs() < 0.01);
        assert!(sp.acceleration.raw().abs() < 0.01);
        // Position should be positive — drone moved forward while braking
        assert!(sp.position.raw() > 0.0);
    }

    #[test]
    fn update_velocity_never_exceeds_max() {
        // max_velocity=3, target=10 → should clamp to 3
        let mut profile = SCurveProfile::new(5.mps3(), 5.mps2(), 3.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        let dt = 0.01.seconds();
        for _ in 0..1000 {
            profile.update(10.mps(), dt);
        }
        assert!(profile.velocity().raw() <= 3.01);
    }

    #[test]
    fn update_acceleration_never_exceeds_max() {
        // Track peak acceleration across all ticks
        let mut profile = SCurveProfile::new(100.mps3(), 5.mps2(), 10.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        let dt = 0.01.seconds();
        let mut max_acc = 0.0_f64;
        for _ in 0..500 {
            let sp = profile.update(10.mps(), dt);
            max_acc = max_acc.max(sp.acceleration.raw().abs());
        }
        assert!(max_acc <= 5.01);
    }

    #[test]
    fn update_state_is_preserved_between_calls() {
        // Two ticks of 0.5s should equal one tick of 1.0s... approximately
        // (not exact because durations are recomputed, but should be close)
        let mut profile_a = SCurveProfile::new(2.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        profile_a.update(10.mps(), 0.5.seconds());
        let sp_a = profile_a.update(10.mps(), 0.5.seconds());

        let mut profile_b = SCurveProfile::new(2.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        let sp_b = profile_b.update(10.mps(), 1.0.seconds());

        assert!((sp_a.velocity.raw() - sp_b.velocity.raw()).abs() < 0.01);
        assert!((sp_a.position.raw() - sp_b.position.raw()).abs() < 0.01);
    }

    //============= STOPPING DISTANCE =============//

    #[test]
    fn stopping_distance_from_rest_is_zero() {
        let profile = SCurveProfile::new(5.mps3(), 3.mps2(), 10.mps()).with_state(
            0.meters(),
            0.mps(),
            0.mps2(),
        );
        assert!(profile.stopping_distance().raw().abs() < EPSILON);
    }

    #[test]
    fn stopping_distance_positive_velocity() {
        // v=8, a=0, jerk=2, a_max=10
        // Same as deceleration test: t1=2, t2=0, t3=2
        // Walk through phases:
        // T1: j=-2, a0=0, v0=8, x0=0, t=2
        //   a = 0 + (-2)*2 = -4
        //   v = 8 + 0 + 0.5*(-2)*4 = 4
        //   x = 0 + 8*2 + 0 + (1/6)*(-2)*8 = 16 - 2.667 = 13.333
        // T3: j=+2, a0=-4, v0=4, x0=13.333, t=2
        //   a = -4 + 2*2 = 0
        //   v = 4 + (-4)*2 + 0.5*2*4 = 4 - 8 + 4 = 0
        //   x = 13.333 + 4*2 + 0.5*(-4)*4 + (1/6)*2*8 = 13.333 + 8 - 8 + 2.667 = 16
        let profile = SCurveProfile::new(2.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            8.mps(),
            0.mps2(),
        );
        assert!((profile.stopping_distance().raw() - 16.0).abs() < 1e-9);
        assert!((profile.stopping_position().raw() - 16.0).abs() < 1e-9);
    }

    #[test]
    fn stopping_position_accounts_for_initial_position() {
        // Same as above but starting at x=100
        let profile = SCurveProfile::new(2.mps3(), 10.mps2(), 10.mps()).with_state(
            100.meters(),
            8.mps(),
            0.mps2(),
        );
        assert!((profile.stopping_distance().raw() - 16.0).abs() < 1e-9);
        assert!((profile.stopping_position().raw() - 116.0).abs() < 1e-9);
    }

    #[test]
    fn stopping_distance_with_initial_acceleration() {
        // v=5, a=3, jerk=6, a_max=10
        // vel_at_zero_acc: j_cancel=-6, t_cancel=3/6=0.5
        //   v_at_zero = 5 + 3*0.5 + 0.5*(-6)*0.25 = 5 + 1.5 - 0.75 = 5.75
        // target=0 < 5.75 → direction=-1, j=-6
        // Must first cancel positive acceleration, then decelerate
        // stopping_distance should be > 0 (moving forward while braking)
        let profile = SCurveProfile::new(6.mps3(), 10.mps2(), 10.mps()).with_state(
            0.meters(),
            5.mps(),
            3.mps2(),
        );
        let dist = profile.stopping_distance();
        assert!(dist.raw() > 0.0);
    }

    #[test]
    fn stopping_distance_matches_simulation() {
        // Run update(0) until stopped, compare final position with stopping_position
        let mut profile = SCurveProfile::new(4.mps3(), 3.mps2(), 10.mps()).with_state(
            0.meters(),
            6.mps(),
            0.mps2(),
        );
        let predicted = profile.stopping_position();

        let dt = 0.01.seconds();
        for _ in 0..2000 {
            profile.update(0.mps(), dt);
        }
        assert!((profile.position().raw() - predicted.raw()).abs() < 0.05);
    }
}
