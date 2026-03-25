use primitives::{
    frames::{AccelerationNed, Ned, PositionNed, VelocityNed},
    units::{PerSecond, Seconds, SecondsLiteral},
};

use crate::controller::{
    FlightTarget,
    trajectory_generator::{
        axis_trajectory::AxisTrajectory,
        s_curve_profile::{SCurveProfile, SCurveSetpoint},
        trajectory_limits::TrajectoryLimits,
    },
};

pub struct TrajectoryGenerator {
    pub north: AxisTrajectory,
    pub east: AxisTrajectory,
    pub down: AxisTrajectory,
}

pub struct TrajectorySetpoint {
    pub north: SCurveSetpoint,
    pub east: SCurveSetpoint,
    pub down: SCurveSetpoint,
}

impl TrajectorySetpoint {
    pub fn position(&self) -> PositionNed {
        PositionNed::new(self.north.position, self.east.position, self.down.position)
    }

    pub fn velocity(&self) -> VelocityNed {
        VelocityNed::new(self.north.velocity, self.east.velocity, self.down.velocity)
    }

    pub fn acceleration(&self) -> AccelerationNed {
        AccelerationNed::new(
            self.north.acceleration,
            self.east.acceleration,
            self.down.acceleration,
        )
    }
}

impl TrajectoryGenerator {
    pub fn new(
        horizontal_limits_auto: TrajectoryLimits,
        vertical_limits_auto: TrajectoryLimits,
        horizontal_limits_manual: TrajectoryLimits,
        vertical_limits_manual: TrajectoryLimits,
        position_gain: PerSecond,
    ) -> Self {
        Self {
            north: AxisTrajectory::new(
                SCurveProfile::new(),
                position_gain,
                horizontal_limits_auto,
                horizontal_limits_manual,
            )
            .with_delay(0.2.seconds()),
            east: AxisTrajectory::new(
                SCurveProfile::new(),
                position_gain,
                horizontal_limits_auto,
                horizontal_limits_manual,
            )
            .with_delay(0.1.seconds()),
            down: AxisTrajectory::new(
                SCurveProfile::new(),
                position_gain,
                vertical_limits_auto,
                vertical_limits_manual,
            ),
        }
    }

    pub fn update(&mut self, target: &FlightTarget, dt: Seconds) -> TrajectorySetpoint {
        TrajectorySetpoint {
            north: self.north.update(&target.north, dt),
            east: self.east.update(&target.east, dt),
            down: self.down.update(&target.down, dt),
        }
    }
}

#[cfg(test)]
mod trajectory_generator_tests {
    use super::*;
    use crate::controller::AxisTarget;
    use primitives::prelude::*;

    #[test]
    fn generator_dispatches_to_all_axes() {
        let limits = TrajectoryLimits {
            max_velocity: 10.mps(),
            max_acceleration: 3.mps2(),
            max_jerk: 5.mps3(),
        };
        let mut generator =
            TrajectoryGenerator::new(limits, limits, limits, limits, PerSecond(0.5));

        let target = FlightTarget {
            north: AxisTarget::Velocity(3.mps()),
            east: AxisTarget::Velocity((-2.0).mps()),
            down: AxisTarget::Position((-10.0).meters()),
        };

        let dt = 0.01.seconds();
        let mut setpoint = generator.update(&target, dt);
        for _ in 0..1000 {
            setpoint = generator.update(&target, dt);
        }

        assert!((setpoint.north.velocity.raw() - 3.0).abs() < 0.1);
        assert!((setpoint.east.velocity.raw() - (-2.0)).abs() < 0.1);
        assert!((setpoint.down.position.raw() - (-10.0)).abs() < 0.2);
    }

    #[test]
    fn generator_axes_are_independent() {
        let horizontal_limits = TrajectoryLimits {
            max_velocity: 10.mps(),
            max_acceleration: 3.mps2(),
            max_jerk: 5.mps3(),
        };
        let vertical_limits = TrajectoryLimits {
            max_velocity: 5.mps(),
            max_acceleration: 3.mps2(),
            max_jerk: 5.mps3(),
        };
        let mut generator = TrajectoryGenerator::new(
            horizontal_limits,
            vertical_limits,
            horizontal_limits,
            vertical_limits,
            PerSecond(0.5),
        );

        // Give north and east initial velocity
        generator.north.reset(0.meters(), 4.mps(), 0.mps2());
        generator.east.reset(0.meters(), 3.mps(), 0.mps2());

        let target = FlightTarget {
            north: AxisTarget::Velocity(4.mps()),
            east: AxisTarget::Loiter,
            down: AxisTarget::Position((-5.0).meters()),
        };

        let dt = 0.01.seconds();
        for _ in 0..2000 {
            generator.update(&target, dt);
        }

        // North holds velocity
        assert!((generator.north.velocity().raw() - 4.0).abs() < 0.05);
        // East stopped at loiter target
        assert!(generator.east.velocity().raw().abs() < 0.05);
        // Down reached position
        assert!((generator.down.position().raw() - (-5.0)).abs() < 0.2);
    }
}
