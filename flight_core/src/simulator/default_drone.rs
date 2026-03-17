use crate::{
    simulator::{
        drone::{DragCoefficient, Drone},
        types::{angular_damping::AngularDamping, drag::Drag, throttle::Throttle},
    },
    units::{
        SecondsLiteral,
        angles::{AngularAcceleration, Degrees, DegreesLiteral},
    },
};

#[derive(Clone, Copy)]
pub struct DefaultDrone;

impl Drone for DefaultDrone {
    fn max_roll(&self) -> Degrees {
        25.degrees()
    }

    fn max_pitch(&self) -> Degrees {
        30.degrees()
    }

    fn max_pitch_acceleration(&self) -> AngularAcceleration {
        AngularAcceleration::new(4.0)
    }

    fn max_roll_acceleration(&self) -> AngularAcceleration {
        AngularAcceleration::new(4.0)
    }

    fn max_yaw_acceleration(&self) -> AngularAcceleration {
        AngularAcceleration::new(4.0)
    }

    fn thrust_to_waight_ratio(&self) -> f64 {
        2.0
    }

    fn motor_time_constant(&self) -> crate::units::Seconds {
        0.1.seconds()
    }

    fn battery_drain_pct(&self, throttle: Throttle) -> f64 {
        let idle_battery_drain_pct_per_second = 0.05;
        return idle_battery_drain_pct_per_second + (throttle.get() * 0.15);
    }

    fn drag_coefficient(&self) -> super::drone::DragCoefficient {
        DragCoefficient {
            vertical: Drag::new(0.6),
            horizontal: Drag::new(0.4),
            forward: Drag::new(0.2),
        }
    }

    fn pitch_damping_coefficient(&self) -> AngularDamping {
        AngularDamping::new(2.5)
    }

    fn roll_damping_coefficient(&self) -> AngularDamping {
        AngularDamping::new(2.5)
    }

    fn yaw_damping_coefficient(&self) -> AngularDamping {
        AngularDamping::new(1.0)
    }
}
