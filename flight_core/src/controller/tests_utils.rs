use crate::{
    AngularDamping, Drag, Drone, Throttle,
    drone::DragCoefficient,
    units::{angles::*, *},
};

#[derive(Clone, Copy)]
pub struct TestDrone;

impl Drone for TestDrone {
    fn max_roll(&self) -> Degrees {
        Degrees(30.0)
    }
    fn max_pitch(&self) -> Degrees {
        Degrees(45.0)
    }
    fn max_pitch_acceleration(&self) -> AngularAcceleration {
        AngularAcceleration::new(2.0)
    }
    fn max_roll_acceleration(&self) -> AngularAcceleration {
        AngularAcceleration::new(2.0)
    }
    fn max_yaw_acceleration(&self) -> AngularAcceleration {
        AngularAcceleration::new(2.0)
    }
    fn pitch_damping_coefficient(&self) -> AngularDamping {
        AngularDamping::new(2.0)
    }
    fn roll_damping_coefficient(&self) -> AngularDamping {
        AngularDamping::new(2.0)
    }
    fn yaw_damping_coefficient(&self) -> AngularDamping {
        AngularDamping::new(2.0)
    }
    fn thrust_to_waight_ratio(&self) -> f64 {
        2.0
    }
    fn drag_coefficient(&self) -> DragCoefficient {
        DragCoefficient {
            vertical: Drag::new(1.0),
            horizontal: Drag::new(1.0),
            forward: Drag::new(1.0),
        }
    }
    fn battery_drain_pct(&self, _throttle: Throttle) -> f64 {
        0.1
    }
    fn motor_time_constant(&self) -> Seconds {
        Seconds(0.2)
    }
}
