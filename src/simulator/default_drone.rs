use crate::{
    simulator::{
        drone::{DragCoefficient, Drone},
        types::{drag::Drag, throttle::Throttle},
    },
    units::{
        acceleration::{Acceleration, AccelerationLiteral},
        angles::{Degrees, DegreesLiteral, DegreesPerSecond},
        consts::G_EARTH,
        units::Kilograms,
    },
};

pub struct DefaultDrone;

impl Drone for DefaultDrone {
    fn max_acceleration(&self) -> Acceleration {
        G_EARTH + 6.mps2()
    }

    fn max_roll(&self) -> Degrees {
        25.degrees()
    }

    fn max_pitch(&self) -> Degrees {
        45.degrees()
    }

    fn mass(&self) -> Kilograms {
        Kilograms(0.2)
    }

    fn max_heading_rate(&self) -> DegreesPerSecond {
        DegreesPerSecond(45.0)
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
}
