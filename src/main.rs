mod controller;
mod mission_controller;
mod simulator;
mod units;

use mission_controller::segment_def::SegmentDef;
use simulator::default_drone::DefaultDrone;
use simulator::inputs::Inputs;
use simulator::simulator::Simulator;
use simulator::types;
use units::units::Seconds;

use crate::controller::controller::Controller;
use crate::simulator::drone::Drone;
use crate::simulator::force_model::context::Context;
use crate::simulator::force_model::force_model::ForceModel;
use crate::simulator::types::acceleration_3d::WorldFrameAcceleration;
use crate::units::acceleration::Acceleration;
use crate::units::units::{Meters, SecondsLiteral};

struct Wind;

impl<Vehicle: Drone> ForceModel<Vehicle> for Wind {
    fn acceleration_contribution<'a>(
        &self,
        _: &Context<Vehicle>,
        _: Seconds,
    ) -> WorldFrameAcceleration {
        WorldFrameAcceleration::new(Acceleration(0.5), Acceleration(0.5), Acceleration(0.0))
    }
}

fn main() {
    let mut sim_time = Seconds::zero();

    let wind = Wind {};

    let mut simulator = Simulator::new(DefaultDrone {});
    // simulator.add_force(wind);

    // let controller = Controller::new();
    // let mut mission = MissionRuntime::new();
    let mut flight_controller = Controller::new(Meters(10.0));

    let delta_t = 0.1.seconds();

    for _ in 0..500 {
        let inputs = flight_controller.update(&simulator.state, delta_t);

        simulator.tick(delta_t, &inputs);

        sim_time = sim_time + delta_t;

        println!(
            "Time: {:.2}, Throttle: {:.2}, velocity: {:.2}, Altitude: {:.2}, geo: ({}, {})",
            sim_time.0,
            inputs.throttle.get(),
            simulator.state.velocity_ned.down().0,
            simulator.state.altitude.0,
            simulator.state.latitude,
            simulator.state.longitude
        );
    }
}
