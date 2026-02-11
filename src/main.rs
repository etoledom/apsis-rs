mod simulator;

mod mission_controller;

mod units;
use std::thread::Thread;

use simulator::types;

use crate::{
    mission_controller::{
        controller::Controller, mission_def::SegmentDef, mission_runtime::MissionRuntime,
    },
    simulator::{default_drone::DefaultDrone, inputs::Inputs, simulator::Simulator},
    units::units::Seconds,
};

fn main() {
    let mut sim_time = Seconds::zero();

    let mut simulator = Simulator::new(DefaultDrone {});

    let controller = Controller::new();
    let mut mission = MissionRuntime::new();
    let mut current_inputs = Inputs::default();

    for _ in 0..700 {
        let inputs = if let Some(setpoint) = mission.update(sim_time, &simulator.state) {
            controller.control(setpoint, &simulator.state, current_inputs)
        } else {
            Inputs::default()
        };

        current_inputs = inputs;

        simulator.tick(Seconds(0.05), &inputs);

        sim_time = Seconds(sim_time.0 + 0.05);

        let segment = mission.current_segment().unwrap_or(&SegmentDef::Idle {
            duration: Seconds(0.0),
        });

        println!(
            "Time: {:.2}: Step: {}, Throttle: {:.2}, velocity: {:.2}, Altitude: {:.2}",
            sim_time.0,
            segment,
            current_inputs.throttle.get(),
            simulator.state.vertical_velocity.0,
            simulator.state.altitude.0
        );
    }
}
