mod mission_controller;
mod simulator;
mod units;

use mission_controller::controller::Controller;
use mission_controller::mission_runtime::MissionRuntime;

use mission_controller::segment_def::SegmentDef;
use simulator::default_drone::DefaultDrone;
use simulator::inputs::Inputs;
use simulator::simulator::Simulator;
use simulator::types;
use units::units::Seconds;

use crate::simulator::drone::Drone;
use crate::simulator::force_model::context::Context;
use crate::simulator::force_model::force_model::ForceModel;
use crate::simulator::types::acceleration_3d::WorldFrameAcceleration;
use crate::units::acceleration::Acceleration;

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

    let controller = Controller::new();
    let mut mission = MissionRuntime::new();
    let mut current_inputs = Inputs::default();

    for _ in 0..500 {
        let inputs = if let Some(setpoint) = mission.update(sim_time, &simulator.state) {
            controller.control(setpoint, &simulator.state, current_inputs)
        } else {
            Inputs::default()
        };

        current_inputs = inputs;

        simulator.tick(Seconds(0.1), &inputs);

        sim_time = Seconds(sim_time.0 + 0.05);

        let segment = mission.current_segment().unwrap_or(&SegmentDef::Idle {
            duration: Seconds(0.0),
        });

        println!(
            "Time: {:.2}: Step: {}, Throttle: {:.2}, velocity: {:.2}, Altitude: {:.2}, geo: ({}, {})",
            sim_time.0,
            segment,
            current_inputs.throttle.get(),
            simulator.state.velocity_ned.down().0,
            simulator.state.altitude.0,
            simulator.state.latitude,
            simulator.state.longitude
        );
    }
}
