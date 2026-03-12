mod pilot_control;
mod theme;
mod widgets;
use std::time::{Duration, Instant};

use eframe::egui;
use flight_core::{
    controller::{AxisTarget, FlightController},
    units::{SecondsLiteral, acceleration::AccelerationLiteral},
    *,
};
mod sim_ui;
use sim_ui::*;

use crate::pilot_control::controller::Target;

fn main() -> eframe::Result {
    let (state_tx, state_rx) = std::sync::mpsc::channel();
    let (target_tx, target_rx) = std::sync::mpsc::channel();

    std::thread::spawn(move || {
        let drone = DefaultDrone {};
        let mut simulator = Simulator::new(drone);
        let mut controller = FlightController::for_drone(drone);
        let mut last_time = Instant::now();
        let mut previous_pilot_targets: Target = Default::default();
        loop {
            let now = Instant::now();
            let dt = now.duration_since(last_time).as_secs_f64().seconds();
            last_time = now;

            let inputs = controller.update(&simulator.state, dt);
            simulator.tick(&inputs, dt);
            state_tx.send(simulator.state.clone()).ok();

            // Pilot input
            while let Ok(last_target) = target_rx.try_recv() {
                let target: Target = last_target;

                let world_frame_target = simulator
                    .state
                    .attitude
                    .rotate_body_to_world(target.forward, target.right);

                if target.up.0 == 0.0 && previous_pilot_targets.up.0 != 0.0 {
                    controller
                        .set_target_down(AxisTarget::Position(simulator.state.position_ned.down()));
                } else if target.up.0 != 0.0 {
                    controller.set_target_down(AxisTarget::Velocity(-target.up));
                }

                if target.forward.0 != 0.0 || target.right.0 != 0.0 {
                    controller.set_target_north(AxisTarget::Velocity(world_frame_target.north()));
                    controller.set_target_east(AxisTarget::Velocity(world_frame_target.east()));
                } else {
                    let max_deleceleration = 3.mps2();
                    let north_vel = simulator.state.velocity_ned.north();
                    let north_breaking = (north_vel * north_vel.abs()) / (max_deleceleration * 2.0);
                    controller.set_target_north(AxisTarget::Position(
                        simulator.state.position_ned.north() + north_breaking * 2.0,
                    ));
                    controller.set_target_east(AxisTarget::Position(
                        simulator.state.position_ned.east()
                            + simulator.state.velocity_ned.east() * 1.seconds(),
                    ));
                }

                // controller.set_target_velocity(world_frame_target);
                controller.set_yaw_rate_target(target.yaw_rate);

                previous_pilot_targets = target;
            }

            std::thread::sleep(Duration::from_millis(10));
        }
    });

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([900.0, 600.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Simulator",
        options,
        Box::new(|_| Ok(Box::new(SimulatorUI::new(state_rx, target_tx)))),
    )
}
