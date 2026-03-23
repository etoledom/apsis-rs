mod pilot_control;
mod theme;
mod widgets;
use std::time::{Duration, Instant};

use eframe::egui;
use flight_core::{
    controller::{AxisTarget, FlightController},
    *,
};
mod sim_ui;
use primitives::{
    frames::{Ned, VelocityFrd},
    units::{MetersLiteral, SecondsLiteral, VelocityLiteral},
};
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

        controller.set_target_north(AxisTarget::Position(20.meters()));
        controller.set_target_down(AxisTarget::Position(-10.meters()));

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
                let vel_target_frd = VelocityFrd::new(target.forward, target.right, 0.mps());
                let vel_target_ned = vel_target_frd.to_world_frame(&simulator.state.attitude);

                // If there's no command from the pilot, use Lotier mode to hold position.
                if target.up == 0.mps() {
                    controller.set_target_down(AxisTarget::Loiter);
                } else if target.up != 0.mps() {
                    controller.set_target_down(AxisTarget::Velocity(-target.up));
                }

                if target.forward != 0.mps() || target.right != 0.mps() {
                    controller.set_target_north(AxisTarget::Velocity(vel_target_ned.north()));
                    controller.set_target_east(AxisTarget::Velocity(vel_target_ned.east()));
                } else {
                    controller.set_target_north(AxisTarget::Loiter);
                    controller.set_target_east(AxisTarget::Loiter);
                }

                controller.set_yaw_rate_target(target.yaw_rate);
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
