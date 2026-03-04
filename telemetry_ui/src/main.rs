mod pilot_control;
mod theme;
mod widgets;
use std::time::{Duration, Instant};

use eframe::egui;
use flight_core::{
    controller::FlightController,
    units::{MettersLiteral, SecondsLiteral, VelocityLiteral},
    *,
};
mod sim_ui;
use sim_ui::*;

use crate::pilot_control::controller::Target;

fn main() -> eframe::Result {
    let (state_tx, state_rx) = std::sync::mpsc::channel();
    let (target_tx, target_rx) = std::sync::mpsc::channel();

    std::thread::spawn(move || {
        let mut simulator = Simulator::new(DefaultDrone {});
        let mut controller =
            FlightController::new(10.meters(), VelocityNED::new(0.mps(), 0.mps(), 0.mps()));
        let mut last_time = Instant::now();
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

                controller.set_target_velocity(world_frame_target);
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
