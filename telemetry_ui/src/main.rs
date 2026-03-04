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

            println!(
                "wx: {:.3}, wy: {:.3}, wz: {:.3}",
                simulator.state.angular_velocity_body.x(),
                simulator.state.angular_velocity_body.y(),
                simulator.state.angular_velocity_body.z()
            );

            // Pilot input
            while let Ok(last_target) = target_rx.try_recv() {
                let target: Target = last_target;

                controller.set_target_velocity(target.velocity);

                controller.set_heading_rate_target(target.yaw_rate);
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
        Box::new(|cc| Ok(Box::new(SimulatorUI::new(state_rx, target_tx)))),
    )
}
