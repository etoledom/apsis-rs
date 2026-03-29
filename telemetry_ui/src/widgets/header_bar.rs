use eframe::egui::{self, Align, Layout};

use crate::{theme, widgets::components::status_box::StatusBox};
use std::time::Duration;

pub struct HeaderBar {
    pub flight_time: Duration,
    pub loop_hz: f32,
    pub mode: FlightMode,
    pub status: FlightStatus,
}

#[derive(Clone, Copy)]
pub enum FlightMode {
    Manual,
    Autopilot,
}

pub enum FlightStatus {
    Ok,
    Warning,
}

impl HeaderBar {
    pub fn show(&self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("header_bar")
            .frame(theme::panel_frame())
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    // ── Status ---
                    //
                    let (status_text, status_color) = match &self.status {
                        FlightStatus::Ok => ("● OK", theme::GREEN),
                        FlightStatus::Warning => ("● WARN", theme::YELLOW),
                    };
                    StatusBox::new(status_text, status_color).show(ui);

                    ui.add_space(6.0);

                    // --- Mode ---
                    //
                    let (mode_text, mode_color) = match self.mode {
                        FlightMode::Manual => ("MANUAL", theme::ACCENT),
                        FlightMode::Autopilot => ("AUTOPILOT", theme::GREEN),
                    };
                    StatusBox::new(mode_text, mode_color).show(ui);

                    ui.add_space(6.0);

                    // --- Flight Time ---
                    //
                    ui.horizontal(|ui| {
                        ui.label(theme::label("FLT"));

                        let secs = self.flight_time.as_secs();
                        ui.label(theme::value(format!(
                            "{:02}:{:02}:{:02}",
                            secs / 3600,
                            (secs % 3600) / 60,
                            secs % 60
                        )));
                    });

                    // ── RIGHT: loop frequency ──
                    //
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        ui.label(theme::value(format!("{:>4.0} Hz", self.loop_hz)));
                        ui.add_space(2.0);
                        ui.label(theme::label("LOOP"));
                    });
                });
            });
    }
}
