use eframe::egui::{self, Align, Layout, vec2};

use crate::theme;
use std::time::Duration;

pub struct HeaderBar {
    pub flight_time: Duration,
    pub loop_hz: f32,
    pub mode: FlightMode,
    pub status: FlightStatus,
}

pub enum FlightMode {
    Manual,
    Autopilot,
}

pub enum FlightStatus {
    Ok,
    Warning(String),
}

impl HeaderBar {
    fn badge(ui: &mut egui::Ui, text: &str, color: egui::Color32) {
        // Same color for border and text, but with low alpha for the fill
        let bg_color = egui::Color32::from_rgba_unmultiplied(
            color.r(),
            color.g(),
            color.b(),
            20, // very low alpha for subtle tint
        );
        egui::Frame {
            fill: bg_color,
            stroke: egui::Stroke::new(1.0, color),
            corner_radius: egui::CornerRadius::same(1),
            inner_margin: egui::Margin {
                left: 6,
                right: 6,
                top: 2,
                bottom: 2,
            },
            ..Default::default()
        }
        .show(ui, |ui| {
            ui.label(
                egui::RichText::new(text)
                    .font(theme::label_font())
                    .color(color),
            );
        });
    }

    pub fn show(&self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("header_bar")
            .frame(theme::panel_frame())
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    // ── LEFT: status badge + mode badge ──
                    // Status
                    let (status_text, status_color) = match &self.status {
                        FlightStatus::Ok => ("● OK", theme::GREEN),
                        FlightStatus::Warning(_) => ("● WARN", theme::YELLOW),
                    };
                    Self::badge(ui, status_text, status_color);

                    ui.add_space(6.0);

                    // Mode
                    let (mode_text, mode_color) = match self.mode {
                        FlightMode::Manual => ("MANUAL", theme::ACCENT),
                        FlightMode::Autopilot => ("AUTOPILOT", theme::GREEN),
                    };
                    Self::badge(ui, mode_text, mode_color);

                    ui.add_space(6.0);

                    // ── CENTER: flight time ──
                    ui.with_layout(
                        egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                        |ui| {
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing.x = 0.0;
                                ui.label(
                                    egui::RichText::new("FLT  ")
                                        .font(theme::label_font())
                                        .color(theme::TEXT_DIM),
                                );
                                let secs = self.flight_time.as_secs();
                                ui.label(
                                    egui::RichText::new(format!(
                                        "{:02}:{:02}:{:02}",
                                        secs / 3600,
                                        (secs % 3600) / 60,
                                        secs % 60
                                    ))
                                    .font(theme::value_font())
                                    .color(theme::ORANGE),
                                );
                            });
                        },
                    );

                    // ── RIGHT: loop frequency ──
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        ui.allocate_ui(vec2(120.0, ui.available_height()), |ui| {
                            ui.label(
                                egui::RichText::new(format!("{:>4.0} Hz", self.loop_hz))
                                    .font(theme::label_font())
                                    .color(theme::TEXT),
                            );
                        });
                        ui.add_space(2.0);
                        ui.label(
                            egui::RichText::new("LOOP")
                                .font(theme::label_font())
                                .color(theme::TEXT_DIM),
                        );
                    });
                });
            });
    }
}
