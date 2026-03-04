use eframe::egui;

use crate::theme;

pub struct AlertsPanel {
    pub battery_pct: f64,
}

impl AlertsPanel {
    pub fn show(&self, ui: &mut egui::Ui) {
        ui.set_min_width(ui.available_width());
        ui.label(theme::label("ALERTS"));
        ui.add_space(4.0);

        if self.battery_pct < 20.0 {
            let (text, color) = if self.battery_pct < 10.0 {
                ("⚠ CRITICAL BATTERY", theme::RED)
            } else {
                ("⚠ LOW BATTERY", theme::YELLOW)
            };

            let bg = egui::Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 20);
            egui::Frame {
                fill: bg,
                stroke: egui::Stroke::new(1.0, color),
                corner_radius: egui::CornerRadius::same(3),
                inner_margin: egui::Margin {
                    left: 6,
                    right: 6,
                    top: 3,
                    bottom: 3,
                },
                ..Default::default()
            }
            .show(ui, |ui| {
                ui.set_min_width(ui.available_width());
                ui.label(
                    egui::RichText::new(text)
                        .font(theme::label_font())
                        .color(color),
                );
            });
        } else {
            ui.label(
                egui::RichText::new("● ALL SYSTEMS NOMINAL")
                    .font(theme::label_font())
                    .color(theme::GREEN),
            );
        }
    }
}
