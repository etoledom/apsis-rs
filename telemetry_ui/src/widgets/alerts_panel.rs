use eframe::egui;

use crate::{theme, widgets::components::status_box::StatusBox};

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

            StatusBox::new(text, color).full_width().show(ui);
        } else {
            ui.label(theme::label("● ALL SYSTEMS NOMINAL").color(theme::GREEN));
        }
    }
}
