use eframe::egui;

use crate::{
    theme,
    widgets::components::{
        bar::FillBar,
        data_label::{DataLabel, DataLabelSize},
    },
};

pub struct BatteryIndicator {
    pub battery_pct: f64,
}

impl BatteryIndicator {
    fn color(&self) -> egui::Color32 {
        if self.battery_pct < 10.0 {
            theme::RED
        } else if self.battery_pct < 20.0 {
            theme::YELLOW
        } else {
            theme::GREEN
        }
    }

    pub fn show(&self, ui: &mut egui::Ui) {
        let color = self.color();

        ui.label(theme::label("BATTERY"));
        ui.add_space(4.0);

        // ── NUMERIC VALUE ──
        //
        DataLabel::new(self.battery_pct as f32)
            .unit("%")
            .color(color)
            .size(DataLabelSize::Large)
            .hide_plus()
            .show(ui);

        ui.add_space(4.0);

        // ── BAR ──
        // Fills left to right, color changes with level
        FillBar::new(self.battery_pct as f32)
            .set_max(100.0)
            .set_color(color)
            .set_height(4.0)
            .show(ui);
    }
}
