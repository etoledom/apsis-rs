use eframe::egui::{self, vec2};
use flight_core::units::Velocity;

use crate::{
    theme::{self, *},
    widgets::components::{
        bar::{CenterBar, FillBar},
        data_label::{DataLabel, DataLabelSize},
    },
};

pub struct VelocityIndicator {
    pub vel_north: Velocity,
    pub vel_east: Velocity,
}

impl VelocityIndicator {
    // Compute ground speed and cardinal direction from N/E components
    fn ground_speed(&self) -> f32 {
        (self.vel_north * self.vel_north + self.vel_east * self.vel_east)
            .sqrt()
            .0 as f32
    }

    fn cardinal_direction(&self) -> &'static str {
        // atan2 gives angle from north in radians, convert to 0..360
        let bearing = self.vel_east.0.atan2(self.vel_north.0).to_degrees();
        let bearing = (bearing + 360.0) % 360.0;
        // 16-point compass rose, each sector is 22.5°
        // Offset by half a sector (11.25°) so N is centered at 0°
        let index = ((bearing + 11.25) / 22.5) as usize % 16;
        [
            "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW",
            "NW", "NNW",
        ][index]
    }

    pub fn show(&self, ui: &mut egui::Ui) {
        let (vel_north, vel_east) = (self.vel_north.0 as f32, self.vel_east.0 as f32);
        let max_v = 10.0_f32;
        let bar_height = 8.0;
        let gnd_color = ACCENT;
        let north_color = ACCENT;
        let east_color = ORANGE;

        let gnd_speed = self.ground_speed();
        let cardinal_dir = self.cardinal_direction();

        // ── SECTION LABEL ──
        ui.label(
            egui::RichText::new("VELOCITY")
                .monospace()
                .small()
                .color(egui::Color32::DARK_GRAY),
        );

        ui.add_space(4.0);

        // ── GROUND SPEED ROW ──
        //
        ui.horizontal(|ui| {
            DataLabel::new(gnd_speed)
                .unit("m/s")
                .color(gnd_color)
                .label("GND")
                .size(DataLabelSize::Medium)
                .show(ui);

            ui.label(
                egui::RichText::new(cardinal_dir)
                    .font(theme::medium_value_font())
                    .color(theme::TEXT),
            );
        });

        // Ground speed bar —
        //
        FillBar::new(gnd_speed).set_max(max_v).show(ui);

        ui.add_space(18.0);

        // ── N and E ──
        //
        let total_width = ui.available_width();

        let half = (total_width).max(0.0) / 2.0; // 4px gap between the two

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                // North label
                ui.allocate_ui(egui::vec2(half, 20.0), |ui| {
                    DataLabel::new(vel_north)
                        .unit("m/s")
                        .color(north_color)
                        .label("N")
                        .size(DataLabelSize::Medium)
                        .show(ui);
                });
                ui.add_space(4.0);

                // North bar
                ui.allocate_ui(vec2(half, bar_height), |ui| {
                    CenterBar::new(vel_north).set_max(max_v).show(ui);
                });
            });

            ui.vertical(|ui| {
                // East label
                ui.allocate_ui(egui::vec2(half, 20.0), |ui| {
                    DataLabel::new(vel_east)
                        .unit("m/s")
                        .color(east_color)
                        .label("E")
                        .size(DataLabelSize::Medium)
                        .show(ui);
                });
                ui.add_space(4.0);

                // East bar
                ui.allocate_ui(vec2(half, bar_height), |ui| {
                    CenterBar::new(vel_east)
                        .set_max(max_v)
                        .set_color(east_color)
                        .show(ui);
                });
            });
        });
    }
}
