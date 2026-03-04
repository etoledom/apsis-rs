use eframe::egui;
use flight_core::units::Velocity;

use crate::theme::*;

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

    fn draw_bar_centered(
        painter: &egui::Painter,
        rect: egui::Rect,
        value: f32, // signed, m/s
        max: f32,   // positive max scale
        color: egui::Color32,
    ) {
        // Dark background track pill
        painter.rect_filled(rect, 4.0, TRACK);

        // Center x = zero point. Left = negative, right = positive.
        let center_x = rect.center().x;

        // Normalize to 0..1 within one half, clamp to bounds
        let fill_frac = (value.abs() / max).clamp(0.0, 1.0);
        let fill_width = fill_frac * (rect.width() / 2.0);

        // Fill left or right of center depending on sign
        let fill_rect = if value >= 0.0 {
            egui::Rect::from_min_max(
                egui::pos2(center_x, rect.top()),
                egui::pos2(center_x + fill_width, rect.bottom()),
            )
        } else {
            egui::Rect::from_min_max(
                egui::pos2(center_x - fill_width, rect.top()),
                egui::pos2(center_x, rect.bottom()),
            )
        };

        painter.rect_filled(fill_rect, 4.0, color);

        // Center zero tick
        painter.line_segment(
            [
                egui::pos2(center_x, rect.top()),
                egui::pos2(center_x, rect.bottom()),
            ],
            egui::Stroke::new(1.0, egui::Color32::GRAY),
        );
    }

    fn draw_bar_left(
        painter: &egui::Painter,
        rect: egui::Rect,
        value: f32, // positive scalar
        max: f32,
        color: egui::Color32,
    ) {
        // Dark background track
        painter.rect_filled(rect, 4.0, TRACK);

        // Fill from left edge, fraction of total width
        let fill_frac = (value / max).clamp(0.0, 1.0);
        let fill_rect = egui::Rect::from_min_max(
            rect.min,
            egui::pos2(rect.left() + fill_frac * rect.width(), rect.bottom()),
        );

        painter.rect_filled(fill_rect, 4.0, color);
    }

    pub fn show(&self, ui: &mut egui::Ui) {
        let (vel_north, vel_east) = (self.vel_north.0 as f32, self.vel_east.0 as f32);
        let max_v = 10.0_f32;
        let bar_height = 8.0;
        let gnd_color = ACCENT; // cyan
        let n_color = ACCENT; // cyan
        let e_color = ORANGE; // orange

        let gnd = self.ground_speed();
        let dir = self.cardinal_direction();

        // ── SECTION LABEL ──
        ui.label(
            egui::RichText::new("VELOCITY")
                .monospace()
                .small()
                .color(egui::Color32::DARK_GRAY),
        );

        ui.add_space(4.0);

        // ── GROUND SPEED ROW ──
        // Large value + cardinal direction on one line
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("GND")
                    .monospace()
                    .small()
                    .color(egui::Color32::DARK_GRAY),
            );
            ui.label(
                egui::RichText::new(format!("{:.1} m/s", gnd))
                    .monospace()
                    .size(18.0)
                    .color(gnd_color),
            );
            ui.label(
                egui::RichText::new(dir)
                    .monospace()
                    .size(14.0)
                    .color(egui::Color32::WHITE),
            );
        });

        // Ground speed bar — full available width
        let (gnd_rect, _) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), bar_height),
            egui::Sense::hover(),
        );
        Self::draw_bar_left(&ui.painter_at(gnd_rect), gnd_rect, gnd, max_v, gnd_color);

        ui.add_space(18.0);

        // ── N and E LABELS side by side ──
        // Each takes exactly half the available width
        let total_width = ui.available_width();

        let half = (total_width).max(0.0) / 2.0; // 4px gap between the two

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                // North label
                ui.allocate_ui(egui::vec2(half, 20.0), |ui| {
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new("N")
                                .monospace()
                                .small()
                                .color(egui::Color32::DARK_GRAY),
                        );
                        ui.label(
                            egui::RichText::new(format!("{:+.1} m/s", vel_north))
                                .monospace()
                                .color(n_color),
                        );
                    });
                });
                ui.add_space(4.0);

                // ── N and E BARS side by side ──

                // North bar — half width, center zero
                let (n_rect, _) = ui
                    .allocate_exact_size(egui::vec2(half - 4.0, bar_height), egui::Sense::hover());
                Self::draw_bar_centered(&ui.painter_at(n_rect), n_rect, vel_north, max_v, n_color);
            });

            ui.vertical(|ui| {
                // East label
                ui.allocate_ui(egui::vec2(half, 20.0), |ui| {
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new("E")
                                .monospace()
                                .small()
                                .color(egui::Color32::DARK_GRAY),
                        );
                        ui.label(
                            egui::RichText::new(format!("{:+.1} m/s", vel_east))
                                .monospace()
                                .color(e_color),
                        );
                    });
                });
                ui.add_space(4.0);
                // East bar — half width, center zero
                let (e_rect, _) = ui
                    .allocate_exact_size(egui::vec2(half - 4.0, bar_height), egui::Sense::hover());
                Self::draw_bar_centered(&ui.painter_at(e_rect), e_rect, vel_east, max_v, e_color);
            });
        });
    }
}
