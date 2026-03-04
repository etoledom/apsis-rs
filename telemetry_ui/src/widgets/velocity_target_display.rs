use eframe::egui::{self, pos2};
use flight_core::VelocityNED;

use crate::theme;

pub struct VelocityTargetDisplay {
    pub velocity: VelocityNED,
}

impl VelocityTargetDisplay {
    pub fn show(&self, ui: &mut egui::Ui) {
        let east = self.velocity.east().0 as f32;
        let north = self.velocity.north().0 as f32;
        ui.set_min_width(ui.available_width());

        ui.label(theme::label("VELOCITY TARGET"));
        ui.add_space(4.0);

        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            ui.label(theme::label("N:"));
            ui.label(theme::value(format!("{:+.1}", north)));
            ui.label(theme::label("  E:"));
            ui.label(theme::value(format!("{:+.1}", east)));
            ui.label(theme::label("  m/s"));
        });

        // Visual bars so the pilot can see targets at a glance
        // without reading numbers — same style as velocity widget
        let max = 7.0_f32;
        let width = ui.available_width();
        let bar_height = 6.0;

        // North bar
        ui.add_space(3.0);
        let (rect, _) = ui.allocate_exact_size(egui::vec2(width, bar_height), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 3.0, theme::TRACK);
        let center_x = rect.center().x;
        let fill = (north.abs() / max).clamp(0.0, 1.0) * (width / 2.0);
        let fill_rect = if north >= 0.0 {
            egui::Rect::from_min_max(
                pos2(center_x, rect.top()),
                pos2(center_x + fill, rect.bottom()),
            )
        } else {
            egui::Rect::from_min_max(
                pos2(center_x - fill, rect.top()),
                pos2(center_x, rect.bottom()),
            )
        };
        painter.rect_filled(fill_rect, 3.0, theme::ACCENT);
        painter.line_segment(
            [
                egui::pos2(center_x, rect.top()),
                egui::pos2(center_x, rect.bottom()),
            ],
            egui::Stroke::new(1.0, theme::TEXT_DIM),
        );

        // East bar
        ui.add_space(2.0);
        let (rect, _) = ui.allocate_exact_size(egui::vec2(width, bar_height), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 3.0, theme::TRACK);
        let fill = (east.abs() / max).clamp(0.0, 1.0) * (width / 2.0);
        let fill_rect = if east >= 0.0 {
            egui::Rect::from_min_max(
                pos2(center_x, rect.top()),
                pos2(center_x + fill, rect.bottom()),
            )
        } else {
            egui::Rect::from_min_max(
                pos2(center_x - fill, rect.top()),
                pos2(center_x, rect.bottom()),
            )
        };
        painter.rect_filled(fill_rect, 3.0, theme::ORANGE);
        painter.line_segment(
            [
                egui::pos2(center_x, rect.top()),
                egui::pos2(center_x, rect.bottom()),
            ],
            egui::Stroke::new(1.0, theme::TEXT_DIM),
        );

        // Key hints
        ui.add_space(4.0);
        ui.label(theme::label("W/S: north  A/D: east  R: reset"));
    }
}
