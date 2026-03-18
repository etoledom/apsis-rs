use eframe::egui::{self, Align2, FontId, Sense, Stroke, Vec2, pos2};
use flight_core::units::{Meters, traits::RawRepresentable};

use crate::theme::*;

pub struct AltitudeTape {
    pub altitude: Meters,
    pub height: f32,
}

impl AltitudeTape {
    pub fn show(&self, ui: &mut egui::Ui) {
        let altitude = self.altitude.raw() as f32;

        let width = ui.available_width();
        let (rect, _) = ui.allocate_exact_size(Vec2::new(width, self.height), Sense::hover());
        let painter = ui.painter_at(rect);

        let pixel_per_meter = 20.0;
        let center_y = rect.center().y;

        let box_width = ui.available_width() / 3.0;
        let line_width = ui.available_width() / 8.0;
        let mayor_line_width = ui.available_width() / 3.0;
        let value_left_margin = ui.available_width() / 10.0;
        let box_height = 20.0;
        let arrow_width = 8.0;

        for offset in -10..=10 {
            let tick_altitude = altitude as i32 + offset;
            let y = center_y - (tick_altitude as f32 - altitude) * pixel_per_meter;

            if y < rect.top() || y > rect.bottom() {
                continue;
            }

            let is_mayor = tick_altitude % 5 == 0;
            let tick_width = if is_mayor {
                mayor_line_width
            } else {
                line_width
            };

            painter.line_segment(
                [
                    pos2((rect.right() - tick_width) - (box_width + arrow_width), y),
                    pos2(rect.right() - (box_width + arrow_width), y),
                ],
                Stroke::new(1.0, TEXT_DIM),
            );

            if is_mayor {
                painter.text(
                    pos2(rect.left() + value_left_margin, y),
                    Align2::LEFT_CENTER,
                    format!("{}", tick_altitude),
                    FontId::monospace(12.0),
                    TEXT_DIM,
                );
            }
        }

        // Arrow box on the right showing current value
        let arrow_y = center_y;
        let box_x = rect.right() - box_width;

        // Background box
        painter.rect_filled(
            egui::Rect::from_min_size(
                egui::pos2(box_x, arrow_y - box_height / 2.0),
                egui::vec2(box_width, box_height),
            ),
            2.0,
            ACCENT,
        );

        let arrow_half_height = box_height / 2.0;
        // Arrow triangle on the left side of the box
        painter.add(egui::Shape::convex_polygon(
            vec![
                egui::pos2(box_x - 8.0, arrow_y), // tip pointing left
                egui::pos2(box_x + 1.0, arrow_y - arrow_half_height), // top
                egui::pos2(box_x + 1.0, arrow_y + arrow_half_height), // bottom
            ],
            ACCENT,
            egui::Stroke::NONE,
        ));

        // Current value text inside box
        painter.text(
            egui::pos2(box_x + box_width - 4.0, arrow_y),
            egui::Align2::RIGHT_CENTER,
            format!("{:.1} m", altitude),
            egui::FontId::monospace(11.0),
            egui::Color32::BLACK,
        );
    }
}
