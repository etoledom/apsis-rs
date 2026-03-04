use eframe::egui;

use crate::theme;

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
        let width = ui.available_width();

        ui.label(theme::label("BATTERY"));
        ui.add_space(4.0);

        // ── NUMERIC VALUE ──
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            ui.label(
                egui::RichText::new(format!("{:.1}", self.battery_pct))
                    .font(theme::large_value_font())
                    .color(color),
            );
            ui.label(
                egui::RichText::new(" %")
                    .font(theme::value_font())
                    .color(theme::TEXT_DIM),
            );
        });

        ui.add_space(4.0);

        // ── BAR ──
        // Fills left to right, color changes with level
        let bar_height = 4.0;
        let (rect, _) = ui.allocate_exact_size(egui::vec2(width, bar_height), egui::Sense::hover());
        let painter = ui.painter_at(rect);

        // Background track
        painter.rect_filled(rect, 4.0, theme::TRACK);

        // Fill — left to right, fraction of total width
        let fill_frac = (self.battery_pct / 100.0).clamp(0.0, 1.0) as f32;
        let fill_rect = egui::Rect::from_min_max(
            rect.min,
            egui::pos2(rect.left() + fill_frac * rect.width(), rect.bottom()),
        );
        painter.rect_filled(fill_rect, 4.0, color);
    }
}
