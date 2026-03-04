use eframe::egui;
use flight_core::units::Velocity;

use crate::theme::TRACK;

pub struct VerticalSpeedIndicator {
    pub speed: Velocity, // positive = climbing, negative = descending
}

impl VerticalSpeedIndicator {
    pub fn show(&self, ui: &mut egui::Ui) {
        let speed = self.speed.0 as f32;

        // Total width of the widget — matches a typical panel width
        let width = ui.available_width();
        let max_speed = 5.0_f32;

        // ── LABEL ──
        ui.label(
            egui::RichText::new("VERT SPEED")
                .monospace()
                .small()
                .color(egui::Color32::DARK_GRAY),
        );

        // ── NUMERIC VALUE ──
        // Large, colored readout above the bar.
        // Color matches bar: green for climb, orange for descend.
        // {:+.1} always shows sign so direction is unambiguous.
        let value_color = if speed >= 0.0 {
            egui::Color32::from_rgb(57, 255, 20) // green = climbing
        } else {
            egui::Color32::from_rgb(255, 107, 53) // orange = descending
        };
        ui.label(
            egui::RichText::new(format!("{:+.1} m/s", speed))
                .monospace()
                .size(18.0)
                .color(value_color),
        );

        // ── TRACK BAR ──
        // Allocate a wide, thin rect — 8px tall like the mockup.
        let (rect, _) = ui.allocate_exact_size(egui::vec2(width, 8.0), egui::Sense::hover());
        let painter = ui.painter_at(rect);

        // Background track — full width dark pill
        painter.rect_filled(
            rect, 4.0, // border radius — makes it a pill shape
            TRACK,
        );

        // Center x is the zero point.
        // Left half = negative (descending), right half = positive (climbing).
        let center_x = rect.center().x;

        // Normalize speed to 0.0..=1.0 within one half of the bar.
        // Clamp so the fill never exceeds the rect bounds.
        let fill_frac = (speed.abs() / max_speed).clamp(0.0, 1.0);

        // How many pixels to fill from center toward left or right.
        let fill_width = fill_frac * (width / 2.0);

        // Build the fill rect:
        // Climbing (positive): fill from center rightward
        // Descending (negative): fill from center leftward
        let fill_rect = if speed >= 0.0 {
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

        painter.rect_filled(fill_rect, 4.0, value_color);

        // Center tick mark — thin vertical line at zero so the pilot
        // can see the neutral point even when speed is near zero
        painter.line_segment(
            [
                egui::pos2(center_x, rect.top()),
                egui::pos2(center_x, rect.bottom()),
            ],
            egui::Stroke::new(1.0, egui::Color32::GRAY),
        );

        // ── SCALE LABELS ──
        // Small labels below the bar: ▼5 on the left, 0 in the center, 5▲ on the right.
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("-5")
                    .monospace()
                    .small()
                    .color(egui::Color32::DARK_GRAY),
            );
            // Push the center label to the middle
            // let remaining = ui.available_width();
            ui.add_space(center_x - 26.0);
            ui.label(
                egui::RichText::new("0")
                    .monospace()
                    .small()
                    .color(egui::Color32::DARK_GRAY),
            );
            ui.add_space(width / 2.0 - 20.0);
            ui.label(
                egui::RichText::new("+5")
                    .monospace()
                    .small()
                    .color(egui::Color32::DARK_GRAY),
            );
        });
    }
}
