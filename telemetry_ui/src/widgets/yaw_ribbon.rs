use crate::theme;
use eframe::egui;
use egui::{Stroke, pos2, vec2};
use flight_core::units::angles::Degrees;

pub struct YawRibbon {
    pub yaw: Degrees, // degrees, 0-360
}

impl YawRibbon {
    pub fn show(&self, ui: &mut egui::Ui) {
        let width = ui.available_width();
        let height = 30.0;

        let (rect, _) = ui.allocate_exact_size(vec2(width, height), egui::Sense::hover());

        let painter = ui.painter();

        // ── BACKGROUND ──
        painter.rect_filled(rect, 3.0, theme::TRACK);

        // ── TICK MARKS ──
        // Each degree of yaw moves the tape by px_per_deg pixels.
        // We draw ticks for every 5° and labels for every 30°.
        // The center of the ribbon always shows the current yaw.
        let px_per_deg = 1.0_f32;
        let center_x = rect.center().x;

        // How many degrees are visible on each side of center
        let visible_deg = (width / 2.0 / px_per_deg) as i32 + 10;

        let current_deg = (self.yaw.raw_f32() * 10.0).round() / 10.0;

        let anchor = (current_deg / 10.0) as i32 * 10;
        let first_tick = anchor as f32 - (visible_deg as f32 * 10.0);

        let last_tick = anchor as f32 + (visible_deg as f32 * 10.0);

        let mut tick_deg = first_tick;

        while tick_deg <= last_tick {
            let x = center_x + (tick_deg - current_deg) * px_per_deg;

            // Skip if outside rect
            if x < rect.left() || x > rect.right() {
                tick_deg += 10.0;
                continue;
            }

            let normalized = ((tick_deg as i32) % 360 + 360) % 360;

            let is_cardinal = normalized % 90 == 0;
            let is_major = normalized % 30 == 0;
            let is_minor = normalized % 10 == 0;

            let tick_height = if is_cardinal {
                12.0
            } else if is_major {
                8.0
            } else if is_minor {
                5.0
            } else {
                continue; // skip 5° ticks that aren't multiples of 10
            };

            let tick_color = if is_cardinal {
                theme::TEXT
            } else {
                theme::TEXT_DIM
            };

            // Tick line from top of rect downward
            painter.line_segment(
                [pos2(x, rect.top()), pos2(x, rect.top() + tick_height)],
                Stroke::new(if is_cardinal { 1.5 } else { 1.0 }, tick_color),
            );

            if is_major {
                // Always show 3-digit degree number
                let label = format!("{:03}", normalized);
                painter.text(
                    pos2(x, rect.bottom() - 4.0),
                    egui::Align2::CENTER_BOTTOM,
                    label,
                    egui::FontId::monospace(9.0),
                    // Highlight 000, 090, 180, 270 in accent color as subtle cardinal reference
                    if is_cardinal {
                        theme::ACCENT
                    } else {
                        theme::TEXT_DIM
                    },
                );
            }

            tick_deg += 10.0;
        }
        // ── CENTER POINTER ──
        // Fixed downward triangle at center — the current heading marker.
        // The tape scrolls under it.
        let tri_tip = pos2(center_x, rect.top() + 2.0);
        let tri_left = pos2(center_x - 5.0, rect.top() + 10.0);
        let tri_right = pos2(center_x + 5.0, rect.top() + 10.0);

        painter.add(egui::Shape::convex_polygon(
            vec![tri_tip, tri_left, tri_right],
            theme::ACCENT,
            Stroke::NONE,
        ));

        // ── CURRENT VALUE BOX ──
        // Small label below the ribbon showing exact heading degrees
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            let normalized = ((self.yaw.raw() as i32) % 360 + 360) % 360;
            ui.add_space((width - 40.0) / 2.0);
            ui.label(theme::value(format!("{:03}°", normalized)));
        });
    }
}
