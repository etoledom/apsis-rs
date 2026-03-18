use std::f32::consts::PI;

use eframe::egui::{self, Ui};
use primitives::{control::Throttle, traits::RawRepresentable};

use crate::theme::*;

pub struct ThrustGauge {
    pub thrust: Throttle, // 0.0 to 1.0
}

impl ThrustGauge {
    pub fn show(&self, ui: &mut Ui) {
        let thrust = self.thrust.raw() as f32;
        let width = ui.available_width();
        let height = 100.0;

        // ── SECTION LABEL ──
        ui.label(
            egui::RichText::new("THRUST")
                .monospace()
                .small()
                .color(egui::Color32::DARK_GRAY),
        );

        // Allocate a fixed-size region in the UI for this widget.
        // The painter will only draw within this rect.
        let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
        let painter = ui.painter_at(rect);

        // The arc is a semicircle. We anchor the center at the bottom-center
        // of the rect so the arc opens upward, like a speedometer.
        let center = egui::pos2(rect.center().x, rect.bottom());
        let radius = height - 20.0;

        // We draw from 180° (left) to 0° (right) in standard math angles.
        // In egui, angles are in radians, measured clockwise from the right.
        // So PI = left side, 0.0 = right side — this gives us a top semicircle
        // when the center is at the bottom.
        let start_angle = PI; // left end of arc (0% thrust)
        let end_angle = 0.0_f32; // right end of arc (100% thrust)

        // We approximate the arc as many short line segments.
        // More steps = smoother arc. 60 is plenty for this size.
        let steps = 60;

        // ── BACKGROUND ARC ──
        // Draw the full semicircle in a dark color first, so the unfilled
        // portion of the gauge is visible as a dim track.
        let bg_points: Vec<egui::Pos2> = (0..=steps)
            .map(|i| {
                // t goes from 0.0 to 1.0 across the full arc
                let t = i as f32 / steps as f32;
                // Interpolate the angle from start to end
                let angle = start_angle + t * (end_angle - start_angle);
                // Convert polar coordinates (angle, radius) to cartesian (x, y)
                egui::pos2(
                    center.x + radius * angle.cos(),
                    center.y - radius * angle.sin(),
                )
            })
            .collect();

        // Draw each adjacent pair of points as a thick line segment.
        // Together they form a smooth-looking arc.
        for w in bg_points.windows(2) {
            painter.line_segment([w[0], w[1]], egui::Stroke::new(10.0, TRACK));
        }

        // ── FILLED ARC ──
        // Draw the arc again, but only up to the current thrust value.
        // thrust=0.0 draws nothing, thrust=1.0 draws the full arc.
        let fill_steps = (steps as f32 * thrust) as usize;
        let fill_points: Vec<egui::Pos2> = (0..=fill_steps)
            .map(|i| {
                let t = i as f32 / steps as f32;
                let angle = start_angle + t * (end_angle - start_angle);
                egui::pos2(
                    center.x + radius * angle.cos(),
                    center.y - radius * angle.sin(),
                )
            })
            .collect();

        // Color transitions: green (low) → yellow (mid) → red (high).
        // We split the range at 50% and interpolate each half separately.
        let color = if thrust < 0.5 {
            // First half: green → yellow
            // t goes 0.0→1.0 as thrust goes 0%→50%
            let t = thrust * 2.0;
            egui::Color32::from_rgb(
                (255.0 * t) as u8,        // red channel increases
                255,                      // green stays full
                (20.0 * (1.0 - t)) as u8, // blue fades out
            )
        } else {
            // Second half: yellow → red
            // t goes 0.0→1.0 as thrust goes 50%→100%
            let t = (thrust - 0.5) * 2.0;
            egui::Color32::from_rgb(
                255,                       // red stays full
                (255.0 * (1.0 - t)) as u8, // green fades out
                0,
            )
        };

        for w in fill_points.windows(2) {
            painter.line_segment([w[0], w[1]], egui::Stroke::new(10.0, color));
        }

        // ── PERCENTAGE LABEL ──
        // Numeric readout of current thrust inside the arc.
        // Positioned above the center pivot so it doesn't overlap the needle.
        painter.text(
            egui::pos2(center.x, center.y - 10.0),
            egui::Align2::CENTER_CENTER,
            format!("{:.0}%", thrust * 100.0),
            egui::FontId::monospace(18.0),
            ACCENT,
        );
    }
}
