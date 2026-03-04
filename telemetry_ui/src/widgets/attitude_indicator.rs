use crate::theme;
use eframe::egui;
use egui::{Painter, Pos2, Stroke, pos2, vec2};
use flight_core::units::angles::{Degrees, Radians};

pub struct AttitudeIndicator {
    /// Pitch in degrees. Positive = nose up (aerospace convention)
    pub pitch: Degrees,
    /// Roll in degrees. Positive = right wing down (aerospace convention)
    pub roll: Degrees,
}

impl AttitudeIndicator {
    /// Rotate a point around a center by an angle in radians.
    /// Used to rotate all drawn elements by the roll angle.
    fn rotate(point: Pos2, center: Pos2, angle: Radians) -> Pos2 {
        let cos = angle.cos() as f32;
        let sin = angle.sin() as f32;
        let dx = point.x - center.x;
        let dy = point.y - center.y;
        pos2(
            center.x + dx * cos - dy * sin,
            center.y + dx * sin + dy * cos,
        )
    }

    /// Draw a line segment with both endpoints rotated around center.
    /// This is how we apply roll to every line we draw —
    /// instead of rotating the coordinate system, we rotate each point.
    fn draw_rotated_line(
        painter: &Painter,
        p1: Pos2,
        p2: Pos2,
        center: Pos2,
        roll: Radians,
        stroke: Stroke,
    ) {
        painter.line_segment(
            [
                Self::rotate(p1, center, roll),
                Self::rotate(p2, center, roll),
            ],
            stroke,
        );
    }

    /// Draw text rotated around center by roll angle.
    /// Used for pitch line degree labels so they rotate with the sphere.
    fn draw_rotated_text(
        painter: &Painter,
        pos: Pos2,
        center: Pos2,
        roll: Radians,
        text: &str,
        font: egui::FontId,
        color: egui::Color32,
    ) {
        let rotated = Self::rotate(pos, center, roll);
        painter.text(rotated, egui::Align2::CENTER_CENTER, text, font, color);
    }

    pub fn show(&self, ui: &mut egui::Ui) {
        // ── ALLOCATE SPACE ──
        // We want a square widget. Use available width as the size,
        // capped at a reasonable maximum so it doesn't get too large.
        let size = ui.available_width(); //.min(220.0);
        // Radius of the ADI circle — slightly smaller than half the rect
        // to leave room for the border
        let radius = (size * 0.48).min(100.0);
        let (rect, _) =
            ui.allocate_exact_size(vec2(size, radius * 2.0 + 40.0), egui::Sense::hover());

        // The center of the ADI circle — all rotations happen around this point
        let center = rect.center();

        // Pixels per degree of pitch.
        // This controls how fast the horizon moves when pitch changes.
        // 3px/deg means a 10° pitch change moves the horizon 30px.
        let px_per_deg = 3.0_f32;

        // Convert roll to radians for trig functions
        let roll_rad = self.roll.to_radians();

        // Pitch offset in pixels: positive pitch = nose up = horizon moves DOWN
        // in screen space (screen Y increases downward).
        // So we ADD pitch * px_per_deg to move horizon down for nose-up pitch.
        let pitch_offset = self.pitch.raw_f32() * px_per_deg;

        // The horizon Y position after applying pitch offset.
        // At zero pitch the horizon is at the center.
        let horizon_y = center.y + pitch_offset;

        let painter = ui.painter_at(rect);

        // ── CLIP TO BOUNDING SQUARE ──
        // egui can only clip to rectangles. We clip to the rect so drawing
        // operations outside the widget area are invisible. We'll use a
        // ring mask later to hide the corners and make it look circular.
        let clipped_painter = painter.with_clip_rect(rect);

        // ── SKY ──
        // The sky is a large rectangle above the horizon.
        // We make it much larger than the widget so it fills the circle
        // even when the sphere is rotated/translated.
        // The sky rect goes from far above to the horizon line.
        //
        // We draw sky and ground as two large rectangles, then rotate
        // their corner points by the roll angle so the horizon tilts correctly.
        //
        // Since egui doesn't have a rotated rect primitive, we use a filled
        // convex polygon (which can be rotated) for the sky and ground.

        // Sky polygon: a large quad above the tilted horizon line.
        // We define it in unrotated space first, then rotate all 4 corners.
        let large = size * 3.0; // large enough to cover the circle at any rotation
        let sky_color = egui::Color32::from_rgb(15, 42, 69);
        let ground_color = egui::Color32::from_rgb(61, 43, 31);

        // Sky quad: covers everything above the horizon
        // Top-left, top-right, horizon-right, horizon-left
        let sky_pts = [
            pos2(center.x - large, horizon_y - large), // top left
            pos2(center.x + large, horizon_y - large), // top right
            pos2(center.x + large, horizon_y),         // horizon right
            pos2(center.x - large, horizon_y),         // horizon left
        ];

        // Ground quad: covers everything below the horizon
        let ground_pts = [
            pos2(center.x - large, horizon_y),         // horizon left
            pos2(center.x + large, horizon_y),         // horizon right
            pos2(center.x + large, horizon_y + large), // bottom right
            pos2(center.x - large, horizon_y + large), // bottom left
        ];

        // Rotate all points by roll angle around center, then draw as polygons
        let sky_rotated: Vec<Pos2> = sky_pts
            .iter()
            .map(|&p| Self::rotate(p, center, roll_rad))
            .collect();

        let ground_rotated: Vec<Pos2> = ground_pts
            .iter()
            .map(|&p| Self::rotate(p, center, roll_rad))
            .collect();

        clipped_painter.add(egui::Shape::convex_polygon(
            sky_rotated,
            sky_color,
            Stroke::NONE,
        ));
        clipped_painter.add(egui::Shape::convex_polygon(
            ground_rotated,
            ground_color,
            Stroke::NONE,
        ));

        // ── PITCH LINES ──
        // Horizontal lines at every 10° of pitch, with labels.
        // They move with pitch (via horizon_y) and rotate with roll.
        // Line width varies: major lines (10°, 20°) are longer than minor (5°).
        for deg in (-50..=50).step_by(5) {
            if deg == 0 {
                continue;
            } // skip zero — that's the horizon line

            let deg = deg as f32;
            // Y position of this pitch line relative to horizon,
            // then offset by pitch so lines move with the sphere
            let line_y = horizon_y - deg * px_per_deg;

            let is_major = deg as i32 % 10 == 0;
            let half_width = if is_major { 20.0 + deg.abs() } else { 15.0 };
            let stroke = Stroke::new(
                1.0,
                egui::Color32::from_rgba_premultiplied(
                    255,
                    255,
                    255,
                    if is_major { 160 } else { 80 },
                ),
            );

            let left = pos2(center.x - half_width, line_y);
            let right = pos2(center.x + half_width, line_y);

            Self::draw_rotated_line(&clipped_painter, left, right, center, roll_rad, stroke);

            // Degree labels on major lines only
            if is_major {
                let label = format!("{}", deg.abs() as i32);
                let font = egui::FontId::monospace(8.0);
                let color = egui::Color32::from_rgba_premultiplied(255, 255, 255, 160);

                // Label on left side
                Self::draw_rotated_text(
                    &clipped_painter,
                    pos2(center.x - half_width - 12.0, line_y),
                    center,
                    roll_rad,
                    &label,
                    font.clone(),
                    color,
                );
                // Label on right side
                Self::draw_rotated_text(
                    &clipped_painter,
                    pos2(center.x + half_width + 12.0, line_y),
                    center,
                    roll_rad,
                    &label,
                    font,
                    color,
                );
            }
        }

        // ── HORIZON LINE ──
        // The main horizon line — bolder than pitch lines.
        // Spans the full width of the circle so it's always visible.
        // Also includes the two short end caps from the mockup.
        let horizon_left = pos2(center.x - radius, horizon_y);
        let horizon_right = pos2(center.x + radius, horizon_y);

        Self::draw_rotated_line(
            &clipped_painter,
            horizon_left,
            horizon_right,
            center,
            roll_rad,
            Stroke::new(2.0, theme::YELLOW),
        );

        // Short thick end caps at each end of the horizon line
        let cap_len = 8.0;
        let cap_left_start = pos2(center.x - radius + cap_len, horizon_y - cap_len / 2.0);
        let cap_left_end = pos2(center.x - radius + cap_len, horizon_y + cap_len / 2.0);
        let cap_right_start = pos2(center.x + radius - cap_len, horizon_y - cap_len / 2.0);
        let cap_right_end = pos2(center.x + radius - cap_len, horizon_y + cap_len / 2.0);

        Self::draw_rotated_line(
            &clipped_painter,
            cap_left_start,
            cap_left_end,
            center,
            roll_rad,
            Stroke::new(3.0, theme::YELLOW),
        );
        Self::draw_rotated_line(
            &clipped_painter,
            cap_right_start,
            cap_right_end,
            center,
            roll_rad,
            Stroke::new(3.0, theme::YELLOW),
        );

        // ── CIRCULAR MASK ──
        // Since egui clips to rectangles, the sky/ground/lines extend into
        // the corners of the rect. We hide this by drawing a thick ring
        // in the background color over the edge of the circle.
        // The ring is thick enough to cover all drawing outside the circle.
        painter.circle_stroke(
            center,
            radius,
            Stroke::new(radius * 2.0, theme::PANEL), // thick, background color. ERROR: This is a widget in a panel so BG should be PANEL
        );

        // ── ROLL INDICATOR ──
        self.roll_indicator(center, radius, roll_rad, &painter);

        // ── FIXED AIRCRAFT SYMBOL ──
        // The W-shaped aircraft reference symbol.
        // This never moves — it's the fixed reference the pilot uses
        // to read pitch and roll from the moving background.
        // Drawn with the unclipped painter so it's always fully visible.
        let sym_y = center.y;
        let sym_half = size * 0.22;
        let sym_stroke = Stroke::new(2.5, theme::YELLOW);

        // Left wing
        painter.line_segment(
            [
                pos2(center.x - sym_half, sym_y),
                pos2(center.x - sym_half * 0.3, sym_y),
            ],
            sym_stroke,
        );
        // Left inner
        painter.line_segment(
            [
                pos2(center.x - sym_half * 0.3, sym_y),
                pos2(center.x - sym_half * 0.15, sym_y + size * 0.04),
            ],
            sym_stroke,
        );
        // Center left
        painter.line_segment(
            [
                pos2(center.x - sym_half * 0.15, sym_y + size * 0.04),
                pos2(center.x, sym_y),
            ],
            sym_stroke,
        );
        // Center right
        painter.line_segment(
            [
                pos2(center.x, sym_y),
                pos2(center.x + sym_half * 0.15, sym_y + size * 0.04),
            ],
            sym_stroke,
        );
        // Right inner
        painter.line_segment(
            [
                pos2(center.x + sym_half * 0.15, sym_y + size * 0.04),
                pos2(center.x + sym_half * 0.3, sym_y),
            ],
            sym_stroke,
        );
        // Right wing
        painter.line_segment(
            [
                pos2(center.x + sym_half * 0.3, sym_y),
                pos2(center.x + sym_half, sym_y),
            ],
            sym_stroke,
        );

        // Center dot
        // painter.circle_filled(center, 3.0, theme::YELLOW);

        // ── READOUT ──
        // Numeric pitch and roll values below the circle for precise reading.
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            ui.label(theme::label("P:"));
            ui.label(theme::value(format!("{:+.1}°  ", self.pitch.raw())));
            ui.label(theme::label("R:"));
            ui.label(theme::value(format!("{:+.1}°", self.roll.raw())));
        });
    }

    fn roll_indicator(&self, center: Pos2, radius: f32, roll: Radians, painter: &Painter) {
        // ── ROLL TICKS (fixed, outside the circle) ──
        // All ticks are drawn OUTSIDE the circle radius so they're never
        // clipped. They sit in the gap between the circle edge and the widget border.

        // Distance from center where ticks start (just outside circle edge)
        let tick_inner = radius + 3.0;

        // Helper: draw a tick at a given angle, from tick_inner outward by tick_len
        let draw_tick = |angle_deg: f32, tick_len: f32, thickness: f32| {
            // Shift by -90° so 0° is at the top (12 o'clock)
            let a = angle_deg.to_radians() - std::f32::consts::FRAC_PI_2;
            let inner = pos2(
                center.x + tick_inner * a.cos(),
                center.y + tick_inner * a.sin(),
            );
            let outer = pos2(
                center.x + (tick_inner + tick_len) * a.cos(),
                center.y + (tick_inner + tick_len) * a.sin(),
            );
            painter.line_segment([inner, outer], Stroke::new(thickness, theme::TEXT));
        };

        // Helper: draw a downward-pointing triangle at a given angle outside the circle.
        // The triangle points inward (toward center) — like an arrow pointing at the circle.
        let draw_triangle = |angle_deg: f32| {
            let a = angle_deg.to_radians() - std::f32::consts::FRAC_PI_2;
            // Radial direction (outward)
            let rad_x = a.cos();
            let rad_y = a.sin();
            // Perpendicular direction
            let perp_x = -a.sin();
            let perp_y = a.cos();

            // Tip points inward toward the circle
            let tip = pos2(
                center.x + (tick_inner + 2.0) * rad_x,
                center.y + (tick_inner + 2.0) * rad_y,
            );
            // Base sits further out from center
            let base_center = pos2(
                center.x + (tick_inner + 9.0) * rad_x,
                center.y + (tick_inner + 9.0) * rad_y,
            );
            let base_left = pos2(base_center.x - perp_x * 4.0, base_center.y - perp_y * 4.0);
            let base_right = pos2(base_center.x + perp_x * 4.0, base_center.y + perp_y * 4.0);

            painter.add(egui::Shape::convex_polygon(
                vec![tip, base_left, base_right],
                theme::TEXT,
                Stroke::NONE,
            ));
        };

        // 0° — fixed reference triangle at top
        draw_triangle(0.0);

        // Symmetric ticks on both sides
        for &side in &[-1.0_f32, 1.0] {
            draw_tick(side * 10.0, 5.0, 1.0); // ±10° small
            draw_tick(side * 20.0, 5.0, 1.0); // ±20° small
            draw_tick(side * 30.0, 9.0, 1.5); // ±30° longer
            draw_triangle(side * 45.0); // ±45° triangle
            draw_tick(side * 60.0, 9.0, 1.5); // ±60° longer
            draw_tick(side * 90.0, 9.0, 2.5); // ±90° thick
        }

        // ── ROLL TRIANGLE POINTER (rotates with roll) ──
        // Small triangle on the circle edge that rotates with the aircraft roll.
        // Tip points outward toward the fixed tick ring so the pilot reads
        // roll by seeing which tick the pointer aligns with.
        let tri_angle = roll.raw_f32() - std::f32::consts::FRAC_PI_2;
        let rad_x = tri_angle.cos();
        let rad_y = tri_angle.sin();
        let perp_x = -tri_angle.sin();
        let perp_y = tri_angle.cos();

        // Tip sits just inside the circle edge
        let tip = pos2(
            center.x + (radius - 2.0) * rad_x,
            center.y + (radius - 2.0) * rad_y,
        );
        // Base sits further inward
        let base_center = pos2(
            center.x + (radius - 11.0) * rad_x,
            center.y + (radius - 11.0) * rad_y,
        );
        let base_left = pos2(base_center.x - perp_x * 5.0, base_center.y - perp_y * 5.0);
        let base_right = pos2(base_center.x + perp_x * 5.0, base_center.y + perp_y * 5.0);

        painter.add(egui::Shape::convex_polygon(
            vec![tip, base_left, base_right],
            theme::YELLOW,
            Stroke::NONE,
        ));
    }
}
