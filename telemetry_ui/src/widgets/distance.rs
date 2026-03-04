use eframe::egui;
use flight_core::units::Meters;

use crate::theme::{self, TEXT, TEXT_DIM};

pub struct DistanceFromHome {
    pub north: Meters,
    pub east: Meters,
}

impl DistanceFromHome {
    fn distance(&self) -> f32 {
        // Horizontal distance only — altitude not included.
        // This is the straight line distance from takeoff point on the ground plane.
        (self.north.0 * self.north.0 + self.east.0 * self.east.0).sqrt() as f32
    }

    fn cardinal_direction(&self) -> &'static str {
        // Direction FROM home TO drone.
        // atan2(east, north) gives bearing clockwise from north in radians.
        // Only meaningful if we're far enough from home to have a real direction.
        let bearing = self.east.0.atan2(self.north.0).to_degrees();
        let bearing = (bearing + 360.0) % 360.0;
        let index = ((bearing + 11.25) / 22.5) as usize % 16;
        [
            "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW",
            "NW", "NNW",
        ][index]
    }

    pub fn show(&self, ui: &mut egui::Ui) {
        let dist = self.distance();
        let width = ui.available_width();
        ui.set_min_width(width);

        // ── SECTION LABEL ──
        ui.label(
            egui::RichText::new("DISTANCE FROM HOME")
                .monospace()
                .small()
                .color(theme::TEXT_DIM),
        );

        ui.add_space(4.0);

        // ── MAIN READOUT ──
        // Distance is the hero value — large and yellow (warning color,
        // since distance from home is something the pilot should always
        // be aware of). Direction sits beside it as supporting info.
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(format!("{:.0}", dist))
                    .monospace()
                    .size(28.0)
                    .strong()
                    .color(theme::YELLOW),
            );
            ui.label(
                egui::RichText::new(" m")
                    .monospace()
                    .size(12.0)
                    .color(TEXT_DIM),
            );

            // Only show direction if far enough from home to be meaningful.
            // Below ~1m the bearing is noise from floating point.
            if dist > 1.0 {
                ui.add_space(6.0);
                ui.label(
                    egui::RichText::new(self.cardinal_direction())
                        .monospace()
                        .size(16.0)
                        .color(TEXT),
                );
            }
        });

        // ── RAW NED COORDS ──
        // Small debug readout below for precise positioning.
        // Useful during development and for operators who want exact values.
        ui.label(
            egui::RichText::new(format!("N {:.1}  ·  E {:.1}", self.north, self.east))
                .monospace()
                .small()
                .color(TEXT_DIM),
        );
    }
}
