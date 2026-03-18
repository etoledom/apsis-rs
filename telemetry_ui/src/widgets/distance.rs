use eframe::egui;
use flight_core::units::{Meters, traits::RawRepresentable};

use crate::{
    theme::{self},
    widgets::components::data_label::{DataLabel, DataLabelSize},
};

pub struct DistanceFromHome {
    pub north: Meters,
    pub east: Meters,
}

impl DistanceFromHome {
    fn distance(&self) -> f32 {
        // Horizontal distance only — altitude not included.
        // This is the straight line distance from takeoff point on the ground plane.
        (self.north.raw() * self.north.raw() + self.east.raw() * self.east.raw()).sqrt() as f32
    }

    fn cardinal_direction(&self) -> &'static str {
        // Direction FROM home TO drone.
        // atan2(east, north) gives bearing clockwise from north in radians.
        // Only meaningful if we're far enough from home to have a real direction.
        let bearing = self.east.raw().atan2(self.north.raw()).to_degrees();
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
        ui.label(theme::label("DISTANCE FROM HOME"));

        ui.add_space(4.0);

        // ── MAIN READOUT ──
        //
        ui.horizontal(|ui| {
            DataLabel::new(dist)
                .unit("m")
                .color(theme::YELLOW)
                .size(DataLabelSize::XLarge)
                .hide_plus()
                .show(ui);

            // Only show direction if far enough from home to be meaningful.
            // Below ~1m the bearing is noise from floating point.
            if dist > 1.0 {
                ui.add_space(6.0);
                ui.label(theme::large_value(self.cardinal_direction()));
            }
        });

        // ── RAW NED COORDS ──
        //
        ui.horizontal(|ui| {
            DataLabel::new(self.north.raw() as f32)
                .unit("m")
                .label("N")
                .show(ui);
            DataLabel::new(self.east.raw() as f32)
                .unit("m")
                .label("E")
                .show(ui);
        });
    }
}
