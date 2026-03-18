use eframe::egui;
use flight_core::units::{Velocity, traits::RawRepresentable};

use crate::{
    theme::{self},
    widgets::components::{
        bar::CenterBar,
        data_label::{DataLabel, DataLabelSize},
    },
};

pub struct VerticalSpeedIndicator {
    pub speed: Velocity, // positive = climbing, negative = descending
}

impl VerticalSpeedIndicator {
    pub fn show(&self, ui: &mut egui::Ui) {
        let speed = self.speed.raw() as f32;

        // Total width of the widget — matches a typical panel width
        let width = ui.available_width();
        let max_speed = 5.0_f32;

        // ── LABEL ──
        ui.label(
            egui::RichText::new("VERTICAL SPEED")
                .monospace()
                .small()
                .color(theme::TEXT_DIM),
        );

        // ── NUMERIC VALUE ──
        // Large, colored readout above the bar.
        // Color matches bar: green for climb, orange for descend.
        // {:+.1} always shows sign so direction is unambiguous.

        let value_color = if speed >= 0.0 {
            theme::GREEN
        } else {
            theme::ORANGE
        };

        DataLabel::new(speed)
            .unit("m/s")
            .color(value_color)
            .size(DataLabelSize::Large)
            .show(ui);

        // ── TRACK BAR ──
        ui.add_space(12.0);

        CenterBar::new(speed)
            .set_max(max_speed)
            .set_color(value_color)
            .show(ui);

        // ── SCALE LABELS ──
        // Small labels below the bar: ▼5 on the left, 0 in the center, 5▲ on the right.
        //
        let center_x = width / 2.0;
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("-5")
                    .monospace()
                    .small()
                    .color(theme::TEXT_DIM),
            );
            // Push the center label to the middle
            // let remaining = ui.available_width();
            ui.add_space(center_x - 20.0);
            ui.label(
                egui::RichText::new("0")
                    .monospace()
                    .small()
                    .color(theme::TEXT_DIM),
            );
            ui.add_space(width / 2.0 - 20.0);
            ui.label(
                egui::RichText::new("+5")
                    .monospace()
                    .small()
                    .color(theme::TEXT_DIM),
            );
        });
    }
}
