use eframe::egui::{self, vec2};
use primitives::traits::RawRepresentable;

use crate::{
    pilot_control::controller::Target,
    theme,
    widgets::components::{bar::CenterBar, data_label::DataLabel},
};

pub struct VelocityTargetDisplay {
    pub target: Target,
}

impl VelocityTargetDisplay {
    pub fn show(&self, ui: &mut egui::Ui) {
        let forward = self.target.forward.raw() as f32;
        let right = self.target.right.raw() as f32;
        let yaw = self.target.yaw_rate.raw() as f32;
        let up = self.target.up.raw() as f32;

        ui.set_min_width(ui.available_width());

        ui.label(theme::label("PILOT INPUT"));
        ui.add_space(4.0);

        ui.horizontal(|ui| {
            DataLabel::new(forward).label("F:").show(ui);
            DataLabel::new(right).label("R:").show(ui);
            DataLabel::new(up).unit("m/s").label("Up:").show(ui);
        });
        DataLabel::new(yaw).unit("rad/s").label("Y").show(ui);

        // Visual bars
        //
        let max = 7.0_f32;
        let width = ui.available_width();
        let bar_height = 6.0;

        // Forward bar
        ui.add_space(3.0);

        ui.allocate_ui(vec2(width, bar_height * 3.0), |ui| {
            ui.vertical(|ui| {
                CenterBar::new(forward)
                    .set_max(max)
                    .set_height(bar_height)
                    .show(ui);
                CenterBar::new(right)
                    .set_max(max)
                    .set_height(bar_height)
                    .set_color(theme::ORANGE)
                    .show(ui);
                CenterBar::new(up)
                    .set_max(max)
                    .set_height(bar_height)
                    .set_color(theme::YELLOW)
                    .show(ui);
                CenterBar::new(yaw)
                    .set_max(2.0)
                    .set_height(bar_height)
                    .set_color(theme::GREEN)
                    .show(ui);
            })
        });

        // Key hints
        ui.add_space(4.0);
        ui.label(theme::label("W/S: forward/backward"));
        ui.label(theme::label("A/D: east/west"));
        ui.label(theme::label("<-/-> : Yaw rotation"));
        ui.label(theme::label("R: Reset"));
    }
}
