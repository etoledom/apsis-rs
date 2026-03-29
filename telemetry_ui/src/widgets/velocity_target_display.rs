use eframe::egui::{self, RichText, vec2};
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
    pub fn show(&self, ui: &mut egui::Ui, on_go_home: impl FnOnce()) {
        let target = match self.target {
            Target::Pilot(pilot_target) => pilot_target,
            Target::Autopilot => Default::default(),
        };

        let is_autopilot = matches!(self.target, Target::Autopilot);

        let forward = target.forward.raw() as f32;
        let right = target.right.raw() as f32;
        let yaw = target.yaw_rate.raw() as f32;
        let up = target.up.raw() as f32;

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

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                // Key hints
                ui.add_space(4.0);
                ui.label(theme::label("W/S: forward/backward"));
                ui.label(theme::label("A/D: east/west"));
                ui.label(theme::label("<-/-> : Yaw rotation"));
                ui.label(theme::label("R: Reset"));
            });
            ui.vertical(|ui| {
                let background = if is_autopilot {
                    theme::BG
                } else {
                    theme::PANEL
                };
                let text_color = if is_autopilot {
                    theme::TEXT_DIM
                } else {
                    theme::TEXT
                };
                let stroke_color = if is_autopilot {
                    theme::PANEL
                } else {
                    theme::ORANGE
                };
                let response = ui.add(
                    egui::Button::new(
                        RichText::new("GO HOME")
                            .font(theme::value_font())
                            .color(text_color),
                    )
                    .fill(background)
                    .stroke(egui::Stroke::new(1.0, stroke_color)),
                );

                if !is_autopilot && response.clicked() {
                    on_go_home();
                }
            });
        });
    }
}
