use crate::{
    theme,
    widgets::components::{data_label::DataLabel, status_box::StatusBox},
};
use eframe::egui;
use flight_core::state::State;
use primitives::traits::RawRepresentable;

pub struct TelemetryBar<'a> {
    pub state: &'a Option<State>,
}

impl<'a> TelemetryBar<'a> {
    pub fn show(&self, ctx: &egui::Context) {
        egui::TopBottomPanel::bottom("telemetry_bar")
            .frame(theme::panel_frame())
            .show(ctx, |ui| {
                let Some(state) = self.state else {
                    ui.label(
                        egui::RichText::new("● WAITING FOR TELEMETRY")
                            .font(theme::label_font())
                            .color(theme::RED),
                    );
                    return;
                };

                ui.horizontal(|ui| {
                    // ── LEFT: status indicator ──
                    //
                    StatusBox::new("● ACTIVE", theme::GREEN).show(ui);

                    ui.separator();
                    ui.add_space(4.0);

                    // ── POSITION ──
                    //
                    ui.label(theme::label("POS "));
                    DataLabel::new(state.position_ned.north().raw() as f32)
                        .label("N:")
                        .show(ui);
                    DataLabel::new(state.position_ned.east().raw() as f32)
                        .label("E:")
                        .show(ui);
                    DataLabel::new(state.position_ned.down().raw() as f32)
                        .label("D:")
                        .show(ui);

                    ui.add_space(4.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // ── VELOCITY ──
                    //
                    ui.label(theme::label("VEL "));

                    DataLabel::new(state.velocity_ned.north().raw() as f32)
                        .label("N:")
                        .show(ui);
                    DataLabel::new(state.velocity_ned.east().raw() as f32)
                        .label("E:")
                        .show(ui);
                    DataLabel::new(state.velocity_ned.down().raw() as f32)
                        .label("D:")
                        .show(ui);

                    ui.add_space(4.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // ── ATTITUDE ──
                    //
                    ui.label(theme::label("ATT "));

                    DataLabel::new(state.attitude.pitch().to_degrees().raw() as f32)
                        .label("P:")
                        .show(ui);
                    DataLabel::new(state.attitude.roll().to_degrees().raw() as f32)
                        .label("R:")
                        .show(ui);
                    DataLabel::new(state.attitude.yaw().to_degrees().raw() as f32)
                        .label("Y:")
                        .show(ui);

                    ui.add_space(4.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // ── INPUTS ──
                    //
                    DataLabel::new(state.last_inputs.throttle.raw() as f32)
                        .label("THR:")
                        .show(ui);
                    DataLabel::new(state.last_inputs.pitch.raw() as f32)
                        .label("P:")
                        .show(ui);
                    DataLabel::new(state.last_inputs.roll.raw() as f32)
                        .label("R:")
                        .show(ui);

                    // ── RIGHT: version ──
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(theme::label("v0.1.0"));
                    });
                });
            });
    }
}
