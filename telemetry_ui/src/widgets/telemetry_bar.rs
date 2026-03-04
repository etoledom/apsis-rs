use crate::theme;
use eframe::egui;
use flight_core::state::State;

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
                    // Remove spacing between adjacent labels so dim labels
                    // and white values sit flush against each other
                    ui.spacing_mut().item_spacing.x = 0.0;

                    // ── LEFT: status indicator ──
                    ui.label(
                        egui::RichText::new("● ACTIVE  ")
                            .font(theme::label_font())
                            .color(theme::GREEN),
                    );

                    ui.separator();
                    ui.add_space(8.0);

                    // ── POSITION ──
                    ui.label(theme::label("POS  "));
                    ui.label(theme::label("N:"));
                    ui.label(theme::value(format!("{:.2}", state.position_ned.north().0)));
                    ui.label(theme::label("  E:"));
                    ui.label(theme::value(format!("{:.2}", state.position_ned.east().0)));
                    ui.label(theme::label("  D:"));
                    ui.label(theme::value(format!("{:.2}", state.position_ned.down().0)));

                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(8.0);

                    let display_down_speed = if state.velocity_ned.down().0.abs() < 0.01 {
                        0.0
                    } else {
                        state.velocity_ned.down().0
                    };

                    // ── VELOCITY ──
                    ui.label(theme::label("VEL  "));
                    ui.label(theme::label("N:"));
                    ui.label(theme::value(format!("{:.2}", state.velocity_ned.north().0)));
                    ui.label(theme::label("  E:"));
                    ui.label(theme::value(format!("{:.2}", state.velocity_ned.east().0)));
                    ui.label(theme::label("  D:"));
                    ui.label(theme::value(format!("{:.2}", display_down_speed)));

                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(8.0);

                    // ── ATTITUDE ──
                    ui.label(theme::label("ATT  "));
                    ui.label(theme::label("P:"));
                    ui.label(theme::value(format!(
                        "{:.1}°",
                        state.attitude.pitch().to_degrees().raw()
                    )));
                    ui.label(theme::label("  R:"));
                    ui.label(theme::value(format!(
                        "{:.1}°",
                        state.attitude.roll().to_degrees().raw()
                    )));
                    ui.label(theme::label("  Y:"));
                    ui.label(theme::value(format!(
                        "{:.1}°",
                        state.attitude.yaw_normalized().raw()
                    )));

                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(8.0);

                    // ── INPUTS ──
                    ui.label(theme::label("THR:"));
                    ui.label(theme::value(format!(
                        "{:.0}%",
                        state.last_inputs.throttle.get() * 100.0
                    )));
                    ui.label(theme::label("  P:"));
                    ui.label(theme::value(format!(
                        "{:.2}",
                        state.last_inputs.pitch.get()
                    )));
                    ui.label(theme::label("  R:"));
                    ui.label(theme::value(format!("{:.2}", state.last_inputs.roll.get())));

                    // ── RIGHT: version ──
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(theme::label("flight_core v0.1.0"));
                    });
                });
            });
    }
}
