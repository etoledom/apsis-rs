use std::{
    os::macos::raw::stat,
    sync::mpsc::{Receiver, Sender},
};

use eframe::egui::{self, vec2};
use flight_core::{VelocityNED, state::State, units::angles::Degrees};

use crate::{
    pilot_control::controller::{Controller, KeyboardController, Target},
    theme::{BG, panel_frame},
    widgets::{
        altitude_tape::AltitudeTape, attitude_indicator::AttitudeIndicator,
        distance::DistanceFromHome, telemetry_bar::TelemetryBar, thrust_guage::ThrustGauge,
        velocity_indicator::VelocityIndicator, velocity_target_display::VelocityTargetDisplay,
        vertical_speed_indicator::VerticalSpeedIndicator, yaw_ribbon::YawRibbon,
    },
};

pub struct SimulatorUI {
    state_rx: Receiver<State>,
    target_tx: Sender<Target>,
    state: Option<State>,
    target: Target,
}

impl SimulatorUI {
    /// Called once before the first frame.
    pub fn new(reveiver: Receiver<State>, sender: Sender<Target>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.
        Self {
            state_rx: reveiver,
            target_tx: sender,
            state: None,
            target: Default::default(),
        }
    }
}

impl eframe::App for SimulatorUI {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        let keyboard_input = KeyboardController::new(ctx);
        let new_target = keyboard_input.handle_input(&self.target);
        if self.target != new_target {
            self.target = new_target;
            self.target_tx.send(new_target).ok();
        }

        while let Ok(last_state) = self.state_rx.try_recv() {
            self.state = Some(last_state);
        }

        TelemetryBar { state: &self.state }.show(ctx);

        egui::CentralPanel::default()
            .frame(window_frame())
            .show(ctx, |ui| {
                // The central panel the region left after adding TopPanel's and SidePanel's
                //
                ctx.request_repaint();

                let total_width = ui.available_width();
                let total_height = ui.available_height();

                let left_width = total_width * 0.25;
                let center_width = total_width * 0.5;
                let right_width = total_width * 0.25;

                let Some(state) = &self.state else {
                    ui.label("Waiting for sim...");
                    return;
                };

                ui.horizontal(|ui| {
                    ui.allocate_ui(vec2(left_width, total_height), |ui| {
                        ui.vertical(|ui| {
                            panel_frame().show(ui, |ui| {
                                AltitudeTape {
                                    altitude: state.altitude,
                                    height: ui.available_height() - 70.0 * 2.0,
                                }
                                .show(ui)
                            });

                            panel_frame().show(ui, |ui| {
                                VerticalSpeedIndicator {
                                    speed: -state.velocity_ned.down(),
                                }
                                .show(ui);
                            });
                        });
                    });

                    ui.allocate_ui(vec2(center_width, total_height), |ui| {
                        ui.vertical(|ui| {
                            panel_frame().show(ui, |ui| {
                                AttitudeIndicator {
                                    pitch: state.attitude.pitch().to_degrees(),
                                    roll: state.attitude.roll().to_degrees(),
                                }
                                .show(ui);
                            });

                            panel_frame().show(ui, |ui| {
                                YawRibbon {
                                    yaw: Degrees(state.attitude.yaw_normalized().raw()),
                                }
                                .show(ui);
                            });

                            panel_frame().show(ui, |ui| {
                                VelocityIndicator {
                                    vel_north: state.velocity_ned.north(),
                                    vel_east: state.velocity_ned.east(),
                                }
                                .show(ui);
                            });
                        });
                    });

                    ui.allocate_ui(vec2(right_width, total_height), |ui| {
                        ui.vertical(|ui| {
                            panel_frame().show(ui, |ui| {
                                ThrustGauge {
                                    thrust: state.last_inputs.throttle,
                                }
                                .show(ui);
                            });
                            panel_frame().show(ui, |ui| {
                                DistanceFromHome {
                                    north: state.position_ned.north(),
                                    east: state.position_ned.east(),
                                }
                                .show(ui);
                            });
                            panel_frame().show(ui, |ui| {
                                VelocityTargetDisplay {
                                    velocity: self.target.velocity,
                                }
                                .show(ui);
                            })
                        });
                    });
                });
            });
    }
}

fn window_frame() -> egui::Frame {
    egui::Frame {
        fill: BG,
        ..Default::default()
    }
}
