use std::{
    sync::mpsc::{Receiver, Sender},
    time::Instant,
};

use eframe::egui::{self, vec2};
use flight_core::state::State;
use primitives::{traits::RawRepresentable, units::Degrees};

use crate::{
    pilot_control::controller::{Controller, KeyboardController, Target},
    theme::{BG, panel_frame},
    widgets::{
        alerts_panel::AlertsPanel,
        altitude_tape::AltitudeTape,
        attitude_indicator::AttitudeIndicator,
        battery_indicator::BatteryIndicator,
        distance::DistanceFromHome,
        header_bar::{FlightMode, FlightStatus, HeaderBar},
        telemetry_bar::TelemetryBar,
        thrust_guage::ThrustGauge,
        velocity_indicator::VelocityIndicator,
        velocity_target_display::VelocityTargetDisplay,
        vertical_speed_indicator::VerticalSpeedIndicator,
        yaw_ribbon::YawRibbon,
    },
};

pub struct SimulatorUI {
    state_rx: Receiver<State>,
    target_tx: Sender<Target>,
    state: Option<State>,
    target: Target,
    sim_start: Instant,
    last_state_time: Instant,
    loop_hz: f32,
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
            sim_start: Instant::now(),
            last_state_time: Instant::now(),
            loop_hz: 0.0,
        }
    }
}

impl eframe::App for SimulatorUI {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        let now = Instant::now();
        let new_hz = 1.0 / now.duration_since(self.last_state_time).as_secs_f32();
        // Low-pass filter: blend new value with current, ignoring spikes
        self.loop_hz = self.loop_hz * 0.95 + new_hz.min(200.0) * 0.05;

        self.last_state_time = now;

        let keyboard_input = KeyboardController::new(ctx);
        let new_target = keyboard_input.handle_input(&self.target);
        if self.target != new_target {
            self.target = new_target;
            self.target_tx.send(new_target).ok();
        }

        while let Ok(last_state) = self.state_rx.try_recv() {
            self.state = Some(last_state);
        }

        HeaderBar {
            flight_time: self.sim_start.elapsed(),
            loop_hz: self.loop_hz,
            mode: FlightMode::Manual,
            status: FlightStatus::Ok,
        }
        .show(ctx);

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

                    // ui.allocate_ui(vec2(right_width, total_height), |ui| {
                    ui.vertical(|ui| {
                        panel_frame().show(ui, |ui| {
                            ThrustGauge {
                                thrust: state.last_inputs.throttle,
                            }
                            .show(ui);
                        });
                        panel_frame().show(ui, |ui| {
                            BatteryIndicator {
                                battery_pct: state.battery_pct,
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
                                target: self.target,
                            }
                            .show(ui);
                        });
                        panel_frame().show(ui, |ui| {
                            AlertsPanel {
                                battery_pct: state.battery_pct,
                            }
                            .show(ui);
                        });
                        // });
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
