use std::collections::HashSet;

use eframe::egui::{self, Key};
use primitives::units::{AngularVelocity, DegreesPerSecond, Velocity, VelocityLiteral};

#[derive(Clone, Copy, PartialEq)]
pub enum Target {
    Pilot(PilotTarget),
    Autopilot,
}

#[derive(Clone, Copy, Default, PartialEq)]
pub struct PilotTarget {
    pub forward: Velocity,
    pub right: Velocity,
    pub up: Velocity,
    pub yaw_rate: AngularVelocity,
}

pub trait Controller {
    fn handle_input(&self, current: &PilotTarget) -> PilotTarget;
}

pub struct KeyboardController<'a> {
    ctx: &'a egui::Context,
}

impl<'a> KeyboardController<'a> {
    pub fn new(ctx: &'a egui::Context) -> Self {
        Self { ctx }
    }
}

impl<'a> Controller for KeyboardController<'a> {
    fn handle_input(&self, current: &PilotTarget) -> PilotTarget {
        let max_vel = 7.mps();
        let step = 0.1.mps(); // m/s per frame
        let mut updated_target = current.clone();

        self.ctx.input(|input| {
            if input.key_down(Key::W) {
                updated_target.forward += step;
            }
            if input.key_down(Key::S) {
                updated_target.forward -= step;
            }
            if input.key_down(Key::D) {
                updated_target.right += step;
            }
            if input.key_down(Key::A) {
                updated_target.right -= step;
            }

            if input.key_down(Key::ArrowUp) {
                updated_target.up += step;
            }
            if input.key_down(Key::ArrowDown) {
                updated_target.up -= step
            }

            if input.key_down(Key::ArrowLeft) {
                updated_target.yaw_rate = DegreesPerSecond(-90.0).to_angular_velocity();
            }
            if input.key_down(Key::ArrowRight) {
                updated_target.yaw_rate = DegreesPerSecond(90.0).to_angular_velocity();
            }
            if input
                .keys_down
                .intersection(&HashSet::from([Key::ArrowRight, Key::ArrowLeft]))
                .count()
                == 0
            {
                updated_target.yaw_rate = Default::default();
            }

            if input.key_down(Key::R) {
                updated_target = Default::default();
            }
        });

        updated_target.forward.clamp(-max_vel, max_vel);
        updated_target.right.clamp(-max_vel, max_vel);
        updated_target.up.clamp(-max_vel, max_vel);

        return updated_target;
    }
}
