use std::collections::HashSet;

use eframe::egui::{self, Key};
use flight_core::{
    VelocityNED,
    units::{VelocityLiteral, angles::DegreesPerSecond},
};

#[derive(Clone, Copy, Default, PartialEq)]
pub struct Target {
    pub velocity: VelocityNED,
    pub yaw_rate: DegreesPerSecond,
}

pub trait Controller {
    fn handle_input(&self, current: &Target) -> Target;
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
    fn handle_input(&self, current: &Target) -> Target {
        let max_vel = 7.mps();
        let step = 0.1.mps(); // m/s per frame
        let mut updated_target = current.clone();

        self.ctx.input(|input| {
            if input.key_down(Key::W) {
                updated_target.velocity.add_north(step);
            }
            if input.key_down(Key::S) {
                updated_target.velocity.add_north(-step);
            }
            if input.key_down(Key::D) {
                updated_target.velocity.add_east(step);
            }
            if input.key_down(Key::A) {
                updated_target.velocity.add_east(-step);
            }
            if input.key_down(Key::ArrowUp) {
                updated_target.velocity.add_down(-step);
            }
            if input.key_down(Key::ArrowDown) {
                updated_target.velocity.add_down(step);
            }
            if input.key_down(Key::ArrowLeft) {
                updated_target.yaw_rate = DegreesPerSecond(-90.0);
            }
            if input.key_down(Key::ArrowRight) {
                updated_target.yaw_rate = DegreesPerSecond(90.0);
            }
            if input
                .keys_down
                .intersection(&HashSet::from([Key::ArrowRight, Key::ArrowLeft]))
                .count()
                == 0
            {
                updated_target.yaw_rate = DegreesPerSecond(0.0);
            }
            if input.key_down(Key::R) {
                updated_target = Default::default();
            }
        });

        updated_target.velocity.clamp_north(-max_vel, max_vel);
        updated_target.velocity.clamp_east(-max_vel, max_vel);
        updated_target.velocity.clamp_down(-max_vel, max_vel);

        return updated_target;
    }
}
