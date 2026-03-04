use crate::theme;
use eframe::egui::{self, Color32, Painter};
use egui::{Rect, Stroke, pos2};

fn draw_track(painter: &Painter, rect: Rect) {
    painter.rect_filled(rect, 4.0, theme::TRACK);
}

fn draw_fill(painter: &egui::Painter, fill_rect: Rect, color: egui::Color32) {
    painter.rect_filled(fill_rect, 4.0, color);
}

fn draw_center_tick(painter: &egui::Painter, rect: Rect) {
    let cx = rect.center().x;
    painter.line_segment(
        [pos2(cx, rect.top()), pos2(cx, rect.bottom())],
        Stroke::new(1.0, theme::TEXT_DIM),
    );
}

fn allocate_rect(ui: &mut egui::Ui, height: f32) -> Rect {
    let width = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    rect
}

// ── PUBLIC STRUCTS ──

/// A bar that fills from the left edge. For positive scalars (battery, ground speed).
pub struct FillBar {
    value: f32,
    max: f32,
    color: egui::Color32,
    height: f32,
}

impl Default for FillBar {
    fn default() -> Self {
        Self {
            value: 0.0,
            max: 1.0,
            color: theme::ACCENT,
            height: 8.0,
        }
    }
}

impl FillBar {
    pub fn new(value: f32) -> Self {
        Self {
            value,
            ..Default::default()
        }
    }
    pub fn set_height(self, height: f32) -> Self {
        Self { height, ..self }
    }
    pub fn set_max(self, max: f32) -> Self {
        Self { max, ..self }
    }
    pub fn set_color(self, color: Color32) -> Self {
        Self { color, ..self }
    }
    pub fn show(&self, ui: &mut egui::Ui) {
        let rect = allocate_rect(ui, self.height);
        let painter = ui.painter_at(rect);
        draw_track(&painter, rect);

        let fill_frac = (self.value / self.max).clamp(0.0, 1.0);
        let fill_rect = Rect::from_min_max(
            rect.min,
            pos2(rect.left() + fill_frac * rect.width(), rect.bottom()),
        );
        draw_fill(&painter, fill_rect, self.color);
    }
}

/// A bar that fills from the center. For signed values (N/E velocity, vertical speed).
pub struct CenterBar {
    pub value: f32,
    pub max: f32,
    pub color: egui::Color32,
    pub height: f32,
}

impl Default for CenterBar {
    fn default() -> Self {
        Self {
            value: 0.0,
            max: 1.0,
            color: theme::ACCENT,
            height: 8.0,
        }
    }
}

impl CenterBar {
    pub fn new(value: f32) -> Self {
        Self {
            value,
            ..Default::default()
        }
    }
    pub fn set_height(self, height: f32) -> Self {
        Self { height, ..self }
    }
    pub fn set_max(self, max: f32) -> Self {
        Self { max, ..self }
    }
    pub fn set_color(self, color: Color32) -> Self {
        Self { color, ..self }
    }
    pub fn show(&self, ui: &mut egui::Ui) {
        let rect = allocate_rect(ui, self.height);
        let painter = ui.painter_at(rect);
        draw_track(&painter, rect);

        let cx = rect.center().x;
        let fill_frac = (self.value.abs() / self.max).clamp(0.0, 1.0);
        let fill_w = fill_frac * (rect.width() / 2.0);

        let fill_rect = if self.value >= 0.0 {
            Rect::from_min_max(pos2(cx, rect.top()), pos2(cx + fill_w, rect.bottom()))
        } else {
            Rect::from_min_max(pos2(cx - fill_w, rect.top()), pos2(cx, rect.bottom()))
        };

        draw_fill(&painter, fill_rect, self.color);
        draw_center_tick(&painter, rect);
    }
}
