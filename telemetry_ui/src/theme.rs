use eframe::egui::{self, Color32, Margin};

// ── Primary palette ──
pub const ACCENT: Color32 = Color32::from_rgb(0, 212, 255); // cyan — primary data
pub const ACCENT_DIM: Color32 = Color32::from_rgb(0, 140, 180); // dimmed cyan
pub const ORANGE: Color32 = Color32::from_rgb(255, 107, 53); // east / descending
pub const GREEN: Color32 = Color32::from_rgb(57, 255, 20); // ok / climbing
pub const YELLOW: Color32 = Color32::from_rgb(255, 204, 0); // warning / horizon
pub const RED: Color32 = Color32::from_rgb(255, 68, 68); // danger / alert
pub const BORDER: Color32 = Color32::from_rgb(30, 35, 48);

// ── Backgrounds ──
pub const BG: Color32 = Color32::from_rgb(14, 15, 17); // main background
pub const PANEL: Color32 = Color32::from_rgb(19, 21, 26); // panel background
pub const TRACK: Color32 = Color32::from_rgb(30, 35, 48); // bar/track background

// ── Text ──
pub const TEXT: Color32 = Color32::from_rgb(200, 214, 229); // primary text
pub const TEXT_DIM: Color32 = Color32::from_rgb(74, 85, 104); // labels / units

pub fn panel_frame() -> egui::Frame {
    egui::Frame::new()
        .fill(PANEL)
        .stroke(egui::Stroke::new(1.0, BORDER))
        .inner_margin(Margin::same(8))
        .outer_margin(Margin::same(0))
        .corner_radius(4.0)
}

pub fn label_font() -> egui::FontId {
    egui::FontId::monospace(8.0)
}

pub fn value_font() -> egui::FontId {
    egui::FontId::monospace(10.0)
}

pub fn x_large_value_font() -> egui::FontId {
    egui::FontId::monospace(22.0)
}

pub fn large_value_font() -> egui::FontId {
    egui::FontId::monospace(18.0)
}

pub fn medium_value_font() -> egui::FontId {
    egui::FontId::monospace(14.0)
}

pub fn small_label_font() -> egui::FontId {
    egui::FontId::monospace(8.0)
}

pub fn label(text: impl Into<String>) -> egui::RichText {
    egui::RichText::new(text).font(label_font()).color(TEXT_DIM)
}

pub fn value(text: impl Into<String>) -> egui::RichText {
    egui::RichText::new(text).font(value_font()).color(TEXT)
}

pub fn large_value(text: impl Into<String>) -> egui::RichText {
    egui::RichText::new(text)
        .font(large_value_font())
        .color(ACCENT)
}
