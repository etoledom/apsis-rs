use eframe::egui;

use crate::theme;

/// A bordered box component for displaying status labels, badges, and alerts.
/// Uses a colored outline with a subtle tinted fill of the same color.
///
/// # Examples
/// ```
/// StatusBox::new("● OK", theme::GREEN).show(ui);
/// StatusBox::new("MANUAL", theme::ACCENT).show(ui);
/// StatusBox::new("⚠ LOW BATTERY", theme::YELLOW).show(ui);
/// ```
pub struct StatusBox {
    text: &'static str,
    color: egui::Color32,
    full_width: bool,
}

impl StatusBox {
    pub fn new(text: &'static str, color: egui::Color32) -> Self {
        Self {
            text,
            color,
            full_width: false,
        }
    }

    pub fn full_width(self) -> Self {
        Self {
            full_width: true,
            ..self
        }
    }

    pub fn show(&self, ui: &mut egui::Ui) {
        let bg = egui::Color32::from_rgba_unmultiplied(
            self.color.r(),
            self.color.g(),
            self.color.b(),
            20,
        );
        egui::Frame {
            fill: bg,
            stroke: egui::Stroke::new(1.0, self.color),
            corner_radius: egui::CornerRadius::same(2),
            inner_margin: egui::Margin {
                left: 6,
                right: 6,
                top: 2,
                bottom: 2,
            },
            ..Default::default()
        }
        .show(ui, |ui| {
            if self.full_width {
                ui.set_min_width(ui.available_width());
            }
            ui.label(theme::label(self.text).color(self.color));
        });
    }
}
