use crate::theme;
use eframe::egui::{self, FontId};
use egui::RichText;

#[derive(PartialEq)]
pub enum DataLabelSize {
    XLarge,
    Large,
    Medium,
    Small,
}

/// A labeled data readout component for displaying measured values.
///
/// # Examples
/// ```
/// DataLabel::new(47.3)
///     .unit("m/s")
///     .label("ALT")
///     .size(DataLabelSize::Big)
///     .color(theme::ACCENT)
///     .hide_plus()
///     .show(ui);
///
/// DataLabel::new(3.2)
///     .label("N:")
///     .show(ui);
/// ```
pub struct DataLabel {
    value: f32,
    unit: Option<&'static str>,
    label: Option<&'static str>,
    color: egui::Color32,
    size: DataLabelSize,
    show_plus: bool,
}

impl DataLabel {
    pub fn new(value: f32) -> Self {
        Self {
            value,
            unit: None,
            label: None,
            color: theme::TEXT,
            size: DataLabelSize::Small,
            show_plus: true,
        }
    }

    pub fn label(self, label: &'static str) -> Self {
        Self {
            label: Some(label),
            ..self
        }
    }

    pub fn unit(self, unit: &'static str) -> Self {
        Self {
            unit: Some(unit),
            ..self
        }
    }

    pub fn color(self, color: egui::Color32) -> Self {
        Self { color, ..self }
    }

    pub fn size(self, size: DataLabelSize) -> Self {
        Self { size, ..self }
    }

    pub fn hide_plus(self) -> Self {
        Self {
            show_plus: false,
            ..self
        }
    }

    fn format_value(&self) -> String {
        if self.show_plus {
            format!("{:+.1}", self.value)
        } else {
            format!("{:.1}", self.value)
        }
    }

    fn font_for_size(size: &DataLabelSize) -> FontId {
        match size {
            DataLabelSize::XLarge => theme::x_large_value_font(),
            DataLabelSize::Large => theme::large_value_font(),
            DataLabelSize::Medium => theme::medium_value_font(),
            DataLabelSize::Small => theme::small_label_font(),
        }
    }

    pub fn show(&self, ui: &mut egui::Ui) {
        match self.size {
            DataLabelSize::XLarge => self.show_big(ui),
            DataLabelSize::Large => self.show_big(ui),
            DataLabelSize::Medium => self.show_big(ui),
            DataLabelSize::Small => self.show_small(ui),
        }
    }

    fn show_big(&self, ui: &mut egui::Ui) {
        // Large value + smaller unit inline
        ui.horizontal(|ui| {
            if let Some(label) = self.label {
                ui.label(theme::label(label));
            }
            ui.spacing_mut().item_spacing.x = 2.0;
            ui.label(
                RichText::new(self.format_value())
                    .font(Self::font_for_size(&self.size))
                    .color(self.color),
            );
            if let Some(unit) = self.unit {
                ui.label(
                    RichText::new(format!(" {}", unit))
                        .font(theme::label_font())
                        .color(theme::TEXT_DIM),
                );
            }
        });
    }

    fn show_small(&self, ui: &mut egui::Ui) {
        // All inline: label + value + unit
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            if let Some(label) = self.label {
                ui.label(
                    RichText::new(format!("{} ", label))
                        .font(theme::label_font())
                        .color(theme::TEXT_DIM),
                );
            }
            ui.label(
                RichText::new(self.format_value())
                    .font(theme::value_font())
                    .color(self.color),
            );
            if let Some(unit) = self.unit {
                ui.label(
                    RichText::new(format!(" {}", unit))
                        .font(theme::label_font())
                        .color(theme::TEXT_DIM),
                );
            }
        });
    }
}
