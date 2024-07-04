use std::{ops::RangeInclusive, sync::Arc};

use color_eyre::{eyre::OptionExt, Result};
use communication::messages::TextOrBinary;
use eframe::{
    egui::{ComboBox, Response, Slider, Ui, Widget, WidgetText},
    emath::Numeric,
};
use log::{error, info};
use parameters::directory::Scope;
use serde::{Deserialize, Serialize};
use serde_json::{to_value, Value};

use crate::{nao::Nao, panel::Panel, value_buffer::BufferHandle};

use super::image::cycler_selector::VisionCycler;

pub struct VisionTunerPanel {
    nao: Arc<Nao>,
    cycler: VisionCycler,
    vertical_edge_threshold: BufferHandle<u8>,
    red_chromaticity_threshold: BufferHandle<f32>,
    blue_chromaticity_threshold: BufferHandle<f32>,
    green_chromaticity_threshold: BufferHandle<f32>,
    green_luminance_threshold: BufferHandle<u8>,
    luminance_threshold: BufferHandle<u8>,
}

impl Panel for VisionTunerPanel {
    const NAME: &'static str = "Vision Tuner";

    fn new(nao: Arc<Nao>, _value: Option<&Value>) -> Self {
        let cycler = VisionCycler::Top;

        let cycler_path = cycler.as_snake_case_path();
        let vertical_edge_threshold = nao.subscribe_value(format!(
            "parameters.image_segmenter.{cycler_path}.vertical_edge_threshold"
        ));
        let red_chromaticity_threshold = nao.subscribe_value(format!(
            "parameters.field_color_detection.{cycler_path}.red_chromaticity_threshold"
        ));
        let blue_chromaticity_threshold = nao.subscribe_value(format!(
            "parameters.field_color_detection.{cycler_path}.blue_chromaticity_threshold"
        ));
        let green_chromaticity_threshold = nao.subscribe_value(format!(
            "parameters.field_color_detection.{cycler_path}.green_chromaticity_threshold"
        ));
        let green_luminance_threshold = nao.subscribe_value(format!(
            "parameters.field_color_detection.{cycler_path}.green_luminance_threshold"
        ));
        let luminance_threshold = nao.subscribe_value(format!(
            "parameters.field_color_detection.{cycler_path}.luminance_threshold"
        ));

        Self {
            nao,
            cycler,
            vertical_edge_threshold,
            red_chromaticity_threshold,
            blue_chromaticity_threshold,
            green_chromaticity_threshold,
            green_luminance_threshold,
            luminance_threshold,
        }
    }
}

impl Widget for &mut VisionTunerPanel {
    fn ui(self, ui: &mut Ui) -> Response {
        match self.draw(ui) {
            Ok(response) => response,
            Err(error) => ui.label(format!("{error:#}")),
        }
    }
}

impl VisionTunerPanel {
    fn draw(&mut self, ui: &mut Ui) -> Result<Response> {
        let cycler_path = self.cycler.as_snake_case_path();
        ui.style_mut().spacing.slider_width = ui.available_size().x - 250.0;
        self.add_selector_row(ui);
        let (slider, value) = draw_parameter_slider(
            ui,
            &self.vertical_edge_threshold,
            "vertical_edge_threshold",
            0..=255,
        )?;
        if slider.changed() {
            self.nao.write(
                format!("parameters.image_segmenter.{cycler_path}.vertical_edge_threshold",),
                TextOrBinary::Text(to_value(value)?),
            );
        }
        let (slider, value) = draw_parameter_slider(
            ui,
            &self.red_chromaticity_threshold,
            "red_chromaticity_threshold",
            0.0..=1.0,
        )?;
        if slider.changed() {
            self.nao.write(
                format!(
                    "parameters.field_color_detection.{cycler_path}.red_chromaticity_threshold",
                ),
                TextOrBinary::Text(to_value(value)?),
            );
        }
        let (slider, value) = draw_parameter_slider(
            ui,
            &self.blue_chromaticity_threshold,
            "blue_chromaticity_threshold",
            0.0..=1.0,
        )?;
        if slider.changed() {
            self.nao.write(
                format!(
                    "parameters.field_color_detection.{cycler_path}.blue_chromaticity_threshold",
                ),
                TextOrBinary::Text(to_value(value)?),
            );
        }
        let (slider, value) = draw_parameter_slider(
            ui,
            &self.green_chromaticity_threshold,
            "green_chromaticity_threshold",
            0.0..=1.0,
        )?;
        if slider.changed() {
            self.nao.write(
                format!(
                    "parameters.field_color_detection.{cycler_path}.green_chromaticity_threshold",
                ),
                TextOrBinary::Text(to_value(value)?),
            );
        }
        let (slider, value) = draw_parameter_slider(
            ui,
            &self.green_luminance_threshold,
            "green_luminance_threshold",
            0..=255,
        )?;
        if slider.changed() {
            self.nao.write(
                format!(
                    "parameters.field_color_detection.{cycler_path}.green_luminance_threshold",
                ),
                TextOrBinary::Text(to_value(value)?),
            );
        }
        let (slider, value) = draw_parameter_slider(
            ui,
            &self.luminance_threshold,
            "luminance_threshold",
            0..=255,
        )?;
        if slider.changed() {
            self.nao.write(
                format!("parameters.field_color_detection.{cycler_path}.luminance_threshold",),
                TextOrBinary::Text(to_value(value)?),
            );
        }
        Ok(slider)
    }

    fn add_selector_row(&mut self, ui: &mut Ui) -> Response {
        let cycler_path = self.cycler.as_snake_case_path();
        ui.horizontal(|ui| {
            self.add_vision_cycler_selector(ui);
            if ui.button("Save current view to Head").clicked() {
                let vertical_edge_threshold = self
                    .vertical_edge_threshold
                    .get_last_value()
                    .unwrap()
                    .unwrap();
                if let Err(error) = self.nao.store_parameters(
                    &format!("image_segmenter.{cycler_path}.vertical_edge_threshold"),
                    to_value(vertical_edge_threshold).unwrap(),
                    Scope::current_head(),
                ) {
                    error!("{error:#}");
                }
                info!("Saved vertical_edge_threshold: {vertical_edge_threshold}");

                let red_chromaticity_threshold = self
                    .red_chromaticity_threshold
                    .get_last_value()
                    .unwrap()
                    .unwrap();
                if let Err(error) = self.nao.store_parameters(
                    &format!("field_color_detection.{cycler_path}.red_chromaticity_threshold"),
                    to_value(red_chromaticity_threshold).unwrap(),
                    Scope::current_head(),
                ) {
                    error!("{error:#}");
                }
                info!("Saved red_chromaticity_threshold: {red_chromaticity_threshold}");

                let blue_chromaticity_threshold = self
                    .blue_chromaticity_threshold
                    .get_last_value()
                    .unwrap()
                    .unwrap();
                if let Err(error) = self.nao.store_parameters(
                    &format!("field_color_detection.{cycler_path}.blue_chromaticity_threshold"),
                    to_value(blue_chromaticity_threshold).unwrap(),
                    Scope::current_head(),
                ) {
                    error!("{error:#}");
                }
                info!("Saved blue_chromaticity_threshold: {blue_chromaticity_threshold}");

                let green_chromaticity_threshold = self
                    .green_chromaticity_threshold
                    .get_last_value()
                    .unwrap()
                    .unwrap();
                if let Err(error) = self.nao.store_parameters(
                    &format!("field_color_detection.{cycler_path}.green_chromaticity_threshold"),
                    to_value(green_chromaticity_threshold).unwrap(),
                    Scope::current_head(),
                ) {
                    error!("{error:#}");
                }
                info!("Saved green_chromaticity_threshold: {green_chromaticity_threshold}");

                let green_luminance_threshold = self
                    .green_luminance_threshold
                    .get_last_value()
                    .unwrap()
                    .unwrap();
                if let Err(error) = self.nao.store_parameters(
                    &format!("field_color_detection.{cycler_path}.green_luminance_threshold"),
                    to_value(green_luminance_threshold).unwrap(),
                    Scope::current_head(),
                ) {
                    error!("{error:#}");
                }
                info!("Saved green_luminance_threshold: {green_luminance_threshold}");

                let luminance_threshold =
                    self.luminance_threshold.get_last_value().unwrap().unwrap();
                if let Err(error) = self.nao.store_parameters(
                    &format!("field_color_detection.{cycler_path}.luminance_threshold"),
                    to_value(luminance_threshold).unwrap(),
                    Scope::current_head(),
                ) {
                    error!("{error:#}");
                }
                info!("Saved luminance_threshold: {luminance_threshold}");
            }
        })
        .response
    }

    fn add_vision_cycler_selector(&mut self, ui: &mut Ui) -> Response {
        let mut changed = false;
        let response = ComboBox::from_label("Cycler")
            .selected_text(format!("{:?}", self.cycler))
            .show_ui(ui, |ui| {
                if ui
                    .selectable_value(&mut self.cycler, VisionCycler::Top, "VisionTop")
                    .clicked()
                {
                    changed = true;
                };
                if ui
                    .selectable_value(&mut self.cycler, VisionCycler::Bottom, "VisionBottom")
                    .clicked()
                {
                    changed = true;
                };
            })
            .response;
        if changed {
            let cycler_path = self.cycler.as_snake_case_path();
            self.vertical_edge_threshold = self.nao.subscribe_value(format!(
                "parameters.image_segmenter.{cycler_path}.vertical_edge_threshold"
            ));
            self.red_chromaticity_threshold = self.nao.subscribe_value(format!(
                "parameters.field_color_detection.{cycler_path}.red_chromaticity_threshold"
            ));
            self.blue_chromaticity_threshold = self.nao.subscribe_value(format!(
                "parameters.field_color_detection.{cycler_path}.blue_chromaticity_threshold"
            ));
            self.green_chromaticity_threshold = self.nao.subscribe_value(format!(
                "parameters.field_color_detection.{cycler_path}.green_chromaticity_threshold"
            ));
            self.green_luminance_threshold = self.nao.subscribe_value(format!(
                "parameters.field_color_detection.{cycler_path}.green_luminance_threshold"
            ));
            self.luminance_threshold = self.nao.subscribe_value(format!(
                "parameters.field_color_detection.{cycler_path}.luminance_threshold"
            ));
        }
        response
    }
}

fn draw_parameter_slider<T>(
    ui: &mut Ui,
    buffer: &BufferHandle<T>,
    name: impl Into<WidgetText>,
    range: RangeInclusive<T>,
) -> Result<(Response, T)>
where
    T: Numeric + Serialize,
    for<'de> T: Deserialize<'de>,
{
    let mut value = buffer.get_last_value()?.ok_or_eyre("no value")?;
    let response = ui.add(Slider::new(&mut value, range).text(name).smart_aim(false));
    Ok((response, value))
}
