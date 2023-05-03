use eframe::egui::Widget;
use motionfile::MotionFile;
use serde_json::Value;
use types::Joints;

use crate::{panel::Panel, Nao};

pub struct KeyframeEditorPanel {
    motion: Option<MotionFile<Joints<f32>>>,
}

impl KeyframeEditorPanel {
    fn load_motion_file(&mut self) {}
}

impl Panel for KeyframeEditorPanel {
    const NAME: &'static str = "Keyframe Editor";

    fn new(nao: std::sync::Arc<Nao>, value: Option<&Value>) -> Self {
        Self { motion: None }
    }
}

impl Widget for &mut KeyframeEditorPanel {
    fn ui(self, ui: &mut eframe::egui::Ui) -> eframe::egui::Response {
        ui.label("Hello, World")
    }
}
