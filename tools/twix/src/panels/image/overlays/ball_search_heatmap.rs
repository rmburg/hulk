use std::sync::Arc;

use color_eyre::eyre::Result;
use coordinate_systems::Pixel;
use eframe::egui::{Color32, Stroke};
use linear_algebra::Point2;
use types::non_uniform_grid::NonUniformGrid;

use crate::{
    panels::image::{cycler_selector::VisionCycler, overlay::Overlay},
    twix_painter::TwixPainter,
    value_buffer::BufferHandle,
};

type GridType = Option<NonUniformGrid<Option<Point2<Pixel>>>>;

pub struct BallSearchHeatmap {
    ball_search_heatmap_grid_in_pixel: Option<BufferHandle<Option<GridType>>>,
}

impl Overlay for BallSearchHeatmap {
    const NAME: &'static str = "Ball Search Heatmap";

    fn new(nao: Arc<crate::nao::Nao>, selected_cycler: VisionCycler) -> Self {
        let buffer = if selected_cycler == VisionCycler::Top {
            Some(
                nao.subscribe_value("Control.additional_outputs.ball_search_heatmap_grid_in_pixel"),
            )
        } else {
            None
        };

        Self {
            ball_search_heatmap_grid_in_pixel: buffer,
        }
    }

    fn paint(&self, painter: &TwixPainter<Pixel>) -> Result<()> {
        let Some(buffer) = self.ball_search_heatmap_grid_in_pixel.as_ref() else {
            return Ok(());
        };
        let Some(grid) = buffer.get_last_value()?.flatten().flatten() else {
            return Ok(());
        };

        for points in grid.quads().filter_map(|quad| match quad.vertices {
            [Some(v1), Some(v2), Some(v3), Some(v4)] => Some([v1, v2, v3, v4]),
            _ => None,
        }) {
            painter.polygon(points, Stroke::new(2.0, Color32::ORANGE));
        }

        Ok(())
    }
}
