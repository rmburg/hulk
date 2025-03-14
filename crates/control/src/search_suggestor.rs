use std::ops::{Index, IndexMut};

use color_eyre::Result;
use context_attribute::context;
use coordinate_systems::{Field, Ground, Pixel};
use framework::{AdditionalOutput, MainOutput, PerceptionInput};
use itertools::Itertools;
use linear_algebra::{point, Isometry2, Point2};
use nalgebra::clamp;
use ndarray::Array2;
use projection::{camera_matrices::CameraMatrices, camera_matrix::CameraMatrix, Projection};
use serde::{Deserialize, Serialize};
use spl_network_messages::{SubState, Team};
use types::{
    ball_position::{BallPosition, HypotheticalBallPosition},
    field_dimensions::{FieldDimensions, Half, Side},
    filtered_game_controller_state::FilteredGameControllerState,
    image_segments::ImageSegments,
    non_uniform_grid::NonUniformGrid,
    parameters::SearchSuggestorParameters,
    primary_state::PrimaryState,
};

#[derive(Deserialize, Serialize)]
pub struct SearchSuggestor {
    heatmap: Heatmap,
}

#[context]
pub struct CreationContext {
    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    search_suggestor_configuration: Parameter<SearchSuggestorParameters, "search_suggestor">,
}

#[context]
pub struct CycleContext {
    search_suggestor_configuration: Parameter<SearchSuggestorParameters, "search_suggestor">,
    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,

    ball_position: Input<Option<BallPosition<Ground>>, "ball_position?">,
    hypothetical_ball_positions:
        Input<Vec<HypotheticalBallPosition<Ground>>, "hypothetical_ball_positions">,
    ground_to_field: Input<Option<Isometry2<Ground, Field>>, "ground_to_field?">,
    primary_state: Input<PrimaryState, "primary_state">,
    filtered_game_controller_state:
        Input<Option<FilteredGameControllerState>, "filtered_game_controller_state?">,

    heatmap: AdditionalOutput<Array2<f32>, "ball_search_heatmap">,
    image_segments_top: PerceptionInput<ImageSegments, "VisionTop", "image_segments">,
    camera_matrix_top: PerceptionInput<CameraMatrix, "VisionTop", "camera_matrix">,
    // image_segments_bottom: PerceptionInput<ImageSegments, "VisionBottom", "image_segments">,

    // historic_camera_matrices: HistoricInput<Option<CameraMatrices>, "camera_matrices?">,
    camera_matrices: Input<Option<CameraMatrices>, "camera_matrices?">,

    heatmap_grid_in_pixel: AdditionalOutput<
        Option<NonUniformGrid<Option<Point2<Pixel>>>>,
        "ball_search_heatmap_grid_in_pixel",
    >,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub suggested_search_position: MainOutput<Option<Point2<Field>>>,
}

impl SearchSuggestor {
    pub fn new(context: CreationContext) -> Result<Self> {
        let (heatmap_length, heatmap_width) = (
            (context.field_dimensions.length
                * context.search_suggestor_configuration.cells_per_meter)
                .round() as usize,
            (context.field_dimensions.width
                * context.search_suggestor_configuration.cells_per_meter)
                .round() as usize,
        );
        let heatmap = Heatmap {
            map: Array2::zeros((heatmap_length, heatmap_width)),
            field_dimensions: *context.field_dimensions,
            cells_per_meter: context.search_suggestor_configuration.cells_per_meter,
        };

        Ok(Self { heatmap })
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        self.update_heatmap2(&context);
        self.update_heatmap(&context);
        let suggested_search_position = self
            .heatmap
            .get_maximum_position(context.search_suggestor_configuration.minimum_validity);

        context
            .heatmap
            .fill_if_subscribed(|| self.heatmap.map.clone());

        if let (Some(camera_matrices), Some(ground_to_field)) =
            (context.camera_matrices, context.ground_to_field)
        {
            let grid_in_pixel =
                self.project_heatmap_grid_to_pixel(&context, &camera_matrices.top, ground_to_field);

            context
                .heatmap_grid_in_pixel
                .fill_if_subscribed(|| Some(grid_in_pixel));
        } else {
            context.heatmap_grid_in_pixel.fill_if_subscribed(|| None);
        }

        Ok(MainOutputs {
            suggested_search_position: suggested_search_position.into(),
        })
    }

    fn project_heatmap_grid_to_pixel(
        &mut self,
        context: &CycleContext,
        camera_matrix: &CameraMatrix,
        ground_to_field: &Isometry2<Ground, Field>,
    ) -> NonUniformGrid<Option<Point2<Pixel>>> {
        let (heatmap_length, heatmap_width) = (
            (context.field_dimensions.length
                * context.search_suggestor_configuration.cells_per_meter)
                .round() as usize,
            (context.field_dimensions.width
                * context.search_suggestor_configuration.cells_per_meter)
                .round() as usize,
        );

        let half_field_width = context.field_dimensions.width / 2.0;
        let half_field_length = context.field_dimensions.length / 2.0;

        let grid_in_field = NonUniformGrid::<Point2<Field>>::new_uniform(
            -half_field_length..half_field_length,
            -half_field_width..half_field_width,
            heatmap_length,
            heatmap_width,
        );

        let field_to_ground = ground_to_field.inverse();

        grid_in_field.map(|point| {
            let point_in_ground = field_to_ground * point;

            camera_matrix.ground_to_pixel(point_in_ground).ok()
        })
    }

    fn update_heatmap(&mut self, context: &CycleContext) {
        if let Some(ball_position) = context.ball_position {
            if let Some(ground_to_field) = context.ground_to_field {
                self.heatmap[ground_to_field * ball_position.position] = 1.0;
            }
        }
        for ball_hypothesis in context.hypothetical_ball_positions {
            if let Some(ground_to_field) = context.ground_to_field {
                let ball_hypothesis_position = ground_to_field * ball_hypothesis.position;
                self.heatmap[ball_hypothesis_position] =
                    (self.heatmap[ball_hypothesis_position] + ball_hypothesis.validity) / 2.0;
            }
        }
        if let Some(filtered_game_controller_state) = context.filtered_game_controller_state {
            for rule_ball_hypothesis in get_rule_hypotheses(
                *context.primary_state,
                filtered_game_controller_state,
                *context.field_dimensions,
            ) {
                self.heatmap[rule_ball_hypothesis] = 1.0;
            }
        }

        self.heatmap.map *= 1.0 - context.search_suggestor_configuration.heatmap_decay_factor;
    }

    fn update_heatmap2(&mut self, context: &CycleContext) {
        let Some((camera_matrix_time, camera_matrix_buffer)) =
            context.camera_matrix_top.persistent.last_key_value()
        else {
            return;
        };
        let Some((image_segments_time, image_segments_buffer)) =
            context.image_segments_top.persistent.last_key_value()
        else {
            return;
        };
    }
}

// // TODO deduplicate with twix
// struct SegmentInGround {
//     start: Point2<Ground>,
//     end: Point2<Ground>,
//     line_width: f32,
// }

// // TODO deduplicate with twix
// fn project_segment_to_ground(
//     x: f32,
//     segment: &types::image_segments::Segment,
//     camera_matrix: &CameraMatrix,
// ) -> Result<SegmentInGround> {
//     let start = point![x, segment.start as f32];
//     let end = point![x, segment.end as f32];

//     let start_in_ground = camera_matrix.pixel_to_ground(start)?;
//     let end_in_ground = camera_matrix.pixel_to_ground(end)?;

//     let midpoint = center(start, end);
//     let pixel_radius = 100.0 * camera_matrix.get_pixel_radius(0.01, midpoint)?;
//     let line_width = 3.0 / pixel_radius;

//     Ok(SegmentInGround {
//         start: start_in_ground,
//         end: end_in_ground,
//         line_width,
//     })
// }

#[derive(Deserialize, Serialize)]
struct Heatmap {
    map: Array2<f32>,
    field_dimensions: FieldDimensions,
    cells_per_meter: f32,
}

impl Heatmap {
    fn field_to_heatmap(&self, field_point: Point2<Field>) -> (usize, usize) {
        let heatmap_point = (
            ((field_point.x() + self.field_dimensions.length / 2.0) * self.cells_per_meter)
                as usize,
            ((field_point.y() + self.field_dimensions.width / 2.0) * self.cells_per_meter) as usize,
        );
        (
            clamp(heatmap_point.0, 0, self.map.dim().0 - 1),
            clamp(heatmap_point.1, 0, self.map.dim().1 - 1),
        )
    }

    fn get_maximum_position(&self, minimum_validity: f32) -> Option<Point2<Field>> {
        let linear_maximum_heat_heatmap_position =
            self.map.iter().position_max_by(|a, b| a.total_cmp(b))?;
        let maximum_heat_heatmap_position = (
            linear_maximum_heat_heatmap_position / self.map.dim().1,
            linear_maximum_heat_heatmap_position % self.map.dim().1,
        );
        if self.map[maximum_heat_heatmap_position] > minimum_validity {
            let search_suggestion = point![
                ((maximum_heat_heatmap_position.0 as f32 + 1.0 / 2.0) / self.cells_per_meter
                    - self.field_dimensions.length / 2.0),
                ((maximum_heat_heatmap_position.1 as f32 + 1.0 / 2.0) / self.cells_per_meter
                    - self.field_dimensions.width / 2.0)
            ];
            return Some(search_suggestion);
        }
        None
    }
}

impl Index<Point2<Field>> for Heatmap {
    type Output = f32;
    fn index(&self, field_point: Point2<Field>) -> &Self::Output {
        let heatmap_point = self.field_to_heatmap(field_point);
        &self.map[heatmap_point]
    }
}

impl IndexMut<Point2<Field>> for Heatmap {
    fn index_mut(&mut self, field_point: Point2<Field>) -> &mut Self::Output {
        let heatmap_point = self.field_to_heatmap(field_point);
        &mut self.map[heatmap_point]
    }
}

fn get_rule_hypotheses(
    primary_state: PrimaryState,
    filtered_game_controller_state: &FilteredGameControllerState,
    field_dimensions: FieldDimensions,
) -> Vec<Point2<Field>> {
    let kicking_team_half = kicking_team_half(filtered_game_controller_state.kicking_team);

    match (primary_state, filtered_game_controller_state.sub_state) {
        (PrimaryState::Ready, Some(SubState::PenaltyKick)) => {
            let kicking_team_half = kicking_team_half.unwrap_or(Half::Own).mirror();
            vec![field_dimensions.penalty_spot(kicking_team_half)]
        }
        // Kick-off
        (PrimaryState::Ready, None) => vec![field_dimensions.center()],
        (PrimaryState::Playing, Some(SubState::CornerKick)) => {
            if let Some(kicking_team_half) = kicking_team_half {
                let kicking_team_half = kicking_team_half.mirror();
                vec![
                    field_dimensions.corner(kicking_team_half, Side::Left),
                    field_dimensions.corner(kicking_team_half, Side::Right),
                ]
            } else {
                vec![
                    field_dimensions.corner(Half::Own, Side::Left),
                    field_dimensions.corner(Half::Opponent, Side::Left),
                    field_dimensions.corner(Half::Own, Side::Right),
                    field_dimensions.corner(Half::Opponent, Side::Right),
                ]
            }
        }
        (PrimaryState::Playing, Some(SubState::GoalKick)) => {
            if let Some(kicking_team_half) = kicking_team_half {
                vec![
                    field_dimensions.goal_box_corner(kicking_team_half, Side::Left),
                    field_dimensions.goal_box_corner(kicking_team_half, Side::Right),
                ]
            } else {
                vec![
                    field_dimensions.goal_box_corner(Half::Own, Side::Left),
                    field_dimensions.goal_box_corner(Half::Opponent, Side::Left),
                    field_dimensions.goal_box_corner(Half::Own, Side::Right),
                    field_dimensions.goal_box_corner(Half::Opponent, Side::Right),
                ]
            }
        }
        (_, _) => Vec::new(),
    }
}

fn kicking_team_half(kicking_team: Option<Team>) -> Option<Half> {
    match kicking_team {
        Some(Team::Opponent) => Some(Half::Opponent),
        Some(Team::Hulks) => Some(Half::Own),
        None => None,
    }
}
