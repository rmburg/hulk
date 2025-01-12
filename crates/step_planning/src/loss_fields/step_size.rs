use types::support_foot::Side;

use crate::{step_plan::Step, traits::LossField};

pub struct StepSizeField {
    pub walk_volume_coefficients: WalkVolumeCoefficients,
}

#[derive(Clone)]
pub struct WalkVolumeCoefficients {
    pub forward_cost: f32,
    pub backward_cost: f32,
    pub outward_cost: f32,
    pub inward_cost: f32,
    pub outward_rotation_cost: f32,
    pub inward_rotation_cost: f32,
    pub translation_exponent: f32,
    pub rotation_exponent: f32,
}

pub struct WalkVolumeExtents {
    pub forward: f32,
    pub backward: f32,
    pub outward: f32,
    pub inward: f32,
    pub outward_rotation: f32,
    pub inward_rotation: f32,
}

impl WalkVolumeCoefficients {
    pub fn from_extents_and_exponents(
        extents: &WalkVolumeExtents,
        translation_exponent: f32,
        rotation_exponent: f32,
    ) -> Self {
        let WalkVolumeExtents {
            forward,
            backward,
            outward,
            inward,
            outward_rotation,
            inward_rotation,
        } = extents;

        Self {
            forward_cost: 1.0 / forward,
            backward_cost: 1.0 / backward,
            outward_cost: 1.0 / outward,
            inward_cost: 1.0 / inward,
            outward_rotation_cost: 1.0 / outward_rotation,
            inward_rotation_cost: 1.0 / inward_rotation,
            translation_exponent,
            rotation_exponent,
        }
    }
}

impl WalkVolumeCoefficients {
    fn costs(
        &self,
        StepAndSupportFoot { step, support_foot }: &StepAndSupportFoot<f32>,
    ) -> Step<f32> {
        let Self {
            forward_cost: positive_forward_cost,
            backward_cost: negative_forward_cost,
            outward_cost,
            inward_cost,
            outward_rotation_cost,
            inward_rotation_cost,
            translation_exponent: _,
            rotation_exponent: _,
        } = self;

        let Step {
            forward,
            left,
            turn,
        } = step;

        let (
            positive_left_cost,
            negative_left_cost,
            clockwise_rotation_cost,
            counterclockwise_rotation_cost,
        ) = match support_foot {
            Side::Left => (
                inward_cost,
                outward_cost,
                outward_rotation_cost,
                inward_rotation_cost,
            ),
            Side::Right => (
                outward_cost,
                inward_cost,
                inward_rotation_cost,
                outward_rotation_cost,
            ),
        };

        let forward_cost =
            positive_negative(*forward, *positive_forward_cost, *negative_forward_cost);
        let left_cost = positive_negative(*left, *positive_left_cost, *negative_left_cost);
        let turn_cost = positive_negative(
            *turn,
            *clockwise_rotation_cost,
            *counterclockwise_rotation_cost,
        );

        Step {
            forward: forward_cost,
            left: left_cost,
            turn: turn_cost,
        }
    }
}

#[inline]
fn positive_negative(value: f32, positive: f32, negative: f32) -> f32 {
    if value.is_sign_positive() {
        positive
    } else {
        negative
    }
}

fn walk_volume(
    step: &StepAndSupportFoot<f32>,
    walk_volume_coefficients: &WalkVolumeCoefficients,
) -> f32 {
    let costs = walk_volume_coefficients.costs(step);

    let normalized_forward = step.step.forward * costs.forward;
    let normalized_left = step.step.left * costs.left;
    let normalized_turn = step.step.turn * costs.turn;

    (normalized_forward
        .abs()
        .powf(walk_volume_coefficients.translation_exponent)
        + normalized_left
            .abs()
            .powf(walk_volume_coefficients.translation_exponent))
    .powf(
        walk_volume_coefficients.rotation_exponent / walk_volume_coefficients.translation_exponent,
    ) + normalized_turn
        .abs()
        .powf(walk_volume_coefficients.rotation_exponent)
}

fn walk_volume_gradient(
    step: &StepAndSupportFoot<f32>,
    walk_volume_coefficients: &WalkVolumeCoefficients,
) -> Step<f32> {
    let costs = walk_volume_coefficients.costs(step);

    let normalized_forward = (step.step.forward * costs.forward).abs();
    let normalized_left = (step.step.left * costs.left).abs();
    let normalized_turn = (step.step.turn * costs.turn).abs();

    let normalized_forward_powf_t =
        normalized_forward.powf(walk_volume_coefficients.translation_exponent);
    let normalized_left_powf_t =
        normalized_left.powf(walk_volume_coefficients.translation_exponent);
    let normalized_turn_powf_r = normalized_turn.powf(walk_volume_coefficients.rotation_exponent);

    let translation_norm = (normalized_forward.powf(walk_volume_coefficients.translation_exponent)
        + normalized_left.powf(walk_volume_coefficients.translation_exponent))
    .powf(
        (walk_volume_coefficients.rotation_exponent
            - walk_volume_coefficients.translation_exponent)
            / walk_volume_coefficients.translation_exponent,
    );

    Step {
        forward: if step.step.forward == 0.0 {
            0.0
        } else {
            walk_volume_coefficients.rotation_exponent
                * costs.forward.powi(2)
                * step.step.forward
                * translation_norm
                * normalized_forward_powf_t
                / normalized_forward.powi(2)
        },
        left: if step.step.left == 0.0 {
            0.0
        } else {
            walk_volume_coefficients.rotation_exponent
                * costs.left.powi(2)
                * step.step.left
                * translation_norm
                * normalized_left_powf_t
                / normalized_left.powi(2)
        },
        turn: if step.step.turn == 0.0 {
            0.0
        } else {
            walk_volume_coefficients.rotation_exponent
                * costs.turn.powi(2)
                * step.step.turn
                * normalized_turn_powf_r
                / normalized_turn.powi(2)
        },
    }
}

fn penalty_function(walk_volume_value: f32) -> f32 {
    walk_volume_value.powi(6)
}

fn penalty_function_derivative(walk_volume_value: f32) -> f32 {
    walk_volume_value.powi(5) * 6.0
}

#[derive(Clone, Debug)]
pub struct StepAndSupportFoot<T> {
    pub step: Step<T>,
    pub support_foot: Side,
}

impl LossField for StepSizeField {
    type Parameter = StepAndSupportFoot<f32>;
    type Gradient = Step<f32>;
    type Loss = f32;

    fn loss(&self, step: Self::Parameter) -> Self::Loss {
        let value = walk_volume(&step, &self.walk_volume_coefficients);

        penalty_function(value)
    }

    fn grad(&self, step: Self::Parameter) -> Self::Gradient {
        let value = walk_volume(&step, &self.walk_volume_coefficients);
        let grad = walk_volume_gradient(&step, &self.walk_volume_coefficients);

        grad * penalty_function_derivative(value)
    }
}
