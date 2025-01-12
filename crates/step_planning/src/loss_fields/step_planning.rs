use nalgebra::Scalar;

use crate::{
    geometry::Pose,
    loss_fields::{
        path_distance::PathDistanceField, path_progress::PathProgressField,
        step_size::StepSizeField,
    },
    step_plan::{PlannedStep, Step},
    traits::LossField,
};

pub struct StepPlanningLossField<'a> {
    pub path_distance_field: PathDistanceField<'a>,
    pub path_distance_penalty: f32,
    pub path_progress_field: PathProgressField<'a>,
    pub path_progress_reward: f32,
    pub step_size_field: StepSizeField,
    pub step_size_penalty: f32,
}

pub struct PlannedStepGradient<T: Scalar> {
    pub pose: Pose<T>,
    pub step: Step<T>,
}

impl<'a> LossField for StepPlanningLossField<'a> {
    type Parameter = PlannedStep<f32>;
    type Gradient = PlannedStepGradient<f32>;
    type Loss = f32;

    fn loss(&self, parameter: Self::Parameter) -> Self::Loss {
        let PlannedStep { pose, step } = parameter;

        let distance_loss = self.path_distance_field.loss(pose.position);
        let progress_loss = self.path_progress_field.loss(pose.position);
        let step_size_loss = self.step_size_field.loss(step);

        distance_loss * self.path_distance_penalty
            + progress_loss * self.path_progress_reward
            + step_size_loss * self.step_size_penalty
    }

    fn grad(&self, parameter: Self::Parameter) -> Self::Gradient {
        let PlannedStep { pose, step } = parameter;

        let distance_loss_gradient =
            self.path_distance_field.grad(pose.position) * self.path_distance_penalty;
        let progress_loss_gradient =
            self.path_progress_field.grad(pose.position) * self.path_progress_reward;
        let step_size_loss = self.step_size_field.grad(step) * self.step_size_penalty;

        PlannedStepGradient {
            pose: Pose {
                position: (distance_loss_gradient + progress_loss_gradient).as_point(),
                orientation: 0.0,
            },
            step: step_size_loss,
        }
    }
}
