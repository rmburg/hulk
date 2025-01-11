use std::ops::Mul;

use nalgebra::{RealField, Scalar};
use num_traits::Euclid;

use geometry::angle::Angle;
use types::support_foot::Side;

use crate::{
    geometry::{pose::PoseAndSupportFoot, Path, Pose},
    loss_fields::{
        path_distance::PathDistanceField,
        path_progress::PathProgressField,
        step_planning::StepPlanningLossField,
        step_size::{StepAndSupportFoot, StepSizeField, WalkVolumeCoefficients},
    },
};

pub struct StepPlan<'a, T>(&'a [T]);

impl<'a, T> From<&'a [T]> for StepPlan<'a, T> {
    fn from(value: &'a [T]) -> Self {
        assert!(value.len() % 3 == 0);

        Self(value)
    }
}

impl<'a, T: RealField> StepPlan<'a, T> {
    pub fn steps(&self) -> impl Iterator<Item = Step<T>> + 'a {
        self.0.chunks_exact(3).map(Step::from_slice)
    }
}

#[derive(Clone)]
pub struct StepPlanning {
    pub path: Path,
    pub initial_pose: Pose<f32>,
    pub initial_support_foot: Side,
    pub path_progress_smoothness: f32,
    pub path_progress_reward: f32,
    pub path_distance_penalty: f32,
    pub step_size_penalty: f32,
    pub walk_volume_coefficients: WalkVolumeCoefficients,
}

impl StepPlanning {
    pub fn planned_steps<'a, T: RealField>(
        &self,
        initial_pose: PoseAndSupportFoot<T>,
        step_plan: &StepPlan<'a, T>,
    ) -> impl Iterator<Item = PlannedStep<T>> + 'a {
        step_plan.steps().scan(initial_pose, |pose, step| {
            pose.pose += step.clone();

            let planned_step = PlannedStep {
                pose: pose.pose.clone(),
                step: step.with_support_foot(pose.support_foot),
            };

            pose.support_foot = pose.support_foot.opposite();

            Some(planned_step)
        })
    }

    pub fn loss_field(&self) -> StepPlanningLossField {
        StepPlanningLossField {
            path_distance_field: PathDistanceField { path: &self.path },
            path_progress_field: PathProgressField {
                path: &self.path,
                smoothness: self.path_progress_smoothness,
            },
            path_distance_penalty: self.path_distance_penalty,
            path_progress_reward: self.path_progress_reward,
            step_size_field: StepSizeField {
                walk_volume_coefficients: self.walk_volume_coefficients.clone(),
            },
            step_size_penalty: self.step_size_penalty,
        }
    }
}

#[derive(Debug)]
pub struct PlannedStep<T: Scalar> {
    /// Pose reached after this step
    pub pose: Pose<T>,
    pub step: StepAndSupportFoot<T>,
}

#[derive(Clone, Debug)]
pub struct Step<T> {
    pub forward: T,
    pub left: T,
    pub turn: T,
}

impl<T: Clone> Step<T> {
    pub fn from_slice(slice: &[T]) -> Self {
        let [forward, left, turn]: &[T; 3] = slice.try_into().unwrap();

        Self {
            forward: forward.clone(),
            left: left.clone(),
            turn: turn.clone(),
        }
    }
}

impl<T: Mul<Output = T> + Clone> Mul<T> for Step<T> {
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            forward: self.forward * rhs.clone(),
            left: self.left * rhs.clone(),
            turn: self.turn * rhs,
        }
    }
}

impl<T: RealField + Euclid> PartialEq for Step<T> {
    fn eq(&self, other: &Self) -> bool {
        self.forward.eq(&other.forward) && self.left.eq(&other.left) && self.turn.eq(&other.turn)
    }
}

impl<T: approx::AbsDiffEq + RealField + Euclid> approx::AbsDiffEq for Step<T>
where
    T::Epsilon: Copy,
{
    type Epsilon = T::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        T::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.forward.abs_diff_eq(&other.forward, epsilon)
            && self.left.abs_diff_eq(&other.left, epsilon)
            && Angle(self.turn).abs_diff_eq(&Angle(other.turn), epsilon)
    }
}

impl<T> Step<T> {
    pub fn with_support_foot(self, support_foot: Side) -> StepAndSupportFoot<T> {
        StepAndSupportFoot {
            step: self,
            support_foot,
        }
    }
}

impl<T: RealField> Step<T> {
    pub fn zero() -> Self {
        Self {
            forward: T::zero(),
            left: T::zero(),
            turn: T::zero(),
        }
    }
}

#[cfg(test)]
mod tests {
    use linear_algebra::{point, Point2};

    use crate::{geometry::Pose, step_plan::Step};

    #[test]
    fn test_pose_step_addition() {
        let pose = Pose {
            position: Point2::origin(),
            orientation: 0.0,
        };
        let step = Step {
            forward: 2.0,
            left: 1.0,
            turn: 3.0,
        };
        let new_pose = pose + step;
        assert_eq!(
            new_pose,
            Pose {
                position: point![2.0, 1.0],
                orientation: 3.0
            }
        );
    }
}
