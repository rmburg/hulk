use std::{array, vec::Vec};

use argmin::{
    core::{CostFunction, Error as ArgminError, Executor, Gradient, State},
    solver::{linesearch::MoreThuenteLineSearch, quasinewton::LBFGS},
};
use color_eyre::{eyre::eyre, Result};
use nalgebra::{
    allocator::Allocator, Const, DefaultAllocator, DimName, OMatrix, SVector, ToTypenum,
};
use num_dual::{Derivative, DualNum, DualNumFloat, DualVec};

use step_planning::{
    geometry::{Path, Pose},
    loss_fields::step_size::{WalkVolumeCoefficients, WalkVolumeExtents},
    step_plan::{PlannedStep, StepPlan, StepPlanning},
    traits::{LossField, ScaledGradient, UnwrapDual, WrapDual},
};
use types::support_foot::Side;

const STEPS_TO_PLAN: usize = 15;
const VARIABLES_PER_STEP: usize = 3;
const NUM_VARIABLES: usize = STEPS_TO_PLAN * VARIABLES_PER_STEP;

fn duals<F: DualNumFloat + DualNum<F>, const N: usize>(
    reals: &[F; N],
) -> [DualVec<F, F, Const<N>>; N]
where
    Const<N>: ToTypenum,
{
    array::from_fn(|i| {
        DualVec::new(
            reals[i],
            Derivative::some(SVector::ith_axis(i).into_inner()),
        )
    })
}

#[derive(Clone)]
struct StepPlanningProblem {
    step_planning: StepPlanning,
}

impl CostFunction for StepPlanningProblem {
    type Param = Vec<f32>;

    type Output = f32;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, ArgminError> {
        let step_planning_loss = self.step_planning.loss_field();

        let step_plan = StepPlan::from(param.as_slice());

        let loss = self
            .step_planning
            .planned_steps(
                self.step_planning
                    .initial_pose
                    .clone()
                    .with_support_foot(self.step_planning.initial_support_foot),
                &step_plan,
            )
            .map(|planned_step| step_planning_loss.loss(planned_step))
            .sum();

        Ok(loss)
    }
}

trait UnwrapGradient<G> {
    fn unwrap_gradient(self) -> G;
}

impl<T: DualNum<F>, F, R: DimName, C: DimName> UnwrapGradient<OMatrix<T, R, C>>
    for Derivative<T, F, R, C>
where
    DefaultAllocator: Allocator<T, R, C>,
{
    fn unwrap_gradient(self) -> OMatrix<T, R, C> {
        self.unwrap_generic(R::name(), C::name())
    }
}

impl Gradient for StepPlanningProblem {
    type Param = <Self as CostFunction>::Param;

    type Gradient = Self::Param;

    fn gradient(&self, param: &Self::Param) -> Result<Self::Gradient, ArgminError> {
        let param_array: [f32; NUM_VARIABLES] = param.as_slice().try_into().unwrap();

        let dual_param = duals(&param_array);

        let step_planning_loss = self.step_planning.loss_field();

        let step_plan = StepPlan::from(dual_param.as_slice());

        let gradient: SVector<f32, NUM_VARIABLES> = self
            .step_planning
            .planned_steps(
                self.step_planning
                    .initial_pose
                    .clone()
                    .with_support_foot(self.step_planning.initial_support_foot)
                    .wrap_dual(),
                &step_plan,
            )
            .map(|dual_planned_step| {
                let (planned_step, planned_step_gradients) = dual_planned_step.unwrap_dual();

                let derivatives = step_planning_loss.grad(planned_step);

                planned_step_gradients
                    .scaled_gradient(derivatives)
                    .unwrap_gradient()
            })
            .sum();

        Ok(gradient.as_slice().into())
    }
}

pub fn plan_steps(
    path: Path,
    initial_pose: Pose<f32>,
    initial_support_foot: Side,
) -> Result<Vec<PlannedStep<f32>>> {
    let line_search = MoreThuenteLineSearch::new();
    let solver = LBFGS::new(line_search, 10);

    let problem = StepPlanningProblem {
        step_planning: StepPlanning {
            path,
            initial_pose: initial_pose.clone(),
            initial_support_foot,
            path_progress_reward: 5.0,
            path_distance_penalty: 50.0,
            path_progress_smoothness: 1.0,
            step_size_penalty: 1.0,
            walk_volume_coefficients: WalkVolumeCoefficients::from_extents_and_exponents(
                &WalkVolumeExtents {
                    forward: 0.045,
                    backward: 0.04,
                    outward: 0.1,
                    inward: 0.01,
                    outward_rotation: 1.0,
                    inward_rotation: 1.0,
                },
                1.5,
                2.0,
            ),
        },
    };

    let result = Executor::new(problem.clone(), solver)
        .configure(|state| state.param(vec![0.0; NUM_VARIABLES]))
        .run()
        .map_err(|error| eyre!("Executor failed: {error:?}"))?;

    println!("{result}");

    let best_param = result.state.get_best_param().unwrap();

    let step_plan = StepPlan::from(best_param.as_slice());

    let steps = problem
        .step_planning
        .planned_steps(
            initial_pose.with_support_foot(initial_support_foot),
            &step_plan,
        )
        .collect();

    Ok(steps)
}
