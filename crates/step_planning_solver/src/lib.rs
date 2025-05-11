use argmin::{
    core::{
        CostFunction, Error as ArgminError, Executor, Gradient, TerminationReason::SolverExit,
        TerminationStatus,
    },
    solver::{linesearch::MoreThuenteLineSearch, quasinewton::LBFGS},
};
use color_eyre::{
    eyre::{eyre, OptionExt},
    Result,
};
use nalgebra::{DVector, Dyn, U1};
use num_dual::{Derivative, DualNum, DualNumFloat, DualVec};

use step_planning::{
    geometry::Pose,
    loss_fields::step_size::{WalkVolumeCoefficients, WalkVolumeExtents},
    step_plan::{StepPlan, StepPlanning},
    traits::{LossField, ScaledGradient, UnwrapDual, WrapDual},
};
use types::{planned_path::Path, support_foot::Side};

fn duals<F: DualNumFloat + DualNum<F>>(reals: &DVector<F>) -> DVector<DualVec<F, F, Dyn>> {
    let num_variables = reals.nrows();

    reals.map_with_location(|row, _, real| {
        DualVec::new(
            real,
            Derivative::some(DVector::from_fn(num_variables, |i, _| {
                if i == row {
                    F::one()
                } else {
                    F::zero()
                }
            })),
        )
    })
}

#[derive(Clone)]
struct StepPlanningProblem {
    step_planning: StepPlanning,
}

impl CostFunction for StepPlanningProblem {
    type Param = DVector<f32>;

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

impl Gradient for StepPlanningProblem {
    type Param = <Self as CostFunction>::Param;

    type Gradient = Self::Param;

    fn gradient(&self, param: &Self::Param) -> Result<Self::Gradient, ArgminError> {
        let num_variables = param.nrows();
        let dual_param = duals(param);

        let step_planning_loss = self.step_planning.loss_field();

        let step_plan = StepPlan::from(dual_param.as_slice());

        let gradient: DVector<f32> = self
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
                    .unwrap_generic(Dyn(num_variables), U1)
            })
            .sum();

        Ok(gradient)
    }
}

pub fn plan_steps(
    path: Path,
    initial_pose: Pose<f32>,
    initial_support_foot: Side,
    initial_parameter_guess: DVector<f32>,
) -> Result<DVector<f32>> {
    let line_search = MoreThuenteLineSearch::new();
    let solver = LBFGS::new(line_search, 10);

    let problem = StepPlanningProblem {
        step_planning: StepPlanning {
            path: path.clone(),
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
        .configure(|state| state.param(initial_parameter_guess).max_iters(1000))
        .run()
        .map_err(|error| eyre!("Executor failed: {error:?}"))?;

    if let TerminationStatus::Terminated(SolverExit(reason)) = result.state.termination_status {
        println!("executor failed: {reason:?}");
        dbg!(path.segments, initial_pose, initial_support_foot);
    };

    result
        .state
        .best_param
        .ok_or_eyre("best_param was none. This should not happen")
}
