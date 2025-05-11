use nalgebra::{DVector, Dyn, U1};
use nlopt::{FailState, Nlopt, SuccessState};
use num_dual::{Derivative, DualNum, DualNumFloat, DualVec};

use step_planning::{
    geometry::Pose,
    loss_fields::step_size::{WalkVolumeCoefficients, WalkVolumeExtents},
    step_plan::{StepPlan, StepPlanning},
    traits::{LossField, ScaledGradient, UnwrapDual, WrapDual},
};
use types::{planned_path::Path, support_foot::Side};

fn duals<F: DualNumFloat + DualNum<F>>(reals: &[F]) -> Vec<DualVec<F, F, Dyn>> {
    let num_variables = reals.len();

    reals
        .iter()
        .enumerate()
        .map(|(row, real)| {
            DualVec::new(
                *real,
                Derivative::some(DVector::from_fn(num_variables, |i, _| {
                    if i == row {
                        F::one()
                    } else {
                        F::zero()
                    }
                })),
            )
        })
        .collect()
}

pub fn objective(
    variables: &[f64],
    gradient: Option<&mut [f64]>,
    parameters: &mut StepPlanning,
) -> f64 {
    let variables = variables.iter().map(|&x| x as f32).collect::<Vec<_>>();

    let step_planning_loss = parameters.loss_field();

    let step_plan = StepPlan::from(variables.as_slice());

    let loss = if let Some(gradient_out) = gradient {
        let num_variables = variables.len();

        let dual_variables = duals(variables.as_slice());

        let step_plan = StepPlan::from(dual_variables.as_slice());

        let (loss, gradient) = parameters
            .planned_steps(
                parameters
                    .initial_pose
                    .clone()
                    .with_support_foot(parameters.initial_support_foot)
                    .wrap_dual(),
                &step_plan,
            )
            .map(|dual_planned_step| {
                let (planned_step, planned_step_gradients) = dual_planned_step.unwrap_dual();

                // FIXME don't compute the loss twice
                let loss = step_planning_loss.loss(planned_step.clone());

                let derivatives = step_planning_loss.grad(planned_step);
                let gradient = planned_step_gradients
                    .scaled_gradient(derivatives)
                    .unwrap_generic(Dyn(num_variables), U1);

                (loss, gradient.cast())
            })
            .fold(
                (0.0, DVector::zeros(num_variables)),
                |(loss_acc, grad_acc), (loss, grad)| (loss_acc + loss, grad_acc + grad),
            );

        gradient_out.copy_from_slice(gradient.as_slice());

        loss
    } else {
        parameters
            .planned_steps(
                parameters
                    .initial_pose
                    .clone()
                    .with_support_foot(parameters.initial_support_foot),
                &step_plan,
            )
            .map(|planned_step| step_planning_loss.loss(planned_step))
            .sum()
    };

    loss as f64
}

pub fn plan_steps(
    path: Path,
    initial_pose: Pose<f32>,
    initial_support_foot: Side,
    initial_parameter_guess: &mut [f64],
) -> Result<(SuccessState, f64), (FailState, f64)> {
    let problem = StepPlanning {
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
    };

    let nlopt = Nlopt::new(
        nlopt::Algorithm::Lbfgs,
        15,
        objective,
        nlopt::Target::Minimize,
        problem,
    );

    nlopt.optimize(initial_parameter_guess)
}
