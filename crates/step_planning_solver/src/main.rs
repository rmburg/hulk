use std::{f32::consts::FRAC_PI_2, time::SystemTime};

use color_eyre::Result;

use geometry::{
    angle::Angle, arc::Arc, circle::Circle, direction::Direction, line_segment::LineSegment,
};
use linear_algebra::point;
use nalgebra::DVector;
use step_planning::geometry::Pose;
use step_planning_solver::plan_steps;
use types::{
    planned_path::{Path, PathSegment},
    support_foot::Side,
};

fn main() -> Result<()> {
    color_eyre::install()?;

    let path = Path {
        segments: vec![
            PathSegment::LineSegment(LineSegment(point![0.0, 0.0], point![0.3, 0.0])),
            PathSegment::Arc(Arc {
                circle: Circle {
                    center: point![0.3, 0.1],
                    radius: 0.1,
                },
                start: Angle(3.0 * FRAC_PI_2),
                end: Angle(0.0),
                direction: Direction::Counterclockwise,
            }),
            PathSegment::LineSegment(LineSegment(point![0.4, 0.1], point![0.4, 0.4])),
        ],
    };

    let initial_pose = Pose {
        position: point![0.0, 0.0],
        orientation: 0.0,
    };
    let initial_support_foot = Side::Left;

    let earlier = SystemTime::now();

    let _raw_step_plan = plan_steps(path, initial_pose, initial_support_foot, DVector::zeros(30))?;

    let elapsed = SystemTime::now().duration_since(earlier).unwrap();

    println!("Took {elapsed:?}");
    // let step_plan = StepPlan::from(raw_step_plan.as_slice());

    // for planned_step in step_plan.steps() {
    //     dbg!(planned_step);
    // }

    Ok(())
}
