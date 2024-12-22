use nalgebra::{Point2, Vector2};

use crate::{
    geometry::Path,
    traits::{LossField, Project},
};

pub struct PathDistanceField<'a> {
    pub path: &'a Path,
}

impl<'a> LossField for PathDistanceField<'a> {
    type Parameter = Point2<f64>;
    type Gradient = Vector2<f64>;
    type Loss = f64;

    fn loss(&self, point: Self::Parameter) -> Self::Loss {
        let projection = self.path.project(point);

        let projection_to_point = point - projection;

        projection_to_point.norm_squared()
    }

    fn grad(&self, point: Self::Parameter) -> Self::Gradient {
        let projection = self.path.project(point);

        let projection_to_point = point - projection;

        projection_to_point * 2.0
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, SQRT_2};

    use approx::assert_abs_diff_eq;
    use nalgebra::{point, vector, Vector2};

    use crate::{
        geometry::{Angle, Arc, Circle, Direction, LineSegment, Path, PathSegment},
        loss_fields::path_distance::PathDistanceField,
        traits::LossField,
    };

    fn test_path() -> Path {
        Path {
            segments: vec![
                PathSegment::LineSegment(LineSegment(point![0.0, 0.0], point![3.0, 0.0])),
                PathSegment::Arc(Arc {
                    circle: Circle {
                        center: point![3.0, 1.0],
                        radius: 1.0,
                    },
                    start: Angle(3.0 * FRAC_PI_2),
                    end: Angle(0.0),
                    direction: Direction::Counterclockwise,
                }),
                PathSegment::LineSegment(LineSegment(point![4.0, 1.0], point![4.0, 4.0])),
            ],
        }
    }

    #[test]
    fn test_path_distance() {
        let loss_field = PathDistanceField { path: &test_path() };

        // Start
        let sample_point_1 = point![0.0, 0.0];
        let loss_1 = loss_field.loss(sample_point_1);
        let grad_1 = loss_field.grad(sample_point_1);

        assert_abs_diff_eq!(loss_1, 0.0);
        assert_abs_diff_eq!(grad_1, Vector2::zeros());

        // Before start
        let sample_point_2 = point![-1.0, 0.0];
        let loss_2 = loss_field.loss(sample_point_2);
        let grad_2 = loss_field.grad(sample_point_2);

        assert_abs_diff_eq!(loss_2, 1.0);
        assert_abs_diff_eq!(grad_2, vector![-2.0, 0.0]);

        // End of first line segment, start of arc
        let sample_point_3 = point![3.0, 0.0];
        let loss_3 = loss_field.loss(sample_point_3);
        let grad_3 = loss_field.grad(sample_point_3);

        assert_abs_diff_eq!(loss_3, 0.0);
        assert_abs_diff_eq!(grad_3, Vector2::zeros());

        // Below start of arc
        let sample_point_4 = point![3.0, -1.0];
        let loss_4 = loss_field.loss(sample_point_4);
        let grad_4 = loss_field.grad(sample_point_4);

        assert_abs_diff_eq!(loss_4, 1.0);
        assert_abs_diff_eq!(grad_4, vector![0.0, -2.0]);

        // End of arc
        let sample_point_5 = point![4.0, 1.0];
        let loss_5 = loss_field.loss(sample_point_5);
        let grad_5 = loss_field.grad(sample_point_5);

        assert_abs_diff_eq!(loss_5, 0.0);
        assert_abs_diff_eq!(grad_5, Vector2::zeros());

        // End
        let sample_point_6 = point![4.0, 4.0];
        let loss_6 = loss_field.loss(sample_point_6);
        let grad_6 = loss_field.grad(sample_point_6);

        assert_abs_diff_eq!(loss_6, 0.0);
        assert_abs_diff_eq!(grad_6, Vector2::zeros());

        // Outside of arc
        let sample_point_7 = point![4.0, 0.0];
        let loss_7 = loss_field.loss(sample_point_7);
        let grad_7 = loss_field.grad(sample_point_7);

        assert_abs_diff_eq!(loss_7, (SQRT_2 - 1.0).powi(2));
        assert_abs_diff_eq!(grad_7, vector![2.0 - SQRT_2, -(2.0 - SQRT_2)]);
    }
}
