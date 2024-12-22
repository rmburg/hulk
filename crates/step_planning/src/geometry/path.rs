use std::ops::Neg;

use nalgebra::{vector, Point2, RealField, Vector2};
use num_traits::One;

use crate::geometry::Angle;

#[derive(Clone)]
pub struct LineSegment(pub Point2<f64>, pub Point2<f64>);

#[derive(Clone, Copy, Debug)]
pub enum Direction {
    Clockwise,
    Counterclockwise,
}

impl Direction {
    pub fn angle_sign<T: One + Neg<Output = T>>(self) -> T {
        match self {
            Direction::Clockwise => -T::one(),
            Direction::Counterclockwise => T::one(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Circle {
    pub center: Point2<f64>,
    pub radius: f64,
}

impl Circle {
    pub fn point_at_angle(&self, angle: Angle<f64>) -> Point2<f64> {
        self.center + angle.as_direction() * self.radius
    }

    pub fn circumference(&self) -> f64 {
        f64::two_pi() * self.radius
    }

    pub fn tangent(&self, angle: Angle<f64>, direction: Direction) -> Vector2<f64> {
        let radius = angle.as_direction();

        match direction {
            Direction::Clockwise => {
                vector![radius.y, -radius.x]
            }
            Direction::Counterclockwise => {
                vector![-radius.y, radius.x]
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct Arc {
    pub circle: Circle,
    pub start: Angle<f64>,
    pub end: Angle<f64>,
    pub direction: Direction,
}

pub enum ArcProjectionKind {
    OnArc,
    Start,
    End,
}

impl Arc {
    pub fn classify_point(&self, point: Point2<f64>) -> ArcProjectionKind {
        let center_to_point = point - self.circle.center;
        let angle = Angle::new(center_to_point.y.atan2(center_to_point.x));

        let angle_to_end = self.start.angle_to(self.end, self.direction);
        let angle_to_point = self.start.angle_to(angle, self.direction);

        let is_between_start_and_end = angle_to_point.into_inner() < angle_to_end.into_inner();

        if is_between_start_and_end {
            ArcProjectionKind::OnArc
        } else {
            let start_point = self.circle.point_at_angle(self.start);
            let end_point = self.circle.point_at_angle(self.end);

            let start_to_point = point - start_point;
            let end_to_point = point - end_point;

            let squared_distance_to_start = start_to_point.norm_squared();
            let squared_distance_to_end = end_to_point.norm_squared();

            if squared_distance_to_start < squared_distance_to_end {
                ArcProjectionKind::Start
            } else {
                ArcProjectionKind::End
            }
        }
    }
}

#[derive(Clone)]
pub enum PathSegment {
    LineSegment(LineSegment),
    Arc(Arc),
}

#[derive(Clone)]
pub struct Path {
    pub segments: Vec<PathSegment>,
}
