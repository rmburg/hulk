use coordinate_systems::Ground;
use geometry::{angle::Angle, circle::Circle, direction::Direction, line_segment::LineSegment};
use linear_algebra::Point2;

#[derive(Debug, Clone)]
pub struct Arc {
    pub circle: Circle<Ground>,
    pub start: Angle<f32>,
    pub end: Angle<f32>,
    pub direction: Direction,
}

pub enum ArcProjectionKind {
    OnArc,
    Start,
    End,
}

impl Arc {
    pub fn classify_point(&self, point: Point2<Ground>) -> ArcProjectionKind {
        let center_to_point = point - self.circle.center;
        let angle = Angle::new(center_to_point.y().atan2(center_to_point.x()));

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

            let squared_distance_to_start = start_to_point.inner.norm_squared();
            let squared_distance_to_end = end_to_point.inner.norm_squared();

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
    LineSegment(LineSegment<Ground>),
    Arc(Arc),
}

#[derive(Clone)]
pub struct Path {
    pub segments: Vec<PathSegment>,
}
