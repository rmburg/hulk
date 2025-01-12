use coordinate_systems::Ground;
use geometry::angle::Angle;
use linear_algebra::{Point2, Vector2};

use crate::{
    geometry::{path::ArcProjectionKind, Arc, LineSegment, Path, PathSegment},
    traits::{Length, Project},
};

pub trait PathProgress {
    fn progress(&self, point: Point2<Ground>) -> f32;
    fn forward(&self, point: Point2<Ground>) -> Vector2<Ground>;
}

impl PathProgress for LineSegment {
    fn progress(&self, point: Point2<Ground>) -> f32 {
        let Self(start, end) = self;
        let start_to_point = point - start;
        let start_to_end = end - start;

        start_to_point.dot(&start_to_end.normalize())
    }

    fn forward(&self, _point: Point2<Ground>) -> Vector2<Ground> {
        let Self(start, end) = self;
        let start_to_end = end - start;

        start_to_end.normalize()
    }
}

impl PathProgress for Arc {
    fn progress(&self, point: Point2<Ground>) -> f32 {
        match self.classify_point(point) {
            ArcProjectionKind::OnArc => {
                let center_to_point = point - self.circle.center;
                let angle = Angle::new(center_to_point.y().atan2(center_to_point.x()));
                let angle_to_point = self.start.angle_to(angle, self.direction);

                self.circle.radius * angle_to_point.into_inner()
            }
            ArcProjectionKind::Start => {
                let start_point = self.circle.point_at_angle(self.start);
                let start_to_point = point - start_point;

                let forward = self.circle.tangent(self.start, self.direction);

                start_to_point.dot(&forward)
            }
            ArcProjectionKind::End => {
                let end_point = self.circle.point_at_angle(self.end);
                let end_to_point = point - end_point;

                let forward = self.circle.tangent(self.end, self.direction);

                self.length() + end_to_point.dot(&forward)
            }
        }
    }

    fn forward(&self, point: Point2<Ground>) -> Vector2<Ground> {
        match self.classify_point(point) {
            ArcProjectionKind::OnArc => {
                let center_to_point = point - self.circle.center;
                let distance_to_center = center_to_point.norm();
                let angle = Angle::new(center_to_point.y().atan2(center_to_point.x()));
                let forward_scale = self.circle.radius / distance_to_center;

                self.circle.tangent(angle, self.direction) * forward_scale
            }
            ArcProjectionKind::Start => self.circle.tangent(self.start, self.direction),
            ArcProjectionKind::End => self.circle.tangent(self.end, self.direction),
        }
    }
}

impl PathProgress for PathSegment {
    fn progress(&self, point: Point2<Ground>) -> f32 {
        match self {
            PathSegment::LineSegment(line_segment) => line_segment.progress(point),
            PathSegment::Arc(arc) => arc.progress(point),
        }
    }

    fn forward(&self, point: Point2<Ground>) -> Vector2<Ground> {
        match self {
            PathSegment::LineSegment(line_segment) => line_segment.forward(point),
            PathSegment::Arc(arc) => arc.forward(point),
        }
    }
}

impl PathProgress for Path {
    fn progress(&self, point: Point2<Ground>) -> f32 {
        let (progress_before_segment_start, segment, _) = self
            .segments
            .iter()
            .scan(0.0, |progress, segment| {
                let old_progress = *progress;
                *progress += segment.length();

                let projection = segment.project(point);
                let squared_distance = (projection - point).norm_squared();

                Some((old_progress, segment, squared_distance))
            })
            .min_by(|(_, _, squared_distance_1), (_, _, squared_distance_2)| {
                squared_distance_1.total_cmp(squared_distance_2)
            })
            .expect("Path was empty");

        progress_before_segment_start + segment.progress(point)
    }

    fn forward(&self, point: Point2<Ground>) -> Vector2<Ground> {
        let (segment, _) = self
            .segments
            .iter()
            .map(|segment| {
                let projection = segment.project(point);
                let squared_distance = (projection - point).norm_squared();

                (segment, squared_distance)
            })
            .min_by(|(_, squared_distance_1), (_, squared_distance_2)| {
                squared_distance_1.total_cmp(squared_distance_2)
            })
            .expect("Path was empty");

        segment.forward(point)
    }
}
