use crate::geometry::{Arc, LineSegment, Path, PathSegment};

pub trait Length {
    fn length(&self) -> f64;
}

impl Length for Path {
    fn length(&self) -> f64 {
        self.segments.iter().map(PathSegment::length).sum()
    }
}

impl Length for PathSegment {
    fn length(&self) -> f64 {
        match self {
            PathSegment::LineSegment(line_segment) => line_segment.length(),
            PathSegment::Arc(arc) => arc.length(),
        }
    }
}

impl Length for LineSegment {
    fn length(&self) -> f64 {
        let Self(start, end) = self;

        (*end - *start).norm()
    }
}

impl Length for Arc {
    fn length(&self) -> f64 {
        let angle_start_to_end = self.start.angle_to(self.end, self.direction);

        self.circle.radius * angle_start_to_end.into_inner()
    }
}
