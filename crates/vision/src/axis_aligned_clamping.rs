use coordinate_systems::Pixel;
use geometry::{line::Line2, line_segment::LineSegment, rectangle::Rectangle};
use linear_algebra::Point2;

#[derive(Clone, Copy)]
pub enum Axis {
    X,
    Y,
}

#[derive(Clone, Copy)]
pub enum ClampDirection {
    /// Everything above is valid
    Min,
    /// Everything below is valid
    Max,
}

fn is_outside(point: Point2<Pixel>, axis: Axis, direction: ClampDirection, value: f32) -> bool {
    match (axis, direction) {
        (Axis::X, ClampDirection::Max) => point.x() > value,
        (Axis::Y, ClampDirection::Max) => point.y() > value,
        (Axis::X, ClampDirection::Min) => point.x() < value,
        (Axis::Y, ClampDirection::Min) => point.y() < value,
    }
}

fn line_axis_intersection(axis: Axis, value: f32, line: Line2<Pixel>) -> Point2<Pixel> {
    let direction = line.1 - line.0;

    let t = match axis {
        Axis::X => (value - line.0.x()) / direction.x(),
        Axis::Y => (value - line.0.y()) / direction.y(),
    };

    line.0 + direction * t
}

pub trait AxisAlignedClamping
where
    Self: std::marker::Sized,
{
    /// Clamp `self` to a line at `value` parallel to `axis`
    fn clamp_to_axis(self, axis: Axis, direction: ClampDirection, value: f32) -> Option<Self>;

    fn clamp_to_rect(self, rect: Rectangle<Pixel>) -> Option<Self> {
        self.clamp_to_axis_range(Axis::X, rect.min.x(), rect.max.x())?
            .clamp_to_axis_range(Axis::Y, rect.min.y(), rect.max.y())
    }

    fn clamp_to_axis_range(self, axis: Axis, min: f32, max: f32) -> Option<Self> {
        self.clamp_to_axis(axis, ClampDirection::Min, min)?
            .clamp_to_axis(axis, ClampDirection::Max, max)
    }
}

impl AxisAlignedClamping for LineSegment<Pixel> {
    fn clamp_to_axis(self, axis: Axis, direction: ClampDirection, value: f32) -> Option<Self> {
        match (
            is_outside(self.0, axis, direction, value),
            is_outside(self.1, axis, direction, value),
        ) {
            (true, true) => None,
            (true, false) => {
                let intersection = line_axis_intersection(axis, value, self.into());

                Some(LineSegment(intersection, self.1))
            }
            (false, true) => {
                let intersection = line_axis_intersection(axis, value, self.into());

                Some(LineSegment(self.0, intersection))
            }
            (false, false) => Some(self),
        }
    }
}

#[cfg(test)]
mod test {
    use linear_algebra::point;

    use super::*;

    #[test]
    fn test_aa_clamping() {
        let rect = Rectangle::<Pixel> {
            min: point![0.0, 0.0],
            max: point![5.0, 3.0],
        };

        let line_1 = LineSegment(point![1.0, 1.0], point![4.0, 2.0]);
        assert_eq!(Some(line_1), line_1.clamp_to_rect(rect));

        let line_2 = LineSegment(point![1.0, 1.0], point![6.0, 1.0]);
        assert_eq!(
            Some(LineSegment(point![1.0, 1.0], point![5.0, 1.0])),
            line_2.clamp_to_rect(rect)
        );

        let line_3 = LineSegment(point![3.0, -2.0], point![7.0, 1.0]);
        assert_eq!(None, line_3.clamp_to_rect(rect));

        let line_4 = LineSegment(point![1.0, 6.0], point![4.0, -3.0]);
        assert_eq!(
            Some(LineSegment(point![2.0, 3.0], point![3.0, 0.0])),
            line_4.clamp_to_rect(rect)
        );

        let line_5 = LineSegment(point![10.0, -3.0], point![-5.0, 6.0]);
        assert_eq!(
            Some(LineSegment(point![5.0, 0.0], point![0.0, 3.0])),
            line_5.clamp_to_rect(rect)
        );
    }
}
