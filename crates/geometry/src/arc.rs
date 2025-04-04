use approx::{AbsDiffEq, RelativeEq};
use serde::{Deserialize, Serialize};

use linear_algebra::Point2;
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};

use crate::{angle::Angle, circle::Circle, direction::Direction};

#[derive(
    Clone,
    Copy,
    Debug,
    Deserialize,
    PartialEq,
    PathDeserialize,
    PathIntrospect,
    PathSerialize,
    Serialize,
)]
pub struct Arc<Frame> {
    pub circle: Circle<Frame>,
    pub start: Angle<f32>,
    pub end: Angle<f32>,
    pub direction: Direction,
}

impl<Frame> AbsDiffEq for Arc<Frame>
where
    Frame: AbsDiffEq,
{
    type Epsilon = f32;

    fn default_epsilon() -> Self::Epsilon {
        Self::Epsilon::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.direction == other.direction
            && self.circle.abs_diff_eq(&other.circle, epsilon)
            && self.start.abs_diff_eq(&other.start, epsilon)
            && self.end.abs_diff_eq(&other.end, epsilon)
    }
}

impl<Frame> RelativeEq for Arc<Frame>
where
    Frame: RelativeEq,
{
    fn default_max_relative() -> f32 {
        f32::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        self.direction == other.direction
            && self
                .circle
                .relative_eq(&other.circle, epsilon, max_relative)
            && self.start.relative_eq(&other.start, epsilon, max_relative)
            && self.end.relative_eq(&other.end, epsilon, max_relative)
    }
}

impl<Frame: Copy> Arc<Frame> {
    pub fn new(
        circle: Circle<Frame>,
        start: Angle<f32>,
        end: Angle<f32>,
        direction: Direction,
    ) -> Self {
        Self {
            circle,
            start,
            end,
            direction,
        }
    }

    pub fn length(&self) -> f32 {
        let angle = self.start.angle_to(self.end, self.direction);

        angle.0 * self.circle.radius
    }

    pub fn start_point(&self) -> Point2<Frame> {
        self.circle.point_at_angle(self.start)
    }

    pub fn classify_point(&self, point: Point2<Frame>) -> ArcProjectionKind {
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

pub enum ArcProjectionKind {
    OnArc,
    Start,
    End,
}

#[cfg(test)]
mod tests {
    use std::f32::consts::{FRAC_PI_2, PI, TAU};

    use approx::assert_relative_eq;

    use linear_algebra::point;

    use super::*;

    #[derive(Clone, Copy)]
    struct SomeFrame;

    #[test]
    fn arc_cost_90_degrees() {
        let arc = Arc::<SomeFrame> {
            circle: Circle {
                center: point![1.0, 1.0],
                radius: 2.0,
            },
            start: Angle::new(FRAC_PI_2),
            end: Angle::new(0.0),
            direction: Direction::Clockwise,
        };
        assert_relative_eq!(arc.length(), PI);

        let arc = Arc::<SomeFrame> {
            circle: Circle {
                center: point![1.0, 1.0],
                radius: 2.0,
            },
            start: Angle::new(FRAC_PI_2),
            end: Angle::new(0.0),
            direction: Direction::Counterclockwise,
        };
        assert_relative_eq!(arc.length(), 3.0 * PI);

        let arc = Arc::<SomeFrame> {
            circle: Circle {
                center: point![1.0, 1.0],
                radius: 2.0,
            },
            start: Angle::new(0.0),
            end: Angle::new(FRAC_PI_2),
            direction: Direction::Clockwise,
        };
        assert_relative_eq!(arc.length(), 3.0 * PI);

        let arc = Arc::<SomeFrame> {
            circle: Circle {
                center: point![1.0, 1.0],
                radius: 2.0,
            },
            start: Angle::new(0.0),
            end: Angle::new(FRAC_PI_2),
            direction: Direction::Counterclockwise,
        };
        assert_relative_eq!(arc.length(), PI);
    }

    #[test]
    fn arc_cost_generic() {
        for angle_index in 0..100 {
            let angle = angle_index as f32 / 100.0 * TAU;
            for angle_distance_index in 1..100 {
                let angle_distance = angle_distance_index as f32 / 100.0 * TAU;
                let center = point![PI, 4.20];
                let radius = 5.0;

                println!("angle: {angle} angle_distance {angle_distance}");

                let arc = Arc::<SomeFrame> {
                    circle: Circle { center, radius },
                    start: Angle(angle),
                    end: Angle(angle + angle_distance),
                    direction: Direction::Counterclockwise,
                };
                assert_relative_eq!(arc.length(), radius * angle_distance, epsilon = 0.001);

                let arc = Arc::<SomeFrame> {
                    circle: Circle { center, radius },
                    start: Angle(angle),
                    end: Angle(angle + angle_distance),
                    direction: Direction::Clockwise,
                };
                assert_relative_eq!(
                    arc.length(),
                    radius * (TAU - angle_distance),
                    epsilon = 0.001
                );
            }
        }
    }
}
