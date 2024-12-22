use std::{
    fmt::Debug,
    ops::{Add, AddAssign},
};

use nalgebra::{vector, Point2, RealField, Rotation2, Scalar};
use num_traits::Euclid;

use crate::step_plan::Step;

#[derive(Clone, Debug)]
pub struct Pose<T: Scalar> {
    pub position: Point2<T>,
    pub orientation: T,
}

impl<T: RealField + Euclid> PartialEq for Pose<T> {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position && self.orientation == other.orientation
    }
}

impl<T: Scalar> Pose<T> {
    pub fn with_support_foot(self, support_foot: Side) -> PoseAndSupportFoot<T> {
        PoseAndSupportFoot {
            pose: self,
            support_foot,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    #[must_use]
    pub fn opposite(&self) -> Self {
        match self {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
        }
    }

    pub fn flip(&mut self) {
        *self = self.opposite();
    }
}

#[derive(Clone, Debug)]
pub struct PoseAndSupportFoot<T: Scalar> {
    pub pose: Pose<T>,
    pub support_foot: Side,
}

impl<T: RealField> Add<Step<T>> for Pose<T> {
    type Output = Self;

    fn add(self, step: Step<T>) -> Self::Output {
        let Self {
            position,
            orientation,
        } = self;
        let Step {
            forward,
            left,
            turn,
        } = step;

        Self {
            position: position + (Rotation2::new(orientation.clone()) * vector![forward, left]),
            orientation: orientation + turn,
        }
    }
}

impl<T: RealField> AddAssign<Step<T>> for Pose<T> {
    fn add_assign(&mut self, step: Step<T>) {
        *self = self.clone() + step;
    }
}
