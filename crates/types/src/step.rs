use std::ops::{Add, Mul, Sub};

use approx::AbsDiffEq;
use nalgebra::RealField;
use num_traits::Euclid;
use serde::{Deserialize, Serialize};

use geometry::angle::Angle;
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};

use crate::support_foot::Side;

#[derive(
    Clone,
    Copy,
    Debug,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    Default,
)]
pub struct Step<T = f32> {
    pub forward: T,
    pub left: T,
    pub turn: T,
}

impl Step {
    pub const ZERO: Self = Self {
        forward: 0.0,
        left: 0.0,
        turn: 0.0,
    };

    pub fn mirrored(self) -> Self {
        Self {
            forward: self.forward,
            left: -self.left,
            turn: -self.turn,
        }
    }
}

impl<T: Clone> Step<T> {
    pub fn from_slice(slice: &[T]) -> Self {
        let [forward, left, turn]: &[T; 3] = slice.try_into().unwrap();

        Self {
            forward: forward.clone(),
            left: left.clone(),
            turn: turn.clone(),
        }
    }
}

impl<T: Mul<Output = T> + Clone> Mul<T> for Step<T> {
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            forward: self.forward * rhs.clone(),
            left: self.left * rhs.clone(),
            turn: self.turn * rhs,
        }
    }
}

impl<T: RealField + Euclid> PartialEq for Step<T> {
    fn eq(&self, other: &Self) -> bool {
        self.forward.eq(&other.forward) && self.left.eq(&other.left) && self.turn.eq(&other.turn)
    }
}

impl<T: AbsDiffEq + RealField + Euclid> AbsDiffEq for Step<T>
where
    T::Epsilon: Copy,
{
    type Epsilon = T::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        T::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.forward.abs_diff_eq(&other.forward, epsilon)
            && self.left.abs_diff_eq(&other.left, epsilon)
            && Angle(self.turn).abs_diff_eq(&Angle(other.turn), epsilon)
    }
}

impl Add for Step {
    type Output = Step;

    fn add(self, right: Step) -> Self::Output {
        Self {
            forward: self.forward + right.forward,
            left: self.left + right.left,
            turn: self.turn + right.turn,
        }
    }
}

impl Sub<Step> for Step {
    type Output = Step;

    fn sub(self, right: Step) -> Self::Output {
        Self {
            forward: self.forward - right.forward,
            left: self.left - right.left,
            turn: self.turn - right.turn,
        }
    }
}

#[derive(Clone, Debug)]
pub struct StepAndSupportFoot<T> {
    pub step: Step<T>,
    pub support_foot: Side,
}
