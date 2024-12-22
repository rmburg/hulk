use std::ops::{Add, Div, Mul, Sub};

use nalgebra::{vector, RealField, Rotation2, Vector2};
use num_traits::Euclid;

use crate::geometry::path::Direction;

#[derive(Clone, Copy, Debug)]
pub struct Angle<T>(pub T);

impl<T> Angle<T> {
    pub fn new(value: T) -> Self {
        Self(value)
    }

    pub fn into_inner(self) -> T {
        self.0
    }
}

impl<T: Euclid + RealField> PartialEq for Angle<T> {
    fn eq(&self, other: &Self) -> bool {
        self.normalized().0 == other.normalized().0
    }
}

#[cfg(test)]
impl<T: Euclid + RealField + Clone> approx::AbsDiffEq for Angle<T> {
    type Epsilon = T::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        T::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        let difference = (self.clone() - other.clone()).normalized().into_inner();

        difference.abs_diff_eq(&T::zero(), epsilon.clone())
            || difference.abs_diff_eq(&T::two_pi(), epsilon)
    }
}

impl<T: RealField + Euclid> Angle<T> {
    pub fn cos(&self) -> T {
        self.0.clone().cos()
    }

    pub fn sin(&self) -> T {
        self.0.clone().sin()
    }

    #[must_use]
    pub fn angle_to(&self, to: Self, direction: Direction) -> Self {
        ((to - self.clone()) * direction.angle_sign::<T>()).normalized()
    }

    pub fn as_direction(&self) -> Vector2<T> {
        vector![self.cos(), self.sin()]
    }
}

impl<T: Euclid + RealField> Angle<T> {
    #[must_use]
    pub fn normalized(&self) -> Self {
        Angle(self.0.rem_euclid(&T::two_pi()))
    }
}

impl<T: Add<Output = T>> Add for Angle<T> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl<T: Sub<Output = T>> Sub for Angle<T> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl<T: Mul<Output = T>> Mul<T> for Angle<T> {
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl<T: RealField> Mul<Vector2<T>> for Angle<T> {
    type Output = Vector2<T>;

    fn mul(self, rhs: Vector2<T>) -> Self::Output {
        Rotation2::new(self.0) * rhs
    }
}

impl<T: Div<Output = T>> Div for Angle<T> {
    type Output = T;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

impl<T: Div<Output = T>> Div<T> for Angle<T> {
    type Output = Self;

    fn div(self, rhs: T) -> Self::Output {
        Angle(self.0 / rhs)
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_3};

    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn angle_to() {
        let eps = 1e-15;

        assert_abs_diff_eq!(
            Angle(0.0).angle_to(Angle(FRAC_PI_2), Direction::Clockwise),
            Angle(3.0 * FRAC_PI_2),
            epsilon = eps
        );
        assert_abs_diff_eq!(
            Angle(0.0).angle_to(Angle(FRAC_PI_2), Direction::Counterclockwise),
            Angle(FRAC_PI_2),
            epsilon = eps
        );
        assert_abs_diff_eq!(
            Angle(5.0 * FRAC_PI_3).angle_to(Angle(FRAC_PI_3), Direction::Clockwise),
            Angle(4.0 * FRAC_PI_3),
            epsilon = eps
        );
        assert_abs_diff_eq!(
            Angle(5.0 * FRAC_PI_3).angle_to(Angle(FRAC_PI_3), Direction::Counterclockwise),
            Angle(2.0 * FRAC_PI_3),
            epsilon = eps
        );
    }
}
