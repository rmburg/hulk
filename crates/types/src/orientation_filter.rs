use std::convert::Infallible;

use coordinate_systems::Field;
use linear_algebra::{Orientation2, Orientation3};
use nalgebra::{UnitComplex, UnitQuaternion, Vector3};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

#[derive(
    Clone, Debug, Default, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct Parameters {
    pub acceleration_threshold: f32,
    pub delta_angular_velocity_threshold: f32,
    pub angular_velocity_bias_weight: f32,
    pub acceleration_weight: f32,
    pub falling_threshold: f32,
    pub force_sensitive_resistor_threshold: f32,
}

#[derive(
    Clone, Debug, Default, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct State {
    pub previous_angular_velocity: Vector3<f32>,
    pub angular_velocity_bias: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
    pub is_initialized: bool,
}

impl State {
    pub fn yaw(&self) -> UnitComplex<f32> {
        let (_, _, yaw) = self.orientation.inverse().euler_angles();
        UnitComplex::new(yaw)
    }

    pub fn angles(&self) -> UnitQuaternion<f32> {
        self.orientation.inverse()
    }
}

#[derive(
    Clone, Debug, Default, Deserialize, PathDeserialize, PathIntrospect, PathSerialize, Serialize,
)]
#[path_serde(add_leaf(euler_angles: EulerAngles))]
pub struct Orientation {
    inner: Orientation3<Field>,
}

impl Orientation {
    pub fn from(angles: UnitQuaternion<f32>) -> Self {
        Self {
            inner: Orientation3::wrap(angles),
        }
    }

    pub fn yaw(&self) -> Orientation2<Field, f32> {
        Orientation2::new(self.inner.inner.euler_angles().2)
    }
}

#[derive(Serialize)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl TryFrom<&Orientation> for EulerAngles {
    type Error = Infallible;

    fn try_from(value: &Orientation) -> Result<Self, Self::Error> {
        let (roll, pitch, yaw) = value.inner.inner.euler_angles();

        Ok(EulerAngles { roll, pitch, yaw })
    }
}
