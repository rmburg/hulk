use std::convert::Infallible;

use coordinate_systems::Field;
use linear_algebra::{Orientation2, Orientation3, Vector3};
use nalgebra::{UnitComplex, UnitQuaternion};
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
#[path_serde(add_leaf(aldebaran: AldebaranAngles))]
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

#[derive(Serialize)]
pub struct AldebaranAngles {
    pub angle_x: f32,
    pub angle_y: f32,
    pub yaw: f32,
}

impl TryFrom<&Orientation> for AldebaranAngles {
    type Error = Infallible;

    fn try_from(value: &Orientation) -> Result<Self, Self::Error> {
        let rotation = value.inner.rotation::<Field>();

        let x = rotation * Vector3::x_axis();
        let y = rotation * Vector3::y_axis();
        let z = rotation * Vector3::z_axis();

        let up = Vector3::<Field>::z_axis();

        let up_in_yz = project_to_plane(x, up);
        let up_in_xz = project_to_plane(y, up);

        let angle_x = z.angle(up_in_yz);
        let angle_y = z.angle(up_in_xz);

        let (_, _, yaw) = value.inner.inner.euler_angles();

        Ok(AldebaranAngles {
            angle_x,
            angle_y,
            yaw,
        })
    }
}

fn project_to_plane<Frame>(normal: Vector3<Frame>, vector: Vector3<Frame>) -> Vector3<Frame> {
    (vector - vector * normal.dot(vector)).normalize()
}
