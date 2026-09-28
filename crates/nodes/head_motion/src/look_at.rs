//! Analytic gaze geometry for K1. Target selection and motor control live elsewhere.

use std::f32::consts::TAU;

use coordinate_systems::{Ground, Pixel, Robot};
use kinematics::{joints::head::HeadJoints, robot_dimensions::RobotDimensions};
use linear_algebra::{Isometry3, Point2, Point3, nalgebra::Vector3, point};
use projection::camera_matrix::CameraMatrix;
use types::{motion_command::ImageRegion, parameters::ImageRegionParameters};

// Numerical tolerances for solving and checking gaze geometry.
const MINIMUM_DISTANCE: f32 = 1e-6;
const MAXIMUM_PIXEL_ERROR: f32 = 0.05;

pub struct GazeGeometry<'a> {
    pub camera_matrix: &'a CameraMatrix,
    pub ground_to_robot: Isometry3<Ground, Robot>,
}

struct RayGeometry {
    /// Vector from the pitch pivot to the target, expressed in Robot axes.
    pivot_target: Vector3<f32>,
    /// Camera optical center expressed in Head coordinates.
    camera_origin: Vector3<f32>,
    /// Unit direction of the requested image ray, expressed in Head axes.
    camera_ray: Vector3<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LookAtError {
    InvalidTarget,
    InvalidGeometry,
    InvalidReference,
    NoSolution,
}

/// Place a ground-relative point at the requested image position.
/// Height is measured along Ground's +Z axis, in meters (e.g. ball radius).
/// The reference chooses the nearest solution, including equivalent full turns;
/// use the current joint-control reference, or measurements before initialization.
/// Returned angles are unconstrained: joint control owns mechanical limits.
pub fn look_at(
    target: Point2<Ground>,
    height_above_ground: f32,
    image_region: ImageRegion,
    geometry: &GazeGeometry<'_>,
    parameters: &ImageRegionParameters,
    reference: HeadJoints<f32>,
) -> Result<HeadJoints<f32>, LookAtError> {
    let target = point![target.x(), target.y(), height_above_ground];
    if !target.inner.coords.iter().all(|value| value.is_finite()) {
        return Err(LookAtError::InvalidTarget);
    }
    if !reference.into_iter().all(f32::is_finite) {
        return Err(LookAtError::InvalidReference);
    }
    let pixel = requested_pixel(image_region, parameters, geometry)?;
    let RayGeometry {
        pivot_target,
        camera_origin,
        camera_ray,
    } = ray_geometry(target, pixel, geometry);
    let mut best = None;
    let mut best_distance = f32::INFINITY;

    for distance in
        ray_distances(pivot_target, camera_origin, camera_ray).ok_or(LookAtError::NoSolution)?
    {
        if distance <= MINIMUM_DISTANCE || !distance.is_finite() {
            continue;
        }
        let point_on_ray = camera_origin + distance * camera_ray;
        let Some(candidates) = joint_solutions(pivot_target, point_on_ray, reference) else {
            continue;
        };
        for candidate in candidates {
            if !frames_target(candidate, target, pixel, geometry) {
                continue;
            }
            let distance = (candidate.yaw - reference.yaw).powi(2)
                + (candidate.pitch - reference.pitch).powi(2);
            if distance < best_distance {
                best = Some(candidate);
                best_distance = distance;
            }
        }
    }
    best.ok_or(LookAtError::NoSolution)
}

fn requested_pixel(
    region: ImageRegion,
    parameters: &ImageRegionParameters,
    geometry: &GazeGeometry<'_>,
) -> Result<Point2<Pixel>, LookAtError> {
    let camera = geometry.camera_matrix;
    let normalized = match region {
        ImageRegion::Center => parameters.center,
        ImageRegion::Bottom => parameters.bottom,
        ImageRegion::Top => parameters.top,
    };
    let valid = normalized
        .inner
        .coords
        .iter()
        .all(|value| (0.0..=1.0).contains(value))
        && camera
            .image_size
            .inner
            .iter()
            .chain(camera.intrinsics.focals.iter())
            .all(|value| value.is_finite() && *value > 0.0)
        && camera
            .intrinsics
            .optical_center
            .inner
            .coords
            .iter()
            .all(|value| value.is_finite())
        && valid_transform(geometry.ground_to_robot)
        && valid_transform(camera.head_to_camera)
        && (camera.correction_in_robot.inner.quaternion().norm_squared() - 1.0).abs() < 1e-5;
    if !valid {
        return Err(LookAtError::InvalidGeometry);
    }
    Ok(point![
        normalized.x() * camera.image_size.x(),
        normalized.y() * camera.image_size.y()
    ])
}

fn valid_transform<From, To>(transform: Isometry3<From, To>) -> bool {
    transform
        .inner
        .translation
        .vector
        .iter()
        .all(|value| value.is_finite())
        && (transform.inner.rotation.quaternion().norm_squared() - 1.0).abs() < 1e-5
}

fn ray_geometry(
    target: Point3<Ground>,
    pixel: Point2<Pixel>,
    geometry: &GazeGeometry<'_>,
) -> RayGeometry {
    let camera = geometry.camera_matrix;
    let target_in_robot = camera.correction_in_robot * (geometry.ground_to_robot * target);
    // K1's pitch pivot lies on the yaw axis, so its position is independent of yaw.
    let pivot = RobotDimensions::ROBOT_TO_NECK.inner + RobotDimensions::NECK_TO_HEAD.inner;
    let pivot_target = target_in_robot.inner.coords - pivot;
    let camera_to_head = camera.head_to_camera.inner.inverse();
    let camera_origin = camera_to_head.translation.vector;
    let camera_ray = (camera_to_head.rotation
        * Vector3::new(
            (pixel.x() - camera.intrinsics.optical_center.x()) / camera.intrinsics.focals.x,
            (pixel.y() - camera.intrinsics.optical_center.y()) / camera.intrinsics.focals.y,
            1.0,
        ))
    .normalize();
    RayGeometry {
        pivot_target,
        camera_origin,
        camera_ray,
    }
}

/// Rotations preserve distance to the pivot. Intersect c + d*r with the sphere
/// of radius |target|: d² + 2(c·r)d + |c|² - |target|² = 0, with |r| = 1.
fn ray_distances(
    target: Vector3<f32>,
    origin: Vector3<f32>,
    ray: Vector3<f32>,
) -> Option<[f32; 2]> {
    let along = origin.dot(&ray);
    let constant = origin.norm_squared() - target.norm_squared();
    let discriminant = along * along - constant;
    if discriminant < 0.0 {
        return None;
    }
    // Stable quadratic roots avoid cancellation when a target is close to the camera.
    let root = -along - discriminant.sqrt().copysign(along);
    if root == 0.0 {
        Some([0.0; 2])
    } else {
        Some([root, constant / root])
    }
}

/// Solve target = Rz(yaw) * Ry(pitch) * point. Pitch preserves the Y coordinate;
/// yaw preserves height. Both possible signs of the intermediate X are considered.
fn joint_solutions(
    target: Vector3<f32>,
    point: Vector3<f32>,
    reference: HeadJoints<f32>,
) -> Option<[HeadJoints<f32>; 2]> {
    let horizontal_squared = target.x * target.x + target.y * target.y;
    let x_squared = horizontal_squared - point.y * point.y;
    let roundoff = 8.0 * f32::EPSILON * target.norm_squared().max(point.norm_squared());
    if x_squared < -roundoff {
        return None;
    }
    Some(
        [x_squared.max(0.0).sqrt(), -x_squared.max(0.0).sqrt()].map(|x| {
            let yaw = if horizontal_squared <= MINIMUM_DISTANCE.powi(2) {
                reference.yaw
            } else {
                target.y.atan2(target.x) - point.y.atan2(x)
            };
            let pitch = if point.x.hypot(point.z) <= MINIMUM_DISTANCE {
                reference.pitch
            } else {
                point.z.atan2(point.x) - target.z.atan2(x)
            };
            HeadJoints {
                yaw: nearest_equivalent(yaw, reference.yaw),
                pitch: nearest_equivalent(pitch, reference.pitch),
            }
        }),
    )
}

fn nearest_equivalent(angle: f32, reference: f32) -> f32 {
    angle + TAU * ((reference - angle) / TAU).round()
}

fn frames_target(
    joints: HeadJoints<f32>,
    target: Point3<Ground>,
    pixel: Point2<Pixel>,
    geometry: &GazeGeometry<'_>,
) -> bool {
    let camera = geometry.camera_matrix;
    let camera_target = camera.ground_to_camera_at(&joints, geometry.ground_to_robot) * target;
    if camera_target.z() <= MINIMUM_DISTANCE {
        return false;
    }
    let projected = camera.intrinsics.project(camera_target.coords());
    (projected - pixel).norm() <= MAXIMUM_PIXEL_ERROR
}
