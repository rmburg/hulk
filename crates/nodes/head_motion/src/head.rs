//! Request resolution and coordination, independent of ROS interfaces and logging.

use std::{sync::Arc, time::Duration};

use color_eyre::{
    Result,
    eyre::{ensure, eyre},
};
use coordinate_systems::Ground;
use kinematics::joints::head::HeadJoints;
use linear_algebra::{Point2, Rotation2, point};
use ros_z::time::Time;
use types::{
    field_dimensions::GlobalFieldSide,
    joint_limits::JointLimits,
    motion_command::{HeadMotion, ImageRegion},
    motor_command::MotorCommand,
    parameters::ImageRegionParameters,
    support_foot::Side,
};

use crate::{
    joint_control::{HeadObservation, JointTarget, motor_commands},
    look_at::{GazeGeometry, LookAtError, look_at},
    parameters::Parameters,
    patterns::{GlanceState, ScanKind, ScanState},
};

pub struct HeadInputs {
    pub parameters: Arc<Parameters>,
    pub joint_limits: Arc<JointLimits>,
    pub geometry: Option<GazeGeometry>,
    pub field_width: Option<f32>,
    pub global_field_side: Option<GlobalFieldSide>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldReason {
    MissingGeometry,
    MissingFieldDimensions,
    InvalidFieldWidth,
    Geometry(LookAtError),
}

pub struct HeadOutput {
    pub commands: HeadJoints<MotorCommand>,
    pub hold_reason: Option<HoldReason>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Direct,
    Scan(ScanKind),
    Glance,
    Damping,
    Injected,
}

struct TimedObservation {
    value: HeadObservation,
    time: Time,
}

#[derive(Default)]
pub struct HeadController {
    observation: Option<TimedObservation>,
    mode: Option<Mode>,
    last_evaluation: Option<Time>,
    reference_position: Option<HeadJoints<f32>>,
    hold_target: Option<HeadJoints<f32>>,
    scan: ScanState,
    glance: GlanceState,
}

impl HeadController {
    pub fn observe(&mut self, observation: HeadObservation, source_time: Time) -> Result<()> {
        observation.validate()?;
        if self
            .observation
            .as_ref()
            .is_some_and(|previous| source_time < previous.time)
        {
            self.reset_motion();
        }
        self.observation = Some(TimedObservation {
            value: observation,
            time: source_time,
        });
        Ok(())
    }

    pub fn evaluate(
        &mut self,
        request: &HeadMotion,
        inputs: &HeadInputs,
        now: Time,
    ) -> Result<HeadOutput> {
        let parameters = &inputs.parameters;
        let observation = self.current_observation(now, parameters.maximum_observation_age)?;
        let joint_limits = &inputs.joint_limits;

        let mode = mode_for(request, parameters.injected_head_joints.is_some());
        self.prepare_mode(mode, now, parameters.joint_control.reseed_after);
        let reference = self.reference_position.unwrap_or(observation.positions);
        let elapsed = if self.reference_position.is_some() {
            self.last_evaluation
                .map_or(0.0, |last| now.duration_since(last).as_secs_f32())
        } else {
            0.0
        };
        let resolution = self.resolve(request, inputs, reference, now);
        let commands = motor_commands(
            resolution.target,
            reference,
            elapsed,
            &parameters.joint_control,
            joint_limits,
        )?;
        self.reference_position = if mode == Mode::Damping {
            None
        } else {
            Some(HeadJoints {
                yaw: commands.yaw.position,
                pitch: commands.pitch.position,
            })
        };
        self.last_evaluation = Some(now);
        Ok(HeadOutput {
            commands,
            hold_reason: resolution.hold_reason,
        })
    }

    fn current_observation(&self, now: Time, maximum_age: Duration) -> Result<HeadObservation> {
        let observation = self
            .observation
            .as_ref()
            .ok_or_else(|| eyre!("head observation is unavailable"))?;
        ensure!(
            now >= observation.time,
            "head observation source time {:?} is ahead of node time {now:?}; \
             check source/node clock alignment or clock rollback",
            observation.time,
        );
        let age = now.duration_since(observation.time);
        ensure!(
            age <= maximum_age,
            "latest valid head observation is stale: age={age:?}, maximum_age={maximum_age:?}; \
             check motor-state delivery and source/node clock alignment"
        );
        Ok(observation.value)
    }

    fn prepare_mode(&mut self, mode: Mode, now: Time, reseed_after: Duration) {
        if self
            .last_evaluation
            .is_some_and(|last| now < last || now.duration_since(last) > reseed_after)
        {
            self.reset_motion();
        }
        if self.mode != Some(mode) {
            self.scan = ScanState::default();
            self.glance = GlanceState::default();
            self.hold_target = None;
        }
        self.mode = Some(mode);
    }

    fn resolve(
        &mut self,
        request: &HeadMotion,
        inputs: &HeadInputs,
        reference: HeadJoints<f32>,
        now: Time,
    ) -> Resolution {
        let parameters = &inputs.parameters;
        if let Some(position) = parameters.injected_head_joints {
            self.hold_target = None;
            return Resolution::motion(move_to(position, parameters.direct_travel_speed));
        }
        match *request {
            HeadMotion::ZeroAngles => {
                self.hold_target = None;
                Resolution::motion(move_to(
                    HeadJoints::fill(0.0),
                    parameters.direct_travel_speed,
                ))
            }
            HeadMotion::Damping => {
                self.hold_target = None;
                Resolution::motion(JointTarget::Damping)
            }
            HeadMotion::Center {
                image_region_target,
            } => self.center(image_region_target, inputs, reference),
            HeadMotion::LookAt {
                target,
                height_above_ground,
                image_region_target,
            } => {
                let angles = gaze(
                    target,
                    height_above_ground,
                    image_region_target,
                    inputs.geometry.as_ref(),
                    &parameters.image_region_parameters,
                    reference,
                );
                self.gaze_motion(angles, parameters.direct_travel_speed, reference)
            }
            HeadMotion::LookAround | HeadMotion::SearchForLostBall => {
                let kind = if matches!(request, HeadMotion::LookAround) {
                    ScanKind::LookAround
                } else {
                    ScanKind::SearchForLostBall
                };
                let side = if inputs.global_field_side == Some(GlobalFieldSide::Away) {
                    Side::Right
                } else {
                    Side::Left
                };
                self.hold_target = None;
                let scan_parameters = match kind {
                    ScanKind::LookAround => &parameters.look_around,
                    ScanKind::SearchForLostBall => &parameters.search_for_lost_ball,
                };
                let position = self.scan.update(kind, side, scan_parameters, now);
                Resolution::motion(move_to(position, scan_parameters.travel_speed))
            }
            HeadMotion::LookLeftAndRightOf {
                target,
                height_above_ground,
            } => {
                let angle = self.glance.angle(
                    parameters.glance.angle,
                    parameters.glance.phase_duration,
                    now,
                );
                let angles = gaze(
                    Rotation2::<Ground, Ground>::new(angle) * target,
                    height_above_ground,
                    ImageRegion::Center,
                    inputs.geometry.as_ref(),
                    &parameters.image_region_parameters,
                    reference,
                );
                self.gaze_motion(angles, parameters.glance.travel_speed, reference)
            }
            HeadMotion::MoveWithVelocity { yaw, pitch } => {
                self.hold_target = None;
                Resolution::motion(move_to(
                    reference + HeadJoints { yaw, pitch },
                    parameters.direct_travel_speed,
                ))
            }
        }
    }

    fn center(
        &mut self,
        image_region: ImageRegion,
        inputs: &HeadInputs,
        reference: HeadJoints<f32>,
    ) -> Resolution {
        let parameters = &inputs.parameters;
        let angles = inputs
            .field_width
            .ok_or(HoldReason::MissingFieldDimensions)
            .and_then(|width| {
                if !width.is_finite() || width <= 0.0 {
                    return Err(HoldReason::InvalidFieldWidth);
                }
                gaze(
                    point![width / 2.0, 0.0],
                    0.0,
                    image_region,
                    inputs.geometry.as_ref(),
                    &parameters.image_region_parameters,
                    reference,
                )
            });
        self.gaze_motion(angles, parameters.direct_travel_speed, reference)
    }

    fn gaze_motion(
        &mut self,
        angles: Result<HeadJoints<f32>, HoldReason>,
        travel_speed: HeadJoints<f32>,
        reference: HeadJoints<f32>,
    ) -> Resolution {
        match angles {
            Ok(position) => {
                self.hold_target = None;
                Resolution::motion(move_to(position, travel_speed))
            }
            Err(reason) => {
                let position = *self.hold_target.get_or_insert(reference);
                Resolution {
                    target: move_to(position, travel_speed),
                    hold_reason: Some(reason),
                }
            }
        }
    }

    /// Explicitly reset all motion state and seed the measurement cache.
    pub fn reset(&mut self, observation: HeadObservation, now: Time) -> Result<()> {
        self.reset_motion();
        self.observe(observation, now)
    }

    fn reset_motion(&mut self) {
        self.mode = None;
        self.last_evaluation = None;
        self.reference_position = None;
        self.hold_target = None;
        self.scan = ScanState::default();
        self.glance = GlanceState::default();
    }
}

struct Resolution {
    target: JointTarget,
    hold_reason: Option<HoldReason>,
}

impl Resolution {
    fn motion(target: JointTarget) -> Self {
        Self {
            target,
            hold_reason: None,
        }
    }
}

fn move_to(position: HeadJoints<f32>, travel_speed: HeadJoints<f32>) -> JointTarget {
    JointTarget::MoveTo {
        position,
        travel_speed,
    }
}

fn gaze(
    target: Point2<Ground>,
    height: f32,
    image_region: ImageRegion,
    geometry: Option<&GazeGeometry>,
    parameters: &ImageRegionParameters,
    reference: HeadJoints<f32>,
) -> Result<HeadJoints<f32>, HoldReason> {
    let geometry = geometry.ok_or(HoldReason::MissingGeometry)?;
    look_at(
        target,
        height,
        image_region,
        geometry,
        parameters,
        reference,
    )
    .map_err(HoldReason::Geometry)
}

fn mode_for(request: &HeadMotion, injected: bool) -> Mode {
    if injected {
        return Mode::Injected;
    }
    match request {
        HeadMotion::LookAround => Mode::Scan(ScanKind::LookAround),
        HeadMotion::SearchForLostBall => Mode::Scan(ScanKind::SearchForLostBall),
        HeadMotion::LookLeftAndRightOf { .. } => Mode::Glance,
        HeadMotion::Damping => Mode::Damping,
        HeadMotion::ZeroAngles | HeadMotion::Center { .. } | HeadMotion::LookAt { .. } => {
            Mode::Direct
        }
        HeadMotion::MoveWithVelocity { .. } => Mode::Injected,
    }
}
