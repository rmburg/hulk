use std::{future::Future, pin::Pin, sync::Arc, time::Duration};

use booster::LowState;
use color_eyre::{Report, Result, eyre::WrapErr};
use coordinate_systems::{Ground, Robot};
use kinematics::joints::head::HeadJoints;
use linear_algebra::Isometry3;
use projection::camera_matrix::CameraMatrix;
use ros_z::{
    Result as RosResult,
    prelude::*,
    pubsub::Received,
    qos::{QosDurability, QosHistory},
    time::Time,
};
use ros_z_schema::{ServiceDef, compute_hash};
use serde::{Deserialize, Serialize};
use types::motor_command::MotorCommand;
use types::{
    field_dimensions::FieldDimensions, filtered_game_controller_state::FilteredGameControllerState,
    joint_limits::JointLimits, motion_command::HeadMotion, time_wrapper::TimeWrapper,
};

use crate::{
    head::{HeadContext, HeadController},
    logging::{FailureKind, NodeLogger},
    look_at::GazeGeometry,
    parameters::Parameters,
};

pub const HEAD_MOTION_SERVICE_TOPIC: &str = "services/head_motion";

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

pub async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("head_motion").build().await?;

    let parameters = node.bind_parameter_as::<Parameters>("head_motion")?;
    parameters.add_validation_hook(Parameters::validate)?;
    let joint_limits_cache = node
        .subscriber::<JointLimits>("joint_limits")
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .cache(1)
        .build()
        .await?;

    let field_dimensions_cache = node
        .subscriber::<FieldDimensions>("field_dimensions")
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .cache(1)
        .build()
        .await?;

    let low_state_sub = node
        .subscriber::<LowState>("inputs/low_state")
        .qos(QosProfile {
            history: QosHistory::from_depth(1),
            ..Default::default()
        })
        .build()
        .await?;
    let camera_matrix_cache = node
        .subscriber::<TimeWrapper<CameraMatrix>>("camera_matrix")
        .cache(1)
        .with_stamp(|wrapper: &TimeWrapper<CameraMatrix>| wrapper.time)
        .build()
        .await?;
    let ground_to_robot_cache = node
        .subscriber::<TimeWrapper<Option<Isometry3<Ground, Robot>>>>("ground_to_robot")
        .cache(1)
        .with_stamp(|wrapper: &TimeWrapper<Option<Isometry3<Ground, Robot>>>| wrapper.time)
        .build()
        .await?;
    let filtered_game_controller_state_cache = node
        .subscriber::<FilteredGameControllerState>("filtered_game_controller_state")
        .cache(1)
        .build()
        .await?;

    let mut head_motion_service = node
        .service_server::<HeadMotionService>(HEAD_MOTION_SERVICE_TOPIC)
        .qos(QosProfile {
            history: QosHistory::from_depth(1),
            ..Default::default()
        })
        .build()
        .await?;

    let mut controller = HeadController::default();
    let mut logger = NodeLogger::default();

    loop {
        tokio::select! {
            received = low_state_sub.recv_with_metadata() => {
                receive_observation(
                    received, &mut controller, &mut logger,
                    parameters.snapshot().typed().joint_control.warning_interval,
                    node.clock().now(),
                );
            }
            received = head_motion_service.take_request_async() => {
                let snapshot = parameters.snapshot();
                let parameters = snapshot.typed();
                let (request, reply) = match received {
                    Ok(received) => received.into_parts(),
                    Err(error) => {
                        logger.log_error(FailureKind::Request, None, &error.into(),
                            parameters.joint_control.warning_interval, node.clock().now());
                        continue;
                    }
                };
                let camera = camera_matrix_cache.get_latest();
                let ground = ground_to_robot_cache.get_latest();
                let limits = joint_limits_cache.get_latest();
                let field = field_dimensions_cache.get_latest();
                let game = filtered_game_controller_state_cache.get_latest();
                let context = HeadContext {
                    geometry: camera.as_deref().zip(ground.as_deref()).and_then(|(camera, ground)| {
                        ground.inner.map(|ground_to_robot| GazeGeometry {
                            camera_matrix: &camera.inner,
                            ground_to_robot,
                        })
                    }),
                    joint_limits: limits.as_deref(),
                    field_dimensions: field.as_deref(),
                    field_side: game.as_deref().map(|game| game.global_field_side),
                };
                let now = node.clock().now();
                let response = match controller.evaluate(&request, &context, parameters, now) {
                    Ok(output) => {
                        logger.log_output(&request, &output, parameters.joint_control.warning_interval, now);
                        Ok(output.commands)
                    }
                    Err(error) => {
                        logger.log_error(FailureKind::Request, Some(&request), &error,
                            parameters.joint_control.warning_interval, now);
                        Err(HeadMotionError { source: Arc::new(error) })
                    }
                };
                if let Err(error) = reply.reply_async(&response).await {
                    logger.log_error(FailureKind::Response, Some(&request), &error.into(),
                        parameters.joint_control.warning_interval, now);
                }
            }
        }
    }
}

fn receive_observation(
    received: RosResult<Received<LowState>>,
    controller: &mut HeadController,
    logger: &mut NodeLogger,
    warning_interval: Duration,
    now: Time,
) {
    let result = received.map_err(Report::new).and_then(|received| {
        let head = received
            .message
            .serial_motor_states()
            .wrap_err("invalid serial motor states in LowState")?
            .head;
        controller.observe(head.into(), received.source_time)
    });
    if let Err(error) = result {
        controller.invalidate_observation();
        logger.log_error(
            FailureKind::Observation,
            None,
            &error,
            warning_interval,
            now,
        );
    }
}

pub struct HeadMotionService;

#[derive(Clone, Debug, Serialize, Deserialize, Message, thiserror::Error)]
#[error("head motion failed: {source:#}")]
pub struct HeadMotionError {
    #[serde(with = "ros_z::message::report")]
    pub source: Arc<Report>,
}

impl Service for HeadMotionService {
    type Request = HeadMotion;
    type Response = Result<HeadJoints<MotorCommand>, HeadMotionError>;
}

impl ServiceTypeInfo for HeadMotionService {
    fn service_type_info() -> TypeInfo {
        let descriptor = ServiceDef::new(
            "head_motion::node::HeadMotionService",
            HeadMotion::type_name(),
            <Self as Service>::Response::type_name(),
        )
        .expect("static head motion service descriptor is valid");
        let hash = compute_hash(&descriptor).expect("static head motion service hash is valid");
        TypeInfo::new(descriptor.type_name.as_str(), hash)
    }
}
