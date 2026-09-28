//! Position commands with independent joint speed limits.

use booster::MotorState;
use color_eyre::{Result, eyre::ensure};
use kinematics::joints::head::{HeadJoint, HeadJoints};
use types::{joint_limits::JointLimits, motor_command::MotorCommand};

use crate::parameters::JointControlParameters;

#[derive(Clone, Copy)]
pub struct HeadObservation {
    pub positions: HeadJoints<f32>,
}

impl From<HeadJoints<MotorState>> for HeadObservation {
    fn from(head: HeadJoints<MotorState>) -> Self {
        Self {
            positions: HeadJoints {
                yaw: head.yaw.position,
                pitch: head.pitch.position,
            },
        }
    }
}

impl HeadObservation {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.positions.into_iter().all(f32::is_finite),
            "head observation contains non-finite positions"
        );
        Ok(())
    }
}

pub enum JointTarget {
    MoveTo {
        position: HeadJoints<f32>,
        travel_speed: HeadJoints<f32>,
    },
    Damping,
}

/// The caller seeds the reference from measurements on activation. Position bounds
/// take precedence over speed limiting if measurements or live limit edits put the
/// reference outside the permitted range. No acceleration or jerk limits apply.
pub fn motor_commands(
    target: JointTarget,
    reference: HeadJoints<f32>,
    elapsed: f32,
    parameters: &JointControlParameters,
    limits: &JointLimits,
) -> Result<HeadJoints<MotorCommand>> {
    let mut commands = HeadJoints::fill(MotorCommand::zeros());
    match target {
        JointTarget::MoveTo {
            position,
            travel_speed,
        } => {
            ensure!(
                position.into_iter().all(f32::is_finite),
                "head target contains non-finite values"
            );
            for joint in [HeadJoint::Yaw, HeadJoint::Pitch] {
                let [minimum, maximum] = limits.position.head[joint];
                let start = reference[joint].clamp(minimum, maximum);
                let goal = position[joint].clamp(minimum, maximum);
                let step = travel_speed[joint].min(parameters.maximum_velocity[joint]) * elapsed;
                commands[joint] = MotorCommand {
                    position: (start + (goal - start).clamp(-step, step)).clamp(minimum, maximum),
                    kp: parameters.kp[joint],
                    kd: parameters.kd[joint],
                    ..MotorCommand::zeros()
                };
            }
        }
        JointTarget::Damping => {
            for joint in [HeadJoint::Yaw, HeadJoint::Pitch] {
                let [minimum, maximum] = limits.position.head[joint];
                commands[joint] = MotorCommand {
                    position: reference[joint].clamp(minimum, maximum),
                    kd: parameters.damping_kd[joint],
                    ..MotorCommand::zeros()
                };
            }
        }
    }
    Ok(commands)
}
