use std::fmt::Debug;

use crate::{condition::Response, Condition};

use serde::{Deserialize, Serialize};
use types::ConditionInput;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FallenAbort {}

impl Condition for FallenAbort {
    fn evaluate(&self, condition_input: &ConditionInput) -> Response {
        match condition_input.fall_state {
            types::FallState::Fallen { .. } => Response::Abort,
            _ => Response::Continue,
        }
    }
}
