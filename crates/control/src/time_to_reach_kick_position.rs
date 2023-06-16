use color_eyre::Result;

use context_attribute::context;
use framework::MainOutput;
use types::BallState;

#[context]
pub struct CycleContext {
    pub ball_state: Input<Option<BallState>, "ball_state?">,
    pub rule_ball_state: Input<Option<BallState>, "rule_ball_state?">,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct MainOutputs {
    pub time_to_reach_kick_position: MainOutput<Option<f32>>,
}

pub struct TimeToReachKickPosition {}

impl TimeToReachKickPosition {
    pub fn new(_: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, context: CycleContext) -> Result<MainOutputs> {
        let time_to_reach_kick_position = match (context.ball_state, context.rule_ball_state) {
            (Some(ball_state), _) | (None, Some(ball_state)) => {
                // For now, just use the euclidean distance to the ball.
                // This should be replaced by something more sophisticated down the line.
                Some(ball_state.ball_in_ground.coords.norm())
            }
            _ => None,
        };

        Ok(MainOutputs {
            time_to_reach_kick_position: time_to_reach_kick_position.into(),
        })
    }
}
