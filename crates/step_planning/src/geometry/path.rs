use types::planned_path::PathSegment;

#[derive(Clone)]
pub struct Path {
    pub segments: Vec<PathSegment>,
}
