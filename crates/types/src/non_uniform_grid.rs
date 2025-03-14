use std::ops::Range;

use itertools::Itertools;
use linear_algebra::{point, Point2};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect)]
pub struct NonUniformGrid<T> {
    vertices: Vec<Vec<T>>,
}

impl<Frame> NonUniformGrid<Point2<Frame>> {
    pub fn new_uniform(
        x_range: Range<f32>,
        y_range: Range<f32>,
        x_resolution: usize,
        y_resolution: usize,
    ) -> Self {
        let x_dimension = x_range.end - x_range.start;
        let y_dimension = y_range.end - y_range.start;
        let cell_x_dimension = x_dimension / x_resolution as f32;
        let cell_y_dimension = y_dimension / y_resolution as f32;
        let vertices = (0..=y_resolution)
            .map(|i| y_range.start + i as f32 * cell_y_dimension)
            .map(|y| {
                (0..=x_resolution)
                    .map(|j| x_range.start + j as f32 * cell_x_dimension)
                    .map(|x| point![x, y])
                    .collect()
            })
            .collect();

        Self { vertices }
    }
}

impl<T: Clone> NonUniformGrid<T> {
    pub fn map<T2>(self, op: impl Fn(T) -> T2) -> NonUniformGrid<T2> {
        let vertices = self
            .vertices
            .into_iter()
            .map(|row| row.into_iter().map(&op).collect())
            .collect();

        NonUniformGrid { vertices }
    }

    // pub fn try_map<T2, Error>(
    //     self,
    //     op: impl Fn(T) -> Result<T2, Error>,
    // ) -> Result<NonUniformGrid<T2>, Error> {
    //     let vertices = self
    //         .vertices
    //         .into_iter()
    //         .map(|row| row.into_iter().map(&op).collect::<Result<Vec<_>, _>>())
    //         .collect::<Result<Vec<_>, _>>()?;

    //     Ok(NonUniformGrid { vertices })
    // }

    pub fn quads(&self) -> impl Iterator<Item = Quad<T>> + '_ {
        self.vertices
            .iter()
            .tuple_windows()
            .flat_map(|(upper, lower)| {
                upper
                    .iter()
                    .zip(lower.iter())
                    .tuple_windows()
                    .map(|((tl, bl), (tr, br))| Quad {
                        vertices: [tl.clone(), tr.clone(), br.clone(), bl.clone()],
                    })
            })
    }
}

#[derive(Debug, PartialEq)]
pub struct Quad<T> {
    pub vertices: [T; 4],
}

#[cfg(test)]
mod tests {
    use linear_algebra::{point, Point2};

    use super::{NonUniformGrid, Quad};

    #[derive(Debug, PartialEq)]
    struct SomeFrame;

    #[test]
    fn non_uniform_grid_quads() {
        let grid = NonUniformGrid::new_uniform(0.0..4.0, 0.0..6.0, 2, 3);
        let quads: Vec<Quad<Point2<SomeFrame>>> = grid.quads().collect();

        assert_eq!(
            quads,
            [
                Quad {
                    vertices: [
                        point![0.0, 0.0],
                        point![2.0, 0.0],
                        point![2.0, 2.0],
                        point![0.0, 2.0]
                    ]
                },
                Quad {
                    vertices: [
                        point![2.0, 0.0],
                        point![4.0, 0.0],
                        point![4.0, 2.0],
                        point![2.0, 2.0]
                    ]
                },
                Quad {
                    vertices: [
                        point![0.0, 2.0],
                        point![2.0, 2.0],
                        point![2.0, 4.0],
                        point![0.0, 4.0]
                    ]
                },
                Quad {
                    vertices: [
                        point![2.0, 2.0],
                        point![4.0, 2.0],
                        point![4.0, 4.0],
                        point![2.0, 4.0]
                    ]
                },
                Quad {
                    vertices: [
                        point![0.0, 4.0],
                        point![2.0, 4.0],
                        point![2.0, 6.0],
                        point![0.0, 6.0]
                    ]
                },
                Quad {
                    vertices: [
                        point![2.0, 4.0],
                        point![4.0, 4.0],
                        point![4.0, 6.0],
                        point![2.0, 6.0]
                    ]
                },
            ]
        );
    }
}
