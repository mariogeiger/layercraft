//! A part's own frame: the centroid of its areas and their own axes, which turn, move and
//! mirror with the part. The first axis is the areas' own line (`Moments`): their axis of
//! greatest spread or, where they spread alike every way, the first line of their edges, as a
//! square's side. Of the lines and ways along them so left open, it is the one the areas skew
//! the most toward (their third moment along it greatest); the second axis is the quarter turn
//! from it, either way, that they skew more toward, so a mirrored part has a mirrored frame.
//! Areas with no line of their own, as a disc, keep the plate's axes, between whose ways skew
//! alone decides.

use layercraft_geom::{Moments, Point, Region};

/// The frame: where its origin lies on the plate, and its two axes, unit vectors a quarter turn
/// apart.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Frame {
    origin: Point,
    axis: [f64; 2],
    second: [f64; 2],
}

impl Default for Frame {
    /// The plate's own.
    fn default() -> Frame {
        Frame { origin: [0.0, 0.0], axis: [1.0, 0.0], second: [0.0, 1.0] }
    }
}

impl Frame {
    /// The frame of areas, each weighing what is given with it (a layer's height); the plate's
    /// own for no area.
    pub fn of<'a>(areas: impl Iterator<Item = (&'a Region, f64)> + Clone) -> Frame {
        let shapes =
            areas.clone().flat_map(|(region, w)| region.iter().map(move |shape| (shape.as_slice(), w)));
        let Some(moments) = Moments::of(shapes) else {
            let first = areas.flat_map(|(r, _)| r.iter().flatten().flatten()).next().copied();
            return Frame { origin: first.unwrap_or([0.0, 0.0]), ..Frame::default() };
        };
        let lines: Vec<[f64; 2]> = match moments.axes {
            Some(axes) => axes.lines().collect(),
            None => vec![[1.0, 0.0]],
        };
        let ways = lines.into_iter().flat_map(|u| [u, [-u[0], -u[1]]]);
        let axis = moments.most_skewed(ways).unwrap_or([1.0, 0.0]);
        let (left, right) = ([-axis[1], axis[0]], [axis[1], -axis[0]]);
        let second = moments.most_skewed([left, right].into_iter()).unwrap_or(left);
        Frame { origin: moments.centroid, axis, second }
    }

    /// Its first axis, a unit vector.
    pub fn axis(&self) -> [f64; 2] {
        self.axis
    }

    /// Whether its second axis lies a quarter turn anticlockwise from its first, as the plate's.
    pub fn right_handed(&self) -> bool {
        self.axis[0] * self.second[1] - self.axis[1] * self.second[0] > 0.0
    }

    /// The direction of its first axis, in degrees.
    pub fn angle(&self) -> f64 {
        self.axis[1].atan2(self.axis[0]).to_degrees()
    }

    /// A point of the plate in the frame.
    pub fn local(&self, p: Point) -> Point {
        let (x, y) = (p[0] - self.origin[0], p[1] - self.origin[1]);
        [x * self.axis[0] + y * self.axis[1], x * self.second[0] + y * self.second[1]]
    }

    /// A point of the frame on the plate.
    pub fn plate(&self, q: Point) -> Point {
        [
            self.origin[0] + q[0] * self.axis[0] + q[1] * self.second[0],
            self.origin[1] + q[0] * self.axis[1] + q[1] * self.second[1],
        ]
    }
}
