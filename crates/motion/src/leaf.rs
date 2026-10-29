//! The surface a layer's beads lie on and the slab under it, which a bent layer's beads follow.
//!
//! A layer's top lies at `top + warp(x, y)` and its bottom at the top of the layer below, so the
//! slab under a point is `height + warp − below` thick. Both reliefs are linear between the
//! points where a segment crosses their grid's lines (`Relief::breaks`): cut there, a bead's
//! straight pieces lie on the layer exactly, each laying the exact mean of its section.

use layercraft_geom::{Point, Relief};

/// One layer's top, and the top of the layer below it.
pub(crate) struct Leaf<'a> {
    pub top: f64,
    pub height: f64,
    pub warp: &'a Relief,
    pub below: &'a Relief,
}

impl Leaf<'_> {
    /// The height of the layer's top over `p`.
    pub fn z(&self, p: Point) -> f64 {
        self.top + self.warp.at(p)
    }

    /// The thickness of the slab under `p`.
    pub fn thickness(&self, p: Point) -> f64 {
        self.height + self.warp.at(p) - self.below.at(p)
    }

    /// The points from `a` (left out) to `b` (kept) between which both reliefs are linear.
    pub fn pieces(&self, a: Point, b: Point) -> Vec<Point> {
        let mut ts = self.warp.breaks(a, b);
        ts.extend(self.below.breaks(a, b));
        ts.sort_by(f64::total_cmp);
        ts.dedup();
        ts.into_iter().map(|t| [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]).chain([b]).collect()
    }

    /// The highest the layer's top reaches anywhere.
    pub fn highest(&self) -> f64 {
        self.top + self.warp.range()[1]
    }

    /// The highest the layer's top reaches along the segment from `a` to `b`.
    pub fn highest_along(&self, a: Point, b: Point) -> f64 {
        self.top + self.warp.max_along(a, b)
    }

    /// Whether the layer's top is level within `radius` of `p`.
    pub fn level_within(&self, p: Point, radius: f64) -> bool {
        let [lo, hi] = self.warp.range_within(p, radius);
        lo == hi
    }
}
