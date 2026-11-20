//! Items filed by the horizontal bands their heights span, about the square root of them in each
//! band (`Banded`), and a region's edges so filed (`Bands`): whether a point or a segment lies in
//! the region is found from the edges of the bands it spans only.
//!
//! A segment lies in the region when it crosses no edge and each piece of it between the points
//! where it touches the edge lies in the region at its middle: only crossing or touching the edge
//! takes a segment from inside to outside, so each piece lies wholly on one side.

use crate::{Point, Region, crosses_ray, distance, edges, segment_distance};

/// A point this close to the edge, in mm, lies on it.
pub const ON_EDGE_MM: f64 = 1e-4;

/// Items filed by the horizontal bands their heights span: one spanning several is filed in each,
/// so the items reaching a height are among its band's. Heights past the bands fall in the first
/// or the last: the bands make a search quick, its answer does not depend on them.
#[derive(Debug, Clone)]
pub struct Banded<T> {
    /// The height of the first band's foot.
    foot: f64,
    /// The height of each band, in mm.
    height: f64,
    /// Each band's items: every item whose heights reach into the band.
    bands: Vec<Vec<T>>,
}

impl<T: Clone> Banded<T> {
    /// No items yet, in bands over the heights `low` to `high` for about `count` of them: the
    /// square root of `count` bands.
    pub fn new(low: f64, high: f64, count: usize) -> Banded<T> {
        let bands = (count as f64).sqrt().ceil().max(1.0);
        let height = if high > low { (high - low) / bands } else { 1.0 };
        let foot = if low.is_finite() { low } else { 0.0 };
        Banded { foot, height, bands: vec![Vec::new(); bands as usize] }
    }

    /// The band at height `y`, the first or last past them.
    fn band(&self, y: f64) -> usize {
        let k = ((y - self.foot) / self.height).floor();
        if k >= 1.0 { (k as usize).min(self.bands.len().saturating_sub(1)) } else { 0 }
    }

    /// Files `item`, whose heights run from `low` to `high`.
    pub fn insert(&mut self, low: f64, high: f64, item: T) {
        let (first, last) = (self.band(low), self.band(high));
        self.bands.iter_mut().take(last + 1).skip(first).for_each(|band| band.push(item.clone()));
    }

    /// The items of the bands from height `low` to `high`: one that spans several, once for each.
    pub fn between(&self, low: f64, high: f64) -> impl Iterator<Item = &T> + '_ {
        self.bands.get(self.band(low)..=self.band(high)).into_iter().flatten().flatten()
    }
}

/// The lowest and highest of the heights `ys`; infinite the wrong way round for none.
pub fn height_range(ys: impl IntoIterator<Item = f64>) -> (f64, f64) {
    ys.into_iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), y| (low.min(y), high.max(y)))
}

/// The edges of a region, by band.
pub struct Bands {
    edges: Banded<(Point, Point)>,
}

fn cross(o: Point, a: Point, b: Point) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

/// Whether segments `ab` and `cd` cross: each one's ends lie farther than `ON_EDGE_MM` from the
/// other's line, on either side of it.
pub fn crosses(a: Point, b: Point, c: Point, d: Point) -> bool {
    let apart =
        |s: f64, t: f64, tolerance: f64| s > tolerance && t < -tolerance || s < -tolerance && t > tolerance;
    apart(cross(a, b, c), cross(a, b, d), ON_EDGE_MM * distance(a, b))
        && apart(cross(c, d, a), cross(c, d, b), ON_EDGE_MM * distance(c, d))
}

impl Bands {
    /// Files the edges of `region`.
    pub fn new(region: &Region) -> Bands {
        let all: Vec<(Point, Point)> = region.iter().flatten().flat_map(|c| edges(c)).collect();
        let (low, high) = height_range(all.iter().flat_map(|(a, b)| [a[1], b[1]]));
        let mut filed = Banded::new(low, high, all.len());
        for (a, b) in all {
            filed.insert(a[1].min(b[1]), a[1].max(b[1]), (a, b));
        }
        Bands { edges: filed }
    }

    /// The edges of the bands from height `low` to `high`: one that spans several, once for each.
    fn between(&self, low: f64, high: f64) -> impl Iterator<Item = (Point, Point)> + '_ {
        self.edges.between(low, high).copied()
    }

    /// Whether `p` lies inside the region, by the even-odd rule.
    pub fn encloses(&self, p: Point) -> bool {
        self.between(p[1], p[1]).filter(|&e| crosses_ray(p, e)).count() % 2 == 1
    }

    /// Whether `p` lies inside the region or on its edge.
    pub fn covers(&self, p: Point) -> bool {
        self.between(p[1] - ON_EDGE_MM, p[1] + ON_EDGE_MM)
            .any(|(a, b)| segment_distance(p, a, b) < ON_EDGE_MM)
            || self.encloses(p)
    }

    /// Whether the segment `a`–`b` lies in the region, along its edge or inside.
    pub fn holds(&self, a: Point, b: Point) -> bool {
        let low = [a[0].min(b[0]) - ON_EDGE_MM, a[1].min(b[1]) - ON_EDGE_MM];
        let high = [a[0].max(b[0]) + ON_EDGE_MM, a[1].max(b[1]) + ON_EDGE_MM];
        let near = || {
            self.between(low[1], high[1]).filter(|(c, d)| {
                c[0].max(d[0]) >= low[0]
                    && c[0].min(d[0]) <= high[0]
                    && c[1].max(d[1]) >= low[1]
                    && c[1].min(d[1]) <= high[1]
            })
        };
        if near().any(|(c, d)| crosses(a, b, c, d)) {
            return false;
        }
        let along = [b[0] - a[0], b[1] - a[1]];
        let squared = along[0] * along[0] + along[1] * along[1];
        if squared <= 0.0 {
            return self.covers(a);
        }
        // Where it touches the edge: at the vertices on it, each the start of an edge.
        let mut cuts: Vec<f64> = near()
            .map(|(c, _)| c)
            .filter(|&c| segment_distance(c, a, b) < ON_EDGE_MM)
            .map(|c| (((c[0] - a[0]) * along[0] + (c[1] - a[1]) * along[1]) / squared).clamp(0.0, 1.0))
            .chain([0.0, 1.0])
            .collect();
        cuts.sort_by(f64::total_cmp);
        cuts.dedup();
        cuts.windows(2).all(|w| match *w {
            [s, t] => self.covers([a[0] + (s + t) / 2.0 * along[0], a[1] + (s + t) / 2.0 * along[1]]),
            _ => true,
        })
    }
}
