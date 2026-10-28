//! A region's edges filed by the horizontal bands their heights span, about the square root of
//! them in each band: whether a point or a segment lies in the region is found from the edges of
//! the bands it spans only.
//!
//! A segment lies in the region when it crosses no edge and each piece of it between the points
//! where it touches the edge lies in the region at its middle: only crossing or touching the edge
//! takes a segment from inside to outside, so each piece lies wholly on one side.

use crate::{Point, Region, crosses_ray, edges, segment_distance};

/// A point this close to the edge, in mm, lies on it.
pub const ON_EDGE_MM: f64 = 1e-4;

/// The edges of a region, by band.
pub struct Bands {
    /// The height of the first band's foot.
    foot: f64,
    /// The height of each band, in mm.
    height: f64,
    /// Each band's edges: every edge whose height reaches into the band.
    bands: Vec<Vec<(Point, Point)>>,
}

fn cross(o: Point, a: Point, b: Point) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

fn length(a: Point, b: Point) -> f64 {
    ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt()
}

/// Whether segments `ab` and `cd` cross: each one's ends lie farther than `ON_EDGE_MM` from the
/// other's line, on either side of it.
fn crosses(a: Point, b: Point, c: Point, d: Point) -> bool {
    let apart =
        |s: f64, t: f64, tolerance: f64| s > tolerance && t < -tolerance || s < -tolerance && t > tolerance;
    apart(cross(a, b, c), cross(a, b, d), ON_EDGE_MM * length(a, b))
        && apart(cross(c, d, a), cross(c, d, b), ON_EDGE_MM * length(c, d))
}

impl Bands {
    /// Files the edges of `region`.
    pub fn new(region: &Region) -> Bands {
        let all: Vec<(Point, Point)> = region.iter().flatten().flat_map(|c| edges(c)).collect();
        let (low, high) = all
            .iter()
            .flat_map(|(a, b)| [a[1], b[1]])
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), y| (low.min(y), high.max(y)));
        let count = (all.len() as f64).sqrt().ceil().max(1.0);
        let height = if high > low { (high - low) / count } else { 1.0 };
        let foot = if low.is_finite() { low } else { 0.0 };
        let mut bands = Bands { foot, height, bands: vec![Vec::new(); count as usize] };
        for (a, b) in all {
            let (first, last) = (bands.band(a[1].min(b[1])), bands.band(a[1].max(b[1])));
            bands.bands.iter_mut().take(last + 1).skip(first).for_each(|band| band.push((a, b)));
        }
        bands
    }

    /// The band at height `y`, the first or last past them.
    fn band(&self, y: f64) -> usize {
        let k = ((y - self.foot) / self.height).floor();
        if k >= 1.0 { (k as usize).min(self.bands.len().saturating_sub(1)) } else { 0 }
    }

    /// The edges of the bands from height `low` to `high`: one that spans several, once for each.
    fn between(&self, low: f64, high: f64) -> impl Iterator<Item = (Point, Point)> + '_ {
        self.bands.get(self.band(low)..=self.band(high)).into_iter().flatten().flatten().copied()
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
