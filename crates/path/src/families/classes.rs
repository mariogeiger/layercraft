//! Classes of directions, a quarter turn counting as none: the directions of lines and of those
//! across them. Directions closer than `SAME_DIRECTION` chain into one class, at the mean of
//! their weights taken as `4θ` round the circle, so that a quarter turn is a whole one; the
//! chains start past the widest gap, which the directions place, not the plate. A cell tries the
//! classes its edges and the beads below it run along most (`heaviest`).

use layercraft_geom::{Frame, Point, distance};
use std::cmp::Ordering;

/// The classes of directions that a cell's edges, and the beads below it, run along most, which
/// it tries.
const DIRECTIONS_TRIED: usize = 3;

/// Directions closer than this, in degrees, are one.
pub(super) const SAME_DIRECTION: f64 = 0.5;

/// Weights of directions closer than this share are a tie: a part per million, the lattice's
/// nanometre in a millimetre.
const SAME_WEIGHT: f64 = 1e-6;

/// The direction of a segment, in degrees, and its length.
pub(super) fn direction(a: Point, b: Point) -> (f64, f64) {
    ((b[1] - a[1]).atan2(b[0] - a[0]).to_degrees(), distance(a, b))
}

/// How far apart two directions of lines are, in degrees: within a half turn.
pub(crate) fn apart(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(180.0);
    d.min(180.0 - d)
}

/// Lines at `a` and at `b` degrees in the order ties go in, beside `frame`: nearest its axes
/// first, then nearest its first, then on from it towards its second.
pub(super) fn frame_order(a: f64, b: f64, frame: &Frame) -> Ordering {
    let (first, sense) = (frame.angle(), if frame.right_handed() { 1.0 } else { -1.0 });
    let key = |x: f64| {
        let d = apart(x, first);
        (d.min(90.0 - d), d, (sense * (x - first)).rem_euclid(180.0))
    };
    let (x, y) = (key(a), key(b));
    x.0.total_cmp(&y.0).then(x.1.total_cmp(&y.1)).then(x.2.total_cmp(&y.2))
}

/// A class of directions, a quarter turn counting as none, so of lines along them and across:
/// its directions span `width` degrees on from `from`, about their mean `angle`, in [0, 90).
#[derive(Debug, Clone, Copy)]
pub(super) struct Class {
    pub(super) angle: f64,
    from: f64,
    width: f64,
}

impl Class {
    /// How far a direction, in degrees, lies outside the class.
    fn gap(&self, a: f64) -> f64 {
        let t = (a - self.from).rem_euclid(90.0);
        if t <= self.width { 0.0 } else { (t - self.width).min(90.0 - t) }
    }
}

/// Directions in degrees with their weights, in classes: those closer than `SAME_DIRECTION`
/// chain into one, at the mean of their weights taken as `4θ` round the circle, a quarter turn
/// being a whole one. The chains start past the widest gap, which the directions place, not the
/// plate.
pub(super) fn classes(items: &[(f64, f64)]) -> Vec<Class> {
    let mut all: Vec<(f64, f64)> = items
        .iter()
        .filter(|(a, w)| a.is_finite() && w.is_finite() && *w > 0.0)
        .map(|&(a, w)| (a.rem_euclid(90.0), w))
        .collect();
    all.sort_by(|x, y| x.0.total_cmp(&y.0));
    let n = all.len();
    let at = |i: usize| all.get(i % n.max(1)).copied().unwrap_or_default();
    // From each direction on to the next, round the quarter turn.
    let gap = |i: usize| if n == 1 { 90.0 } else { (at(i + 1).0 - at(i).0).rem_euclid(90.0) };
    let Some(widest) = (0..n).max_by(|&i, &j| gap(i).total_cmp(&gap(j))) else { return Vec::new() };
    let close = |(from, width, sum): (f64, f64, [f64; 2])| Class {
        angle: (sum[1].atan2(sum[0]).to_degrees() / 4.0).rem_euclid(90.0),
        from,
        width,
    };
    let mut out: Vec<Class> = Vec::new();
    // The chain open: where it starts, how far on it reaches, and its weights' sum as `4θ`.
    let mut open: Option<(f64, f64, [f64; 2])> = None;
    for i in widest + 1..=widest + n {
        let (a, w) = at(i);
        let (sin, cos) = (4.0 * a).to_radians().sin_cos();
        open = match open {
            Some((from, _, [x, y])) if gap(i - 1) < SAME_DIRECTION => {
                Some((from, (a - from).rem_euclid(90.0), [x + w * cos, y + w * sin]))
            }
            done => {
                out.extend(done.map(close));
                Some((a, 0.0, [w * cos, w * sin]))
            }
        };
    }
    out.extend(open.map(close));
    out
}

/// The class of a direction in degrees: the nearest of `classes`, within `SAME_DIRECTION`.
pub(super) fn class_of(classes: &[Class], a: f64) -> Option<usize> {
    let gaps = classes.iter().map(|c| c.gap(a)).enumerate();
    gaps.filter(|(_, g)| *g < SAME_DIRECTION).min_by(|x, y| x.1.total_cmp(&y.1)).map(|(k, _)| k)
}

/// The classes that `items`, directions with their weights, run along most: the
/// `DIRECTIONS_TRIED` heaviest, and any as heavy as the last of them.
pub(super) fn heaviest(classes: &[Class], items: impl Iterator<Item = (f64, f64)>) -> Vec<usize> {
    let mut weights = vec![0.0; classes.len()];
    for (a, w) in items {
        if let Some(total) = class_of(classes, a).and_then(|k| weights.get_mut(k)) {
            *total += w;
        }
    }
    let mut ranked: Vec<(usize, f64)> = weights.into_iter().enumerate().filter(|(_, w)| *w > 0.0).collect();
    ranked.sort_by(|x, y| y.1.total_cmp(&x.1));
    let last = ranked.get(DIRECTIONS_TRIED - 1).map_or(0.0, |r| r.1 * (1.0 - SAME_WEIGHT));
    ranked.into_iter().take_while(|r| r.1 >= last).map(|r| r.0).collect()
}
