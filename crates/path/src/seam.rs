//! Where a closed bead starts and which way it runs, from the part's own geometry. A seam goes
//! to the concave corner nearest a reference point, else to the vertex nearest it; the first
//! layer of a solid has no seam below, and takes the point of its outline farthest along the
//! solid's frame. A loop of a mirrored part runs the other way. Nothing here names a plate
//! direction, so a part turned, moved or mirrored has its seams turned, moved or mirrored.

use crate::frame::Frame;
use layercraft_geom::Point;
use std::collections::HashMap;

/// The last seam of each solid, which the layer above's seams follow.
#[derive(Default)]
pub(crate) struct Seams(HashMap<usize, Point>);

impl Seams {
    pub fn of(&self, solid: usize) -> Option<Point> {
        self.0.get(&solid).copied()
    }

    pub fn keep(&mut self, solid: usize, seam: Point) {
        self.0.insert(solid, seam);
    }
}

fn at(loop_: &[Point], k: usize) -> Point {
    loop_.get(k % loop_.len().max(1)).copied().unwrap_or([0.0, 0.0])
}

fn dist(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The point at arc length `d` along the loop from vertex `k`, forward or backward.
fn along(loop_: &[Point], k: usize, d: f64, forward: bool) -> Point {
    let n = loop_.len();
    let mut here = at(loop_, k);
    let mut index = k;
    let mut left = d;
    for _ in 0..n {
        let next = if forward { index + 1 } else { index + n - 1 } % n;
        let to = at(loop_, next);
        let step = dist(here, to);
        if step >= left {
            let t = if step > 0.0 { left / step } else { 0.0 };
            return [here[0] + t * (to[0] - here[0]), here[1] + t * (to[1] - here[1])];
        }
        left -= step;
        here = to;
        index = next;
    }
    here
}

/// How far vertex `k` lies to the left of the chord between the points `width / 2` of arc
/// before and after it: positive on the material's side of a loop with material on its left.
fn depth(loop_: &[Point], k: usize, width: f64) -> f64 {
    let (a, b, v) = (along(loop_, k, width / 2.0, false), along(loop_, k, width / 2.0, true), at(loop_, k));
    let chord = dist(a, b);
    if chord <= 0.0 {
        return 0.0;
    }
    ((b[0] - a[0]) * (v[1] - a[1]) - (b[1] - a[1]) * (v[0] - a[0])) / chord
}

/// The concave corners of a loop with material on its left, as vertex indices.
///
/// A vertex is concave when the loop, within its own bead width `width` of it, strays from its
/// chord on the material's side by more than the printer's resolution `resolution`: a corner is
/// where the wall turns sharper than the printer can tell from a curve, a fillet tighter than
/// about `width² / (8 resolution)` (1.7 mm on the X2D). Of consecutive concave vertices, only
/// the tip of the corner, where the strays peak, is kept.
fn concave_corners(loop_: &[Point], width: f64, resolution: f64) -> Vec<usize> {
    let depths: Vec<f64> = (0..loop_.len()).map(|k| depth(loop_, k, width)).collect();
    let n = depths.len();
    let get = |k: usize| depths.get(k % n.max(1)).copied().unwrap_or(0.0);
    (0..n).filter(|&k| get(k) > resolution && get(k) > get(k + n - 1) && get(k) >= get(k + 1)).collect()
}

/// The index of the vertex a loop with material on its left starts at, for a seam near
/// `reference`: the concave corner nearest it, else the vertex nearest it.
pub(crate) fn seam_index(loop_: &[Point], reference: Point, width: f64, resolution: f64) -> usize {
    let nearest = |candidates: Vec<usize>| {
        candidates
            .into_iter()
            .min_by(|&a, &b| dist(at(loop_, a), reference).total_cmp(&dist(at(loop_, b), reference)))
    };
    nearest(concave_corners(loop_, width, resolution))
        .or_else(|| nearest((0..loop_.len()).collect()))
        .unwrap_or(0)
}

/// The point of `outline` farthest along the frame's first axis, ties to the farthest along its
/// second: a solid's first seam.
pub(crate) fn farthest_along(outline: &[Point], frame: &Frame, tolerance: f64) -> Point {
    let (axis, second) = (frame.axis(), frame.second());
    let along = |p: &Point, d: [f64; 2]| p[0] * d[0] + p[1] * d[1];
    let top = outline.iter().map(|p| along(p, axis)).fold(f64::MIN, f64::max);
    outline
        .iter()
        .filter(|p| along(p, axis) >= top - tolerance)
        .max_by(|a, b| along(a, second).total_cmp(&along(b, second)))
        .copied()
        .unwrap_or([0.0, 0.0])
}

/// A loop with material on its left, started at its seam near `reference` and run the way a
/// part of that handedness runs.
pub(crate) fn seamed(
    mut loop_: Vec<Point>,
    reference: Point,
    width: f64,
    resolution: f64,
    right_handed: bool,
) -> Vec<Point> {
    let start = seam_index(&loop_, reference, width, resolution);
    let start = start.min(loop_.len());
    loop_.rotate_left(start);
    if !right_handed {
        loop_.get_mut(1..).into_iter().for_each(|rest| rest.reverse());
    }
    loop_
}
