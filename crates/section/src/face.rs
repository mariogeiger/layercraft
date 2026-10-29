//! The nozzle's flat face as the grid resolves it: how far `d` may fall from a leaf to the one
//! below, to each offset, so that the leaf below never stands above the face printing over it.
//!
//! The face asks `d_{k−1}(y) ≤ d_k(x) + h` for `|y − x| ≤ ρ`. The leaves are linear on the grid's
//! triangles, and for such fields the bounds `d_{k−1}(u + δ) − d_k(u) ≤ W(δ)` at the vertices give
//! `d_{k−1}(y) − d_k(x) ≤ W̃(y − x)` everywhere, `W̃` the linear interpolation of `W` on the same
//! triangles, and nothing more: over two triangles and a triangle of offsets the difference less
//! `W̃` is affine, and the polytope they bound has only pairs of vertices for corners, the sides
//! lying along three directions any two of which span the lattice. The face holds exactly when
//! `W̃ ≤ h` over its disc. `W(δ) = h − t + fall(|δ|)`, `t` the thinnest slab, keeps every pair of
//! parallel planes leaning up to `s` with slabs `t ≥ max(t_min, s·R)` between them, a face of
//! radius `R`: `fall(ℓ) = min(s·ℓ, max(t·ℓ/R, t + s·(ℓ − R)))`. At `R = ρ` its interpolation
//! overshoots on the disc, the function bending between vertices; `R` is the least radius that
//! does not, found by bisection, the test exact: an affine function is greatest over a triangle
//! and a disc at a corner inside the disc, where a side crosses the circle, or where the circle
//! meets its gradient. Offsets whose weight a nearer one and the slope imply are dropped.

use std::collections::HashMap;
use std::f64::consts::SQRT_2;

/// Room for rounding when weights are compared, in mm.
const SLACK: f64 = 1e-9;
/// Halvings of the bisection on the face's radius: far below a nanometre.
const BISECTIONS: usize = 48;

/// The edges from a leaf to the one below.
#[derive(Debug, Clone)]
pub(crate) struct Laps {
    /// Offsets in cells, `[0, 0]` among them, and how far `d` may fall to each, in mm.
    pub edges: Vec<([i64; 2], f64)>,
    /// The face's radius as the grid resolves it, `R`, in mm; 0 without a face.
    pub reach: f64,
    slope: f64,
    thin: f64,
}

impl Laps {
    /// How much further than straight down `d` may fall to a point `l` mm off, at least: every
    /// edge's weight is `h − t + fall(|δ|)`.
    pub fn fall(&self, l: f64) -> f64 {
        fall(l, self.slope, self.thin, self.reach)
    }

    /// The least rate, in mm per mm, at which crossing to the leaf below lets `d` fall sideways.
    pub fn rate(&self) -> f64 {
        if self.reach > 0.0 { self.slope.min(self.thin / self.reach) } else { self.slope }
    }
}

fn fall(l: f64, slope: f64, thin: f64, reach: f64) -> f64 {
    if reach <= 0.0 {
        return slope * l;
    }
    (slope * l).min((thin * l / reach).max(thin + slope * (l - reach)))
}

/// The edges for a face of `face` mm radius on a grid of `cell` mm, leaves leaning up to `slope`,
/// `d` falling by `down` straight down, the thinnest slab `thin`.
pub(crate) fn laps(cell: f64, face: f64, slope: f64, down: f64, thin: f64) -> Laps {
    let straight = Laps { edges: vec![([0, 0], down)], reach: 0.0, slope, thin };
    if face <= 0.0 {
        return straight;
    }
    // Every triangle that meets the disc has its corners within this of its centre.
    let span = face + SQRT_2 * cell;
    let n = (span / cell).ceil() as i64;
    let offsets: Vec<[i64; 2]> = (-n..=n)
        .flat_map(|j| (-n..=n).map(move |i| [i, j]))
        .filter(|&[i, j]| (i as f64).hypot(j as f64) * cell <= span)
        .collect();
    let weights = |reach: f64| -> HashMap<[i64; 2], f64> {
        offsets
            .iter()
            .map(|&[i, j]| ([i, j], down + fall((i as f64).hypot(j as f64) * cell, slope, thin, reach)))
            .collect()
    };
    let sound = |reach: f64| disc_max(&weights(reach), face / cell) <= down + thin;
    // At `span` every weight is at most `h`, so is their interpolation.
    let (mut lo, mut hi) = (face, span);
    if sound(lo) {
        hi = lo;
    } else {
        for _ in 0..BISECTIONS {
            let mid = (lo + hi) / 2.0;
            if sound(mid) { hi = mid } else { lo = mid }
        }
    }
    let all = weights(hi);
    let octile = |[i, j]: [i64; 2]| {
        let (a, b) = (i.abs().max(j.abs()) as f64, i.abs().min(j.abs()) as f64);
        (a + (SQRT_2 - 1.0) * b) * cell * slope
    };
    let mut edges: Vec<([i64; 2], f64)> = all
        .iter()
        .filter(|&(&o, &w)| {
            !all.iter().any(|(&p, &v)| p != o && v + octile([o[0] - p[0], o[1] - p[1]]) < w - SLACK)
        })
        .map(|(&o, &w)| (o, w))
        .collect();
    edges.sort_by_key(|a| a.0);
    Laps { edges, reach: hi, slope, thin }
}

/// The greatest value, over the disc of `radius` cells about the origin, of the interpolation of
/// `w` on the grid's triangles; infinite where a triangle meeting the disc lacks a corner.
fn disc_max(w: &HashMap<[i64; 2], f64>, radius: f64) -> f64 {
    let n = radius.ceil() as i64 + 1;
    let mut best = f64::NEG_INFINITY;
    for j in -n..n {
        for i in -n..n {
            for corners in [[[i, j], [i + 1, j], [i + 1, j + 1]], [[i, j], [i, j + 1], [i + 1, j + 1]]] {
                best = best.max(triangle_max(w, corners, radius));
            }
        }
    }
    best
}

/// The greatest value of the interpolation of `w` over the triangle and the disc together, or
/// minus infinity when they do not meet.
fn triangle_max(w: &HashMap<[i64; 2], f64>, corners: [[i64; 2]; 3], radius: f64) -> f64 {
    let p = corners.map(|[i, j]| [i as f64, j as f64]);
    let [p0, p1, p2] = p;
    let (e1, e2) = ([p1[0] - p0[0], p1[1] - p0[1]], [p2[0] - p0[0], p2[1] - p0[1]]);
    let det = e1[0] * e2[1] - e1[1] * e2[0];
    // Barycentric coordinates of a point: those of `p1` and `p2`, the rest `p0`'s.
    let bary = |q: [f64; 2]| {
        let (x, y) = (q[0] - p0[0], q[1] - p0[1]);
        let b1 = (x * e2[1] - y * e2[0]) / det;
        let b2 = (e1[0] * y - e1[1] * x) / det;
        [1.0 - b1 - b2, b1, b2]
    };
    let mut points: Vec<[f64; 3]> = Vec::new();
    let inside = |q: [f64; 2]| q[0].hypot(q[1]) <= radius;
    for (k, &q) in p.iter().enumerate() {
        if inside(q) {
            let mut b = [0.0; 3];
            if let Some(slot) = b.get_mut(k) {
                *slot = 1.0;
            }
            points.push(b);
        }
    }
    for (a, b) in [(0, 1), (1, 2), (2, 0)] {
        let (Some(&pa), Some(&pb)) = (p.get(a), p.get(b)) else { continue };
        let d = [pb[0] - pa[0], pb[1] - pa[1]];
        // `|pa + τ d| = radius`
        let (qa, qb, qc) = (
            d[0] * d[0] + d[1] * d[1],
            2.0 * (pa[0] * d[0] + pa[1] * d[1]),
            pa[0] * pa[0] + pa[1] * pa[1] - radius * radius,
        );
        let disc = qb * qb - 4.0 * qa * qc;
        if disc < 0.0 {
            continue;
        }
        for tau in [(-qb - disc.sqrt()) / (2.0 * qa), (-qb + disc.sqrt()) / (2.0 * qa)] {
            if (0.0..=1.0).contains(&tau) {
                points.push(bary([pa[0] + tau * d[0], pa[1] + tau * d[1]]));
            }
        }
    }
    let at = |k: usize| corners.get(k).and_then(|c| w.get(c)).copied();
    let value = |b: [f64; 3]| -> f64 {
        let mut v = 0.0;
        for (k, &bk) in b.iter().enumerate() {
            match at(k) {
                Some(x) => v += bk * x,
                None => return f64::INFINITY,
            }
        }
        v
    };
    // Where the circle meets the gradient: the affine function's greatest over the whole disc.
    if let (Some(v0), Some(v1), Some(v2)) = (at(0), at(1), at(2)) {
        let (d1, d2) = (v1 - v0, v2 - v0);
        let g = [(d1 * e2[1] - d2 * e1[1]) / det, (e1[0] * d2 - e2[0] * d1) / det];
        let norm = g[0].hypot(g[1]);
        let q = if norm > 0.0 { [radius * g[0] / norm, radius * g[1] / norm] } else { [radius, 0.0] };
        let b = bary(q);
        if b.iter().all(|&x| x >= -1e-12) {
            points.push(b);
        }
    }
    points.into_iter().map(value).fold(f64::NEG_INFINITY, f64::max)
}
