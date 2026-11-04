//! A region's own lines, from its moments: they turn, move and mirror with it.
//!
//! The moments are Green's sums over each contour's edges, outer contours counting positive and
//! holes negative: `∫∫ x^p y^q` from the vertices exactly. The second about the centroid,
//! `∫∫ (x + iy)²`, sets the axis of greatest spread. Where it vanishes the region spreads alike
//! every way, as a square or a disc does, and its boundary's own lines set its axes: the first
//! of the moments `∮ e^{2imθ} ds` of its edges' directions `θ` that does not vanish, `m` lines
//! `π/m` apart, a square's two sides (`m = 2`), a triangle's or a hexagon's three (`m = 3`).
//! Where they all vanish, as for a disc, nothing of the region's own sets a line. A moment
//! vanishes when moving the boundary by `RESOLUTION_MM` could cancel it: a displacement `η`
//! moves the second by `∮ (x + iy)² η ds`, the edges' `k`-th by `(k² − 1) ∮ κ e^{ikθ} η ds`
//! to first order, `κ` the boundary's curvature, its corners' turns. The third moments,
//! `∫∫ (p·u)³`, tell which way along a line the region skews; the displacement moves one by
//! `∮ (p·u)³ η ds`, less than `RESOLUTION_MM` times `∮ |p·u|³ ds`, which the boundary's second
//! and fourth moments bound (Cauchy–Schwarz): a skew along `u` weighs only the spread along `u`.

use crate::{Contour, Point, RESOLUTION_MM, edges};
use std::f64::consts::PI;

/// The most lines sought among the edges' moments: past a degree apart, a region's lines are
/// a disc's.
const MAX_LINES: u32 = 180;

/// What a region's moments tell of it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Moments {
    /// Its area, each shape weighing what is given with it.
    pub area: f64,
    /// Its centroid.
    pub centroid: Point,
    /// Its own lines through the centroid, where its moments set any.
    pub axes: Option<Axes>,
    /// `∫∫ x³, x²y, xy², y³` about the centroid.
    third: [f64; 4],
    /// `∮ x², xy, y²` and `∮ x⁴, x³y, x²y², xy³, y⁴` along the boundary about the centroid.
    boundary: [f64; 8],
}

/// Lines through one point, `count` of them `π/count` apart, the first along `first`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Axes {
    pub first: [f64; 2],
    pub count: u32,
}

impl Axes {
    /// A unit vector along each line, from the first anticlockwise.
    pub fn lines(&self) -> impl Iterator<Item = [f64; 2]> {
        let (start, count) = (self.first[1].atan2(self.first[0]), self.count.max(1));
        (0..count).map(move |j| {
            let angle = start + PI * j as f64 / count as f64;
            [angle.cos(), angle.sin()]
        })
    }
}

impl Moments {
    /// The moments of shapes, each weighing what is given with it; none without area.
    pub fn of<'a>(shapes: impl Iterator<Item = (&'a [Contour], f64)> + Clone) -> Option<Moments> {
        let contours = || shapes.clone().flat_map(|(shape, w)| shape.iter().map(move |c| (c, w)));
        let about = contours().find_map(|(c, _)| c.first().copied())?;
        // ∫∫ 1, x, y, x², xy, y², x³, x²y, xy², y³ and ∮ 1, x, y, x² + y² about `about`, and how
        // far the boundary turns.
        let (mut s, mut b, mut turning) = ([0.0f64; 10], [0.0f64; 4], 0.0f64);
        for (contour, w) in contours() {
            for (p, q) in edges(contour) {
                let (x0, y0, x1, y1) = (p[0] - about[0], p[1] - about[1], q[0] - about[0], q[1] - about[1]);
                let c = w * (x0 * y1 - x1 * y0);
                let (xx, yy) = (x0 * x0 + x0 * x1 + x1 * x1, y0 * y0 + y0 * y1 + y1 * y1);
                let terms = [
                    c / 2.0,
                    c * (x0 + x1) / 6.0,
                    c * (y0 + y1) / 6.0,
                    c * xx / 12.0,
                    c * (x0 * y1 + 2.0 * x0 * y0 + 2.0 * x1 * y1 + x1 * y0) / 24.0,
                    c * yy / 12.0,
                    c * (x0 + x1) * (x0 * x0 + x1 * x1) / 20.0,
                    c * (3.0 * x0 * x0 * y0 + 2.0 * x0 * x1 * y0 + x1 * x1 * y0) / 60.0
                        + c * (x0 * x0 * y1 + 2.0 * x0 * x1 * y1 + 3.0 * x1 * x1 * y1) / 60.0,
                    c * (3.0 * y0 * y0 * x0 + 2.0 * y0 * y1 * x0 + y1 * y1 * x0) / 60.0
                        + c * (y0 * y0 * x1 + 2.0 * y0 * y1 * x1 + 3.0 * y1 * y1 * x1) / 60.0,
                    c * (y0 + y1) * (y0 * y0 + y1 * y1) / 20.0,
                ];
                for (sum, term) in s.iter_mut().zip(terms) {
                    *sum += term;
                }
                let l = w * (x1 - x0).hypot(y1 - y0);
                for (sum, term) in
                    b.iter_mut().zip([l, l * (x0 + x1) / 2.0, l * (y0 + y1) / 2.0, l * (xx + yy) / 3.0])
                {
                    *sum += term;
                }
            }
            turning += w * turns(contour);
        }
        let [a, sx, sy, sxx, sxy, syy, sxxx, sxxy, sxyy, syyy] = s;
        if !(a.is_finite() && a > 0.0) {
            return None;
        }
        let (cx, cy) = (sx / a, sy / a);
        let (mxx, mxy, myy) = (sxx - cx * sx, sxy - cx * sy, syy - cy * sy);
        let third = [
            sxxx - 3.0 * cx * sxx + 2.0 * cx * cx * sx,
            sxxy - cy * sxx - 2.0 * cx * sxy + 2.0 * cx * cy * sx,
            sxyy - cx * syy - 2.0 * cy * sxy + 2.0 * cx * cy * sy,
            syyy - 3.0 * cy * syy + 2.0 * cy * cy * sy,
        ];
        let centroid = [about[0] + cx, about[1] + cy];
        // ∮ r² ds about the centroid; the boundary's second and fourth moments, exact by
        // Gauss–Legendre's three points, exact to the fifth degree.
        let ring = b[3] - 2.0 * (cx * b[1] + cy * b[2]) + (cx * cx + cy * cy) * b[0];
        let gauss =
            [(0.5 - 0.15f64.sqrt(), 5.0 / 18.0), (0.5, 8.0 / 18.0), (0.5 + 0.15f64.sqrt(), 5.0 / 18.0)];
        let mut boundary = [0.0f64; 8];
        for (contour, w) in contours() {
            for (p, q) in edges(contour) {
                let l = w * (q[0] - p[0]).hypot(q[1] - p[1]);
                for (t, weight) in gauss {
                    let x = p[0] + t * (q[0] - p[0]) - centroid[0];
                    let y = p[1] + t * (q[1] - p[1]) - centroid[1];
                    let (xx, yy) = (x * x, y * y);
                    let terms = [xx, x * y, yy, xx * xx, xx * x * y, xx * yy, x * y * yy, yy * yy];
                    for (sum, term) in boundary.iter_mut().zip(terms) {
                        *sum += l * weight * term;
                    }
                }
            }
        }
        let spread = (mxx - myy).hypot(2.0 * mxy);
        let axes = if spread > RESOLUTION_MM * ring {
            let angle = 0.5 * (2.0 * mxy).atan2(mxx - myy);
            Some(Axes { first: [angle.cos(), angle.sin()], count: 1 })
        } else {
            edge_axes(contours(), b[0], turning)
        };
        Some(Moments { area: a, centroid, axes, third, boundary })
    }

    /// `∫∫ (p·u)³` about the centroid: how far the region skews along the unit `u`.
    pub fn skew(&self, u: [f64; 2]) -> f64 {
        let ([a, b, c, d], [x, y]) = (self.third, u);
        a * x * x * x + 3.0 * b * x * x * y + 3.0 * c * x * y * y + d * y * y * y
    }

    /// How far moving the boundary by `RESOLUTION_MM` could move the skew along the unit `u`:
    /// `∮ |p·u|³ ds` times it, at most `√(∮ (p·u)² ds ∮ (p·u)⁴ ds)`.
    fn skew_tie(&self, u: [f64; 2]) -> f64 {
        let ([a, b, c, d, e, f, g, h], [x, y]) = (self.boundary, u);
        let second = a * x * x + 2.0 * b * x * y + c * y * y;
        let (xx, yy) = (x * x, y * y);
        let fourth =
            d * xx * xx + 4.0 * e * xx * x * y + 6.0 * f * xx * yy + 4.0 * g * x * y * yy + h * yy * yy;
        RESOLUTION_MM * (second.max(0.0) * fourth.max(0.0)).sqrt()
    }

    /// Of `directions`, the one the region skews the most toward: a later one only where it
    /// skews more by more than the boundary's resolution could make either skew.
    pub fn most_skewed(&self, directions: impl Iterator<Item = [f64; 2]>) -> Option<[f64; 2]> {
        let mut best: Option<([f64; 2], f64)> = None;
        for u in directions {
            let s = self.skew(u);
            if best.is_none_or(|(v, b)| s > b + self.skew_tie(u) + self.skew_tie(v)) {
                best = Some((u, s));
            }
        }
        best.map(|(u, _)| u)
    }
}

/// How far a contour's boundary turns, its corners' turns added whichever way they turn.
fn turns(contour: &[Point]) -> f64 {
    let directions: Vec<[f64; 2]> = edges(contour)
        .map(|(p, q)| [q[0] - p[0], q[1] - p[1]])
        .filter(|d| d[0] != 0.0 || d[1] != 0.0)
        .collect();
    let next = directions.iter().cycle().skip(1);
    directions
        .iter()
        .zip(next)
        .map(|(u, v)| (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]).abs())
        .sum()
}

/// The boundary's own lines: the first moment `∮ e^{2imθ} ds` of its edges' directions that no
/// displacement within `RESOLUTION_MM` could cancel, given its length and how far it turns.
fn edge_axes<'a>(
    contours: impl Iterator<Item = (&'a Contour, f64)>,
    length: f64,
    turning: f64,
) -> Option<Axes> {
    // Each edge's weighted length, the square of its direction as a complex number, and the
    // power of that reached.
    let mut terms: Vec<(f64, [f64; 2], [f64; 2])> = Vec::new();
    for (contour, w) in contours {
        for (p, q) in edges(contour) {
            let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
            let l = dx.hypot(dy);
            if l > 0.0 {
                let (c, s) = (dx / l, dy / l);
                terms.push((w * l, [c * c - s * s, 2.0 * c * s], [1.0, 0.0]));
            }
        }
    }
    for m in 1..=MAX_LINES {
        let k = 2.0 * m as f64;
        let tie = (k * k - 1.0) * RESOLUTION_MM * turning;
        if tie >= length {
            return None;
        }
        let mut sum = [0.0f64; 2];
        for (l, z, power) in &mut terms {
            *power = [power[0] * z[0] - power[1] * z[1], power[0] * z[1] + power[1] * z[0]];
            sum = [sum[0] + *l * power[0], sum[1] + *l * power[1]];
        }
        if sum[0].hypot(sum[1]) > tie {
            let angle = sum[1].atan2(sum[0]) / k;
            return Some(Axes { first: [angle.cos(), angle.sin()], count: m });
        }
    }
    None
}
