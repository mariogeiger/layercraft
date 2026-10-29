//! Where a layer's area lies on the rolls' grid, and how its walls hold them.
//!
//! Beyond a `margin` around the area the field is damped back to nothing, `hold` strong a node
//! farther on: walls no flux crosses, which rolls meet at right angles, as convection rolls meet
//! the sides of their pan. Near a wall the anchoring turns them square to it, or off square by a
//! tilt: it weighs the field's slope along the wall's normal turned by the tilt, the most on the
//! wall and fading as a Gaussian `reach` wide. Nodes inside go by the even-odd rule along each
//! row; each node's nearest wall point is sought among the edges within three reaches of it.

use super::Nodes;
use layercraft_geom::Point;

/// What the walls do at each node of the grid, row after row.
pub(super) struct Walls {
    /// How hard the field is damped back to nothing.
    pub damping: Vec<f64>,
    /// How much the anchoring weighs.
    pub weight: Vec<f64>,
    /// The way the lines should run: along the walls' normal, turned by the tilt.
    pub along: Vec<[f64; 2]>,
}

/// The nodes of each row lying inside `contours`, by the even-odd rule.
fn inside(nodes: &Nodes, contours: &[Vec<Point>]) -> Vec<bool> {
    let mut inside = vec![false; nodes.len()];
    let mut xs = Vec::new();
    for j in 0..nodes.ny {
        let y = nodes.origin[1] + j as f64 * nodes.h;
        xs.clear();
        for contour in contours {
            for (k, a) in contour.iter().enumerate() {
                let Some(b) = contour.get((k + 1) % contour.len()) else { continue };
                if (a[1] <= y) != (b[1] <= y) {
                    xs.push(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]));
                }
            }
        }
        xs.sort_by(f64::total_cmp);
        for &[x0, x1] in xs.as_chunks::<2>().0 {
            let from = ((x0 - nodes.origin[0]) / nodes.h).ceil().max(0.0) as usize;
            let to = (((x1 - nodes.origin[0]) / nodes.h).ceil().max(0.0) as usize).min(nodes.nx);
            for i in from..to {
                if let Some(cell) = inside.get_mut(j * nodes.nx + i) {
                    *cell = true;
                }
            }
        }
    }
    inside
}

impl Walls {
    /// The walls of the area bounded by `contours`, in the frame, tilted `tilt` radians.
    pub fn new(
        nodes: &Nodes,
        contours: &[Vec<Point>],
        margin: f64,
        reach: f64,
        hold: f64,
        tilt: f64,
    ) -> Walls {
        let inside = inside(nodes, contours);
        let band = (3.0 * reach).max(margin + nodes.h);
        let (mut distance, mut nearest) = (vec![f64::INFINITY; nodes.len()], vec![[0.0; 2]; nodes.len()]);
        for contour in contours {
            for (k, a) in contour.iter().enumerate() {
                let Some(b) = contour.get((k + 1) % contour.len()) else { continue };
                let lo = nodes.place([a[0].min(b[0]) - band, a[1].min(b[1]) - band]);
                let hi = nodes.place([a[0].max(b[0]) + band, a[1].max(b[1]) + band]);
                let d = [b[0] - a[0], b[1] - a[1]];
                let dd = d[0] * d[0] + d[1] * d[1];
                for j in lo[1].max(0)..=hi[1].min(nodes.ny as i64 - 1) {
                    for i in lo[0].max(0)..=hi[0].min(nodes.nx as i64 - 1) {
                        let p = nodes.node(i as usize, j as usize);
                        let u = if dd > 0.0 {
                            (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / dd).clamp(0.0, 1.0)
                        } else {
                            0.0
                        };
                        let q = [a[0] + u * d[0], a[1] + u * d[1]];
                        let r = (p[0] - q[0]).hypot(p[1] - q[1]);
                        let n = j as usize * nodes.nx + i as usize;
                        if let (Some(best), Some(near)) = (distance.get_mut(n), nearest.get_mut(n))
                            && r < *best
                        {
                            *best = r;
                            *near = q;
                        }
                    }
                }
            }
        }
        let (c, s) = (tilt.cos(), tilt.sin());
        let mut walls = Walls {
            damping: Vec::with_capacity(nodes.len()),
            weight: Vec::with_capacity(nodes.len()),
            along: Vec::with_capacity(nodes.len()),
        };
        for (n, ((&r, q), &is_in)) in distance.iter().zip(&nearest).zip(&inside).enumerate() {
            let p = nodes.node(n % nodes.nx, n / nodes.nx);
            let damping = if is_in || r <= margin { 0.0 } else { hold * ((r - margin) / nodes.h).min(1.0) };
            let weight = if r.is_finite() { (-0.5 * (r / reach).powi(2)).exp() } else { 0.0 };
            // The normal points into the area: from the wall inside, toward it outside.
            let inward = if is_in { 1.0 } else { -1.0 };
            let normal = if r > 0.0 && r.is_finite() {
                [inward * (p[0] - q[0]) / r, inward * (p[1] - q[1]) / r]
            } else {
                [0.0, 0.0]
            };
            walls.damping.push(damping);
            walls.weight.push(weight);
            walls.along.push([c * normal[0] - s * normal[1], s * normal[0] + c * normal[1]]);
        }
        walls
    }

    /// Whether the field is free at each node: not damped.
    pub fn live(&self) -> Vec<bool> {
        self.damping.iter().map(|&m| m == 0.0).collect()
    }
}
