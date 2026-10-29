//! The top surfaces a leaf may follow, and their heights at the grid's vertices.
//!
//! A patch is a connected set of the mesh's up-facing triangles no steeper than the leaves may
//! be, joined across shared edges: one surface `S` over its shadow on the plate. Each vertex of
//! the grid under a patch triangle takes the triangle's height there, a height the leaf must
//! meet exactly (`exact`). The vertices around the shadow, of grid triangles it partly covers,
//! take the height of the nearest patch triangle's plane carried past its edge: there the leaf
//! is linear across the shadow's edge, so it meets `S` up to that edge too, if nothing else
//! claims those vertices.

use layercraft_geom::{Grid, Point, segment_distance};
use layercraft_mesh::{Mesh, Vertex};
use std::collections::HashMap;

/// A vertex of the grid with the height a patch asks the leaf to take there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pin {
    pub vertex: [i64; 2],
    pub height: f64,
    /// Under a patch triangle; otherwise beside the shadow, on a triangle's plane carried past
    /// its edge.
    pub exact: bool,
}

/// One top surface a leaf may follow.
#[derive(Debug, Clone, PartialEq)]
pub struct Patch {
    /// The area of its shadow on the plate, in mm².
    pub area: f64,
    pub pins: Vec<Pin>,
}

/// The outward normal of a triangle, not normalised.
fn normal([a, b, c]: [Vertex; 3]) -> Vertex {
    let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
    [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while let Some(&p) = parent.get(i) {
        if p == i {
            break;
        }
        let up = parent.get(p).copied().unwrap_or(p);
        if let Some(slot) = parent.get_mut(i) {
            *slot = up;
        }
        i = up;
    }
    i
}

/// The up-facing triangles no steeper than `max_slope` (radians), grouped into patches.
fn groups(mesh: &Mesh, max_slope: f64) -> Vec<Vec<[Vertex; 3]>> {
    let cos = max_slope.cos();
    let mut triangles: Vec<[Vertex; 3]> = Vec::new();
    let mut parent: Vec<usize> = Vec::new();
    for body in &mesh.bodies {
        let mut first: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in &body.triangles {
            let Some(corners) = body.corners(tri) else { continue };
            let n = normal(corners);
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            if !(len > 0.0 && n[2] > 0.0 && n[2] >= cos * len) {
                continue;
            }
            let id = triangles.len();
            triangles.push(corners);
            parent.push(id);
            for k in 0..3 {
                let (Some(&a), Some(&b)) = (tri.get(k), tri.get((k + 1) % 3)) else { continue };
                match first.get(&(a.min(b), a.max(b))) {
                    Some(&other) => {
                        let (ra, rb) = (find(&mut parent, id), find(&mut parent, other));
                        if let Some(slot) = parent.get_mut(ra) {
                            *slot = rb;
                        }
                    }
                    None => {
                        first.insert((a.min(b), a.max(b)), id);
                    }
                }
            }
        }
    }
    // Patches numbered by their first triangle, so the order is the mesh's.
    let mut number: HashMap<usize, usize> = HashMap::new();
    let mut out: Vec<Vec<[Vertex; 3]>> = Vec::new();
    for (id, corners) in triangles.into_iter().enumerate() {
        let root = find(&mut parent, id);
        let k = *number.entry(root).or_insert(out.len());
        if k == out.len() {
            out.push(Vec::new());
        }
        if let Some(patch) = out.get_mut(k) {
            patch.push(corners);
        }
    }
    out
}

/// The barycentric coordinates of `q` in the shadow of the triangle, when it has an area.
fn barycentric([a, b, c]: &[Vertex; 3], q: Point) -> Option<[f64; 3]> {
    let det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
    if det.abs() <= f64::MIN_POSITIVE || !det.is_finite() {
        return None;
    }
    let l1 = ((q[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (q[1] - a[1])) / det;
    let l2 = ((b[0] - a[0]) * (q[1] - a[1]) - (q[0] - a[0]) * (b[1] - a[1])) / det;
    Some([1.0 - l1 - l2, l1, l2])
}

/// The patches of the mesh no steeper than `max_slope` (radians), each with its pins on `grid`,
/// largest first.
pub fn patches(mesh: &Mesh, grid: &Grid, max_slope: f64) -> Vec<Patch> {
    let s = grid.cell;
    // Vertices of the grid triangles a shadow partly covers lie within a square's diagonal.
    let reach = s * std::f64::consts::SQRT_2 * (1.0 + 1e-9);
    let mut out = Vec::new();
    for triangles in groups(mesh, max_slope) {
        let mut area = 0.0;
        // Each vertex's height, and how far from the triangle that gave it.
        let mut best: HashMap<[i64; 2], (f64, f64)> = HashMap::new();
        for t in &triangles {
            let n = normal(*t);
            area += n[2] / 2.0;
            let (lo, hi) = t.iter().fold(([f64::MAX; 2], [f64::MIN; 2]), |(lo, hi), v| {
                ([lo[0].min(v[0]), lo[1].min(v[1])], [hi[0].max(v[0]), hi[1].max(v[1])])
            });
            let (glo, ghi) =
                (grid.coords([lo[0] - reach, lo[1] - reach]), grid.coords([hi[0] + reach, hi[1] + reach]));
            if !(glo.iter().chain(&ghi).all(|v| v.is_finite()) && (ghi[0] - glo[0]) * (ghi[1] - glo[1]) < 1e8)
            {
                continue;
            }
            for j in glo[1].ceil() as i64..=ghi[1].floor() as i64 {
                for i in glo[0].ceil() as i64..=ghi[0].floor() as i64 {
                    let q = grid.vertex(i, j);
                    let Some(l) = barycentric(t, q) else { continue };
                    let height = l[0] * t[0][2] + l[1] * t[1][2] + l[2] * t[2][2];
                    let off = if l.iter().all(|&x| x >= -1e-12) {
                        0.0
                    } else {
                        let edge = |a: &Vertex, b: &Vertex| segment_distance(q, [a[0], a[1]], [b[0], b[1]]);
                        edge(&t[0], &t[1]).min(edge(&t[1], &t[2])).min(edge(&t[2], &t[0]))
                    };
                    if off > reach || !height.is_finite() {
                        continue;
                    }
                    let slot = best.entry([i, j]).or_insert((off, height));
                    // The nearest triangle wins; under two at once, the higher.
                    if off < slot.0 || (off == 0.0 && slot.0 == 0.0 && height > slot.1) {
                        *slot = (off, height);
                    }
                }
            }
        }
        let mut pins: Vec<Pin> = best
            .into_iter()
            .map(|(vertex, (off, height))| Pin { vertex, height, exact: off == 0.0 })
            .collect();
        pins.sort_by_key(|p| p.vertex);
        if pins.iter().any(|p| p.exact) {
            out.push(Patch { area, pins });
        }
    }
    out.sort_by(|a, b| b.area.total_cmp(&a.area));
    out
}
