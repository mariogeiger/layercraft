//! Cutting triangles by a surface `z = level(x, y)` and stitching the cuts into oriented loops.
//!
//! Each vertex lies above the surface or not by the sign of its height over it, `z − level`, a
//! number of the vertex alone. Where that height is linear over a triangle, as for a plane or
//! for a leaf over a mesh refined along its grid (`refine`), the surface cuts the triangle along
//! the segment between the points where it crosses two of its edges.

use layercraft_geom::{GRID_MM, Point};
use layercraft_mesh::{Body, Mesh, Vertex};
use std::collections::HashMap;

/// An edge of a body, its vertex indices in increasing order, tagged with the body.
type EdgeKey = (usize, u32, u32);

/// How close to the line through its neighbours a loop's point may lie and still be dropped, in
/// mm: a tenth of the lattice the regions are taken on, so dropping it changes no region. Where a
/// refined mesh's triangles share a plane and the leaf over them is linear, the cut runs
/// straight across their edges, through points that add nothing.
const STRAIGHT_MM: f64 = GRID_MM / 10.0;

/// The loop without the points that lie on the line through their neighbours.
fn straightened(contour: Vec<Point>) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::with_capacity(contour.len());
    let off = |a: Point, p: Point, b: Point| {
        let (d, q) = ([b[0] - a[0], b[1] - a[1]], [p[0] - a[0], p[1] - a[1]]);
        let len = d[0].hypot(d[1]);
        if len > 0.0 { (d[0] * q[1] - d[1] * q[0]).abs() / len } else { q[0].hypot(q[1]) }
    };
    for p in contour {
        while out.len() >= 2 {
            let (Some(&a), Some(&m)) = (out.get(out.len() - 2), out.last()) else { break };
            if off(a, m, p) <= STRAIGHT_MM {
                out.pop();
            } else {
                break;
            }
        }
        out.push(p);
    }
    // Round the seam: the last points against the first.
    while out.len() > 3 {
        let (Some(&a), Some(&m), Some(&b)) = (out.get(out.len() - 2), out.last(), out.first()) else { break };
        if off(a, m, b) <= STRAIGHT_MM {
            out.pop();
            continue;
        }
        let (Some(&a), Some(&m), Some(&b)) = (out.last(), out.first(), out.get(1)) else { break };
        if off(a, m, b) <= STRAIGHT_MM {
            out.remove(0);
            continue;
        }
        break;
    }
    out
}

/// The point where the surface crosses an edge, from the heights `over` of the body's vertices
/// above it, always computed from the lower index so that both triangles of the edge get
/// bit-identical points.
fn crossing(body: &Body, over: &[f64], a: u32, b: u32) -> Option<Point> {
    let (a, b) = (a.min(b), a.max(b));
    let pa = body.vertices.get(a as usize)?;
    let pb = body.vertices.get(b as usize)?;
    let (ga, gb) = (over.get(a as usize)?, over.get(b as usize)?);
    let t = ga / (ga - gb);
    if !t.is_finite() {
        return None;
    }
    Some([pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1])])
}

/// The oriented loops where the surface `z = level(vertex)` cuts the mesh, and the number of
/// chains that do not close. Loops wind counterclockwise around material, seen from above.
pub fn section_loops(mesh: &Mesh, level: impl Fn(Vertex) -> f64) -> (Vec<Vec<Point>>, usize) {
    // For each crossing edge: the point, and the segments leaving it.
    let mut points: HashMap<EdgeKey, Point> = HashMap::new();
    let mut next: HashMap<EdgeKey, Vec<EdgeKey>> = HashMap::new();
    for (bi, body) in mesh.bodies.iter().enumerate() {
        let over: Vec<f64> = body.vertices.iter().map(|&v| v[2] - level(v)).collect();
        for tri in &body.triangles {
            let Some(g) = tri.iter().map(|&i| over.get(i as usize).copied()).collect::<Option<Vec<f64>>>()
            else {
                continue;
            };
            let up = [g.first(), g.get(1), g.get(2)].map(|v| v.is_some_and(|&v| v >= 0.0));
            if up[0] == up[1] && up[1] == up[2] {
                continue;
            }
            // Walking the triangle a→b→c (counterclockwise from outside), the material lies on the
            // inner side; the cut runs from the edge entering the upper part to the edge leaving it,
            // which puts the material on its left seen from above.
            let mut enter = None;
            let mut leave = None;
            for k in 0..3 {
                let (i, j) = (k, (k + 1) % 3);
                let (Some(&ui), Some(&uj)) = (up.get(i), up.get(j)) else { continue };
                if ui == uj {
                    continue;
                }
                let (Some(&a), Some(&b)) = (tri.get(i), tri.get(j)) else { continue };
                let key = (bi, a.min(b), a.max(b));
                if let Some(p) = crossing(body, &over, a, b) {
                    points.insert(key, p);
                }
                if uj { enter = Some(key) } else { leave = Some(key) }
            }
            if let (Some(from), Some(to)) = (leave, enter) {
                next.entry(from).or_default().push(to);
            }
        }
    }
    stitch(&points, next)
}

fn stitch(
    points: &HashMap<EdgeKey, Point>,
    mut next: HashMap<EdgeKey, Vec<EdgeKey>>,
) -> (Vec<Vec<Point>>, usize) {
    let mut loops = Vec::new();
    let mut open = 0;
    let mut starts: Vec<EdgeKey> = next.keys().copied().collect();
    starts.sort_unstable();
    for start in starts {
        while next.get(&start).is_some_and(|v| !v.is_empty()) {
            let mut contour = Vec::new();
            let mut at = start;
            let closed = loop {
                if let Some(p) = points.get(&at) {
                    contour.push(*p);
                }
                let Some(to) = next.get_mut(&at).and_then(Vec::pop) else { break false };
                if to == start {
                    break true;
                }
                at = to;
            };
            let contour = if closed { straightened(contour) } else { contour };
            if closed && contour.len() >= 3 {
                loops.push(contour);
            } else if !closed {
                open += 1;
            }
        }
    }
    (loops, open)
}
