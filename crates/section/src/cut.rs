//! Cutting triangles by a horizontal plane and stitching the cuts into oriented loops.

use layercraft_geom::Point;
use layercraft_mesh::{Body, Mesh};
use std::collections::HashMap;

/// An edge of a body, its vertex indices in increasing order, tagged with the body.
type EdgeKey = (usize, u32, u32);

fn above(z: f64, h: f64) -> bool {
    z >= h
}

/// The point where the plane crosses an edge, always computed from the lower index so that
/// both triangles of the edge get bit-identical points.
fn crossing(body: &Body, a: u32, b: u32, h: f64) -> Option<Point> {
    let (a, b) = (a.min(b), a.max(b));
    let pa = body.vertices.get(a as usize)?;
    let pb = body.vertices.get(b as usize)?;
    let t = (h - pa[2]) / (pb[2] - pa[2]);
    if !t.is_finite() {
        return None;
    }
    Some([pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1])])
}

/// The oriented loops where the plane `z = h` cuts the mesh, and the number of chains that do
/// not close. Loops wind counterclockwise around material, seen from above.
pub fn section_loops(mesh: &Mesh, h: f64) -> (Vec<Vec<Point>>, usize) {
    // For each crossing edge: the point, and the segments leaving it.
    let mut points: HashMap<EdgeKey, Point> = HashMap::new();
    let mut next: HashMap<EdgeKey, Vec<EdgeKey>> = HashMap::new();
    for (bi, body) in mesh.bodies.iter().enumerate() {
        for tri in &body.triangles {
            let Some(corners) = body.corners(tri) else { continue };
            let up = corners.map(|c| above(c[2], h));
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
                if let Some(p) = crossing(body, a, b, h) {
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
            if closed && contour.len() >= 3 {
                loops.push(contour);
            } else if !closed {
                open += 1;
            }
        }
    }
    (loops, open)
}
