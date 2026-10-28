//! Solids with known volumes and sections, built in code: the references tests compare against.
//!
//! Every solid is a closed prism: a counterclockwise outline in the plane, extruded from `z0`
//! to `z1`. Its volume is the outline's area times the height, and each section between the
//! two planes is the outline itself.

use crate::{Body, Vertex};

/// A prism over a simple counterclockwise outline, from `z0` up to `z1`.
pub fn prism(name: &str, outline: &[[f64; 2]], z0: f64, z1: f64) -> Body {
    let n = outline.len();
    let mut vertices: Vec<Vertex> = Vec::with_capacity(2 * n + 2);
    vertices.extend(outline.iter().map(|p| [p[0], p[1], z0]));
    vertices.extend(outline.iter().map(|p| [p[0], p[1], z1]));
    let (cx, cy) = centroid(outline);
    vertices.push([cx, cy, z0]);
    vertices.push([cx, cy, z1]);
    let idx = |i: usize| u32::try_from(i).unwrap_or(u32::MAX);
    let (bottom, top) = (idx(2 * n), idx(2 * n + 1));
    let mut triangles = Vec::with_capacity(4 * n);
    for i in 0..n {
        let j = (i + 1) % n;
        let (a, b, c, d) = (idx(i), idx(j), idx(n + i), idx(n + j));
        triangles.push([a, b, d]);
        triangles.push([a, d, c]);
        triangles.push([bottom, b, a]);
        triangles.push([top, c, d]);
    }
    Body { name: name.to_string(), vertices, triangles }
}

fn centroid(outline: &[[f64; 2]]) -> (f64, f64) {
    let n = outline.len().max(1) as f64;
    let (sx, sy) = outline.iter().fold((0.0, 0.0), |(x, y), p| (x + p[0], y + p[1]));
    (sx / n, sy / n)
}

/// An axis-aligned box.
pub fn cuboid(name: &str, min: Vertex, max: Vertex) -> Body {
    let outline = [[min[0], min[1]], [max[0], min[1]], [max[0], max[1]], [min[0], max[1]]];
    prism(name, &outline, min[2], max[2])
}
