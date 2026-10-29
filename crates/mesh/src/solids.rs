//! Solids with known volumes and sections, built in code: the references tests compare against.
//!
//! A prism is a counterclockwise outline in the plane, extruded from `z0` to `z1`: its volume is
//! the outline's area times the height, and each section between the two planes is the outline
//! itself. A height field stands on z = 0 over a rectangle up to a surface sampled on a grid of
//! squares, each split by a diagonal: its volume is the integral of that piecewise linear surface.

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

/// A solid over the rectangle from `lo` to `hi`, from z = 0 up to `top(x, y)` sampled at the
/// corners of `n × n` squares, each split by its diagonal from its lower left corner.
pub fn height_field(name: &str, lo: [f64; 2], hi: [f64; 2], n: usize, top: impl Fn(f64, f64) -> f64) -> Body {
    let n = n.max(1);
    let side = n + 1;
    let at = |i: usize, j: usize| {
        [lo[0] + (hi[0] - lo[0]) * i as f64 / n as f64, lo[1] + (hi[1] - lo[1]) * j as f64 / n as f64]
    };
    let mut vertices: Vec<Vertex> = Vec::with_capacity(2 * side * side);
    for z in [true, false] {
        for j in 0..side {
            for i in 0..side {
                let [x, y] = at(i, j);
                vertices.push([x, y, if z { top(x, y) } else { 0.0 }]);
            }
        }
    }
    let up = |i: usize, j: usize| (j * side + i) as u32;
    let down = |i: usize, j: usize| (side * side + j * side + i) as u32;
    let mut triangles = Vec::new();
    for j in 0..n {
        for i in 0..n {
            triangles.push([up(i, j), up(i + 1, j), up(i + 1, j + 1)]);
            triangles.push([up(i, j), up(i + 1, j + 1), up(i, j + 1)]);
            triangles.push([down(i, j), down(i + 1, j + 1), down(i + 1, j)]);
            triangles.push([down(i, j), down(i, j + 1), down(i + 1, j + 1)]);
        }
    }
    // The boundary, counterclockwise seen from above, each step walled from the floor up.
    let mut ring: Vec<(usize, usize)> = (0..n).map(|i| (i, 0)).collect();
    ring.extend((0..n).map(|j| (n, j)));
    ring.extend((1..=n).rev().map(|i| (i, n)));
    ring.extend((1..=n).rev().map(|j| (0, j)));
    for (k, &(i, j)) in ring.iter().enumerate() {
        let (ni, nj) = ring.get((k + 1) % ring.len()).copied().unwrap_or((i, j));
        triangles.push([down(i, j), down(ni, nj), up(ni, nj)]);
        triangles.push([down(i, j), up(ni, nj), up(i, j)]);
    }
    Body { name: name.to_string(), vertices, triangles }
}
