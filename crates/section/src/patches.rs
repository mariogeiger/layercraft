//! The top surfaces a leaf may follow, and their flat faces.
//!
//! A patch is a connected set of the mesh's up-facing triangles no steeper than the leaves may
//! be, joined across shared edges: one surface `S` over its shadow on the plate. Its triangles
//! that share an edge and a plane make one facet, a flat polygon whose height is that plane;
//! where two facets meet, `S` has a crease. A leaf that follows the patch is linear on each
//! facet and bends along the creases, so that it meets `S` exactly once the triangles the leaves
//! bend on have every facet's edges among theirs (`triangulation`).

use layercraft_geom::{GRID_MM, GeomError, Point, Region, winding_region};
use layercraft_mesh::{Mesh, Vertex};
use std::collections::HashMap;

/// How far off a triangle's plane its neighbour's far corner may lie and the two still share
/// it, in mm: a thousandth of the lattice step, far above rounding, far below any crease.
const COPLANAR_MM: f64 = GRID_MM / 1000.0;

/// A flat face of a patch.
#[derive(Debug, Clone, PartialEq)]
pub struct Facet {
    /// Its shadow on the plate.
    pub region: Region,
    /// A corner of it, and how its height grows across the plate, in mm per mm.
    pub anchor: Vertex,
    pub gradient: [f64; 2],
}

impl Facet {
    /// The height of the facet's plane over `p`.
    pub fn height(&self, p: Point) -> f64 {
        self.anchor[2]
            + self.gradient[0] * (p[0] - self.anchor[0])
            + self.gradient[1] * (p[1] - self.anchor[1])
    }

    /// How steeply it leans: the tangent of its angle to the plate.
    pub fn slope(&self) -> f64 {
        self.gradient[0].hypot(self.gradient[1])
    }
}

/// One top surface a leaf may follow.
#[derive(Debug, Clone, PartialEq)]
pub struct Patch {
    /// The area of its shadow on the plate, in mm².
    pub area: f64,
    pub facets: Vec<Facet>,
}

/// The outward normal of a triangle, not normalised.
fn normal([a, b, c]: [Vertex; 3]) -> Vertex {
    let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
    [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
}

/// How the height of the triangle's plane grows across the plate, when it is not vertical.
fn gradient(t: [Vertex; 3]) -> Option<[f64; 2]> {
    let n = normal(t);
    (n[2] > 0.0 && n[2].is_finite()).then(|| [-n[0] / n[2], -n[1] / n[2]])
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

fn join(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if let Some(slot) = parent.get_mut(ra) {
        *slot = rb;
    }
}

/// The groups `parent` makes of `0..n`, numbered by their first member, members in order.
fn classes(parent: &mut [usize]) -> Vec<Vec<usize>> {
    let mut number: HashMap<usize, usize> = HashMap::new();
    let mut out: Vec<Vec<usize>> = Vec::new();
    for id in 0..parent.len() {
        let root = find(parent, id);
        let k = *number.entry(root).or_insert(out.len());
        if k == out.len() {
            out.push(Vec::new());
        }
        if let Some(group) = out.get_mut(k) {
            group.push(id);
        }
    }
    out
}

/// The up-facing triangles no steeper than `max_slope` (tangent), grouped into patches, each
/// split into its facets, in the mesh's order.
fn groups(mesh: &Mesh, max_slope: f64) -> Vec<Vec<Vec<[Vertex; 3]>>> {
    let mut triangles: Vec<[Vertex; 3]> = Vec::new();
    // Each triangle's neighbours across its edges, with the neighbour's far corner.
    let mut shared: Vec<(usize, usize, Vertex)> = Vec::new();
    for body in &mesh.bodies {
        let mut first: HashMap<(u32, u32), (usize, u32)> = HashMap::new();
        for tri in &body.triangles {
            let Some(corners) = body.corners(tri) else { continue };
            if gradient(corners).is_none_or(|g| g[0].hypot(g[1]) > max_slope) {
                continue;
            }
            let id = triangles.len();
            triangles.push(corners);
            for (a, b, far) in [(tri[0], tri[1], tri[2]), (tri[1], tri[2], tri[0]), (tri[2], tri[0], tri[1])]
            {
                match first.get(&(a.min(b), a.max(b))) {
                    Some(&(other, other_far)) => {
                        if let Some(&q) = body.vertices.get(other_far as usize) {
                            shared.push((id, other, q));
                        }
                    }
                    None => {
                        first.insert((a.min(b), a.max(b)), (id, far));
                    }
                }
            }
        }
    }
    let mut patch = (0..triangles.len()).collect::<Vec<_>>();
    let mut facet = patch.clone();
    for &(a, b, far) in &shared {
        join(&mut patch, a, b);
        // The same facet when the neighbour's far corner lies on this triangle's plane.
        let Some(t) = triangles.get(a).copied() else { continue };
        let Some(g) = gradient(t) else { continue };
        let on = t[0][2] + g[0] * (far[0] - t[0][0]) + g[1] * (far[1] - t[0][1]);
        if (far[2] - on).abs() <= COPLANAR_MM {
            join(&mut facet, a, b);
        }
    }
    let facets = classes(&mut facet);
    let mut by_patch: HashMap<usize, Vec<Vec<[Vertex; 3]>>> = HashMap::new();
    let mut order: Vec<usize> = Vec::new();
    for members in facets {
        let Some(&head) = members.first() else { continue };
        let root = find(&mut patch, head);
        if !by_patch.contains_key(&root) {
            order.push(root);
        }
        by_patch
            .entry(root)
            .or_default()
            .push(members.iter().filter_map(|&m| triangles.get(m).copied()).collect());
    }
    order.into_iter().filter_map(|root| by_patch.remove(&root)).collect()
}

/// The patches of the mesh no steeper than `max_slope` (tangent), largest first.
pub fn patches(mesh: &Mesh, max_slope: f64) -> Result<Vec<Patch>, GeomError> {
    let mut out = Vec::new();
    for facets in groups(mesh, max_slope) {
        let mut patch = Patch { area: 0.0, facets: Vec::with_capacity(facets.len()) };
        for triangles in facets {
            let Some(&first) = triangles.first() else { continue };
            let Some(g) = gradient(first) else { continue };
            let shadows: Vec<Vec<Point>> =
                triangles.iter().map(|t| t.iter().map(|v| [v[0], v[1]]).collect()).collect();
            let region = winding_region(&shadows)?;
            patch.area += layercraft_geom::area(&region);
            if !region.is_empty() {
                patch.facets.push(Facet { region, anchor: first[0], gradient: g });
            }
        }
        if !patch.facets.is_empty() {
            out.push(patch);
        }
    }
    out.sort_by(|a, b| b.area.total_cmp(&a.area));
    Ok(out)
}
