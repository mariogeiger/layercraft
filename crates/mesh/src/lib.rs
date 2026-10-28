//! Triangle meshes in millimetres, read from 3MF (the input of the compiler).
//!
//! A `Mesh` is a list of bodies placed in the printer's frame; each body is an indexed triangle
//! set whose triangles face outward (counterclockwise seen from outside). 3MF files are hostile
//! input: indices are checked, sizes capped and every coordinate must be finite.

pub mod solids;
mod threemf;

pub use threemf::read_3mf;

use serde::{Deserialize, Serialize};

/// A point in space, in millimetres.
pub type Vertex = [f64; 3];

/// One solid: an indexed set of outward-facing triangles.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Body {
    pub name: String,
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<[u32; 3]>,
}

/// The bodies to print, in the printer's frame.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Mesh {
    pub bodies: Vec<Body>,
}

/// Why a mesh could not be read.
#[derive(Debug, thiserror::Error)]
pub enum MeshError {
    #[error("cannot read the mesh file: {0}")]
    Io(#[from] std::io::Error),
    #[error("not a valid 3MF archive: {0}")]
    Zip(String),
    #[error("the 3MF model is not valid XML: {0}")]
    Xml(String),
    /// The file is readable but its content breaks the 3MF rules.
    #[error("invalid 3MF model: {0}")]
    Invalid(String),
}

impl Body {
    /// The triangle's three corners, or `None` when an index is out of range.
    pub fn corners(&self, t: &[u32; 3]) -> Option<[Vertex; 3]> {
        let v = |i: u32| self.vertices.get(i as usize).copied();
        Some([v(t[0])?, v(t[1])?, v(t[2])?])
    }

    /// The enclosed volume (divergence theorem); negative when the triangles face inward.
    pub fn volume(&self) -> f64 {
        self.triangles
            .iter()
            .filter_map(|t| self.corners(t))
            .map(|[a, b, c]| {
                let cross = [b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]];
                (a[0] * cross[0] + a[1] * cross[1] + a[2] * cross[2]) / 6.0
            })
            .sum()
    }
}

impl Mesh {
    /// The axis-aligned bounds `[min, max]` of all bodies, or `None` when there is no vertex.
    pub fn bounds(&self) -> Option<[Vertex; 2]> {
        let mut all = self.bodies.iter().flat_map(|b| b.vertices.iter());
        let first = *all.next()?;
        Some(all.fold([first, first], |[lo, hi], v| {
            [
                [lo[0].min(v[0]), lo[1].min(v[1]), lo[2].min(v[2])],
                [hi[0].max(v[0]), hi[1].max(v[1]), hi[2].max(v[2])],
            ]
        }))
    }

    /// Moves every body by `delta`.
    pub fn translate(&mut self, delta: Vertex) {
        for v in self.bodies.iter_mut().flat_map(|b| b.vertices.iter_mut()) {
            *v = [v[0] + delta[0], v[1] + delta[1], v[2] + delta[2]];
        }
    }

    /// The total enclosed volume.
    pub fn volume(&self) -> f64 {
        self.bodies.iter().map(Body::volume).sum()
    }
}

#[cfg(test)]
mod tests;
