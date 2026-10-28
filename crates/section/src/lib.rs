//! Stage 1: the exact section of a mesh by each layer's plane.
//!
//! A plane `z = h` cuts a triangle along a segment whose ends lie on two of its edges. A
//! vertex exactly on the plane counts as above it (symbolic perturbation), so every crossing
//! edge is cut exactly once and the two triangles sharing it meet at the same point: segments
//! stitch into closed loops with no tolerance. Oriented by the outward normals, the loops wind
//! once around the solid's inside; the layer region is the set of positive winding number,
//! which also merges overlapping bodies and absorbs non-manifold edges.

mod cut;

pub use cut::section_loops;

use layercraft_geom::{GeomError, Region, winding_region};
use layercraft_mesh::Mesh;
use serde::{Deserialize, Serialize};

/// The largest number of layers a print may have.
pub const MAX_LAYERS: usize = 100_000;

/// How the solid is cut into layers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Layering {
    /// Height of the first layer, in millimetres.
    pub first_layer_height: f64,
    /// Height of every other layer, in millimetres.
    pub layer_height: f64,
}

/// One layer: the slab from `top - height` to `top`, cut at its middle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Layer {
    pub index: usize,
    /// The nozzle height while printing this layer.
    pub top: f64,
    pub height: f64,
    /// The section of the solid at the layer's mid-height.
    pub region: Region,
}

/// The output of the section stage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sections {
    pub layers: Vec<Layer>,
    /// Section chains that did not close (the mesh has holes); they are left out.
    pub open_chains: usize,
}

/// Why the section stage failed.
#[derive(Debug, thiserror::Error)]
pub enum SectionError {
    #[error("bad layer heights: {0}")]
    BadLayering(String),
    #[error("section geometry: {0}")]
    Geometry(#[from] GeomError),
    #[error("the mesh has no vertex")]
    EmptyMesh,
}

/// The layer slabs `(top, height)` from z = 0 up to `height`.
pub fn layer_slabs(layering: &Layering, height: f64) -> Result<Vec<(f64, f64)>, SectionError> {
    let ok = |h: f64| h.is_finite() && (0.01..=2.0).contains(&h);
    if !ok(layering.first_layer_height) || !ok(layering.layer_height) {
        return Err(SectionError::BadLayering(format!("{layering:?} (each must be 0.01 to 2 mm)")));
    }
    if !height.is_finite() || height <= 0.0 {
        return Err(SectionError::BadLayering(format!("solid height {height} mm")));
    }
    let mut slabs = vec![(layering.first_layer_height, layering.first_layer_height)];
    // A slab is printed when its middle is inside the solid.
    let rest = (height - layering.first_layer_height) / layering.layer_height;
    let count = (rest + 0.5).floor().max(0.0);
    if count > MAX_LAYERS as f64 {
        return Err(SectionError::BadLayering(format!("{count} layers is more than {MAX_LAYERS}")));
    }
    for i in 1..=count as usize {
        slabs.push((layering.first_layer_height + i as f64 * layering.layer_height, layering.layer_height));
    }
    Ok(slabs)
}

/// Cuts the mesh, standing on z = 0, into layers.
pub fn section(mesh: &Mesh, layering: &Layering) -> Result<Sections, SectionError> {
    let [lo, hi] = mesh.bounds().ok_or(SectionError::EmptyMesh)?;
    let height = hi[2] - lo[2].min(0.0);
    let mut layers = Vec::new();
    let mut open_chains = 0;
    for (index, (top, height)) in layer_slabs(layering, height)?.into_iter().enumerate() {
        let (loops, open) = section_loops(mesh, top - height / 2.0);
        open_chains += open;
        layers.push(Layer { index, top, height, region: winding_region(&loops)? });
    }
    Ok(Sections { layers, open_chains })
}

#[cfg(test)]
mod tests;
