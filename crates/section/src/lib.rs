//! Stage 1: the exact section of a mesh by each layer's middle surface.
//!
//! A layer is the slab between two leaves (`foliate`): flat planes, unless the layering bends
//! them to follow the solid's gently sloped top surfaces. Each layer is cut at its middle, the
//! surface halfway between its bottom and top. A plane `z = h` cuts a triangle along a segment
//! whose ends lie on two of its edges; a bent leaf does too once the mesh is cut along the grid
//! its leaves bend on (`refine`). A vertex exactly on the surface counts as above it (symbolic
//! perturbation), so every crossing edge is cut exactly once and the two triangles sharing it
//! meet at the same point: segments stitch into closed loops with no tolerance. Oriented by the
//! outward normals, the loops wind once around the solid's inside; the layer region is the set
//! of positive winding number, which also merges overlapping bodies and absorbs non-manifold
//! edges.

mod cut;
mod foliate;
mod patches;
mod refine;

pub use cut::section_loops;
pub use foliate::Foliation;
pub use refine::refine;

use layercraft_geom::{GeomError, Region, Relief, winding_region};
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
    /// How the layers bend to follow gently sloped top surfaces; flat without.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub foliation: Option<Foliation>,
}

/// One layer: the slab from `top - height` to `top`, both bent by the leaves, cut at its middle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Layer {
    pub index: usize,
    /// The nozzle height while printing this layer, where it is flat.
    pub top: f64,
    pub height: f64,
    /// How far the layer's top lies above `top` over the plate: zero where it is flat. The
    /// layer's bottom is the top of the layer below, or the bed.
    #[serde(default, skip_serializing_if = "Relief::is_zero")]
    pub warp: Relief,
    /// The section of the solid at the layer's middle.
    pub region: Region,
}

/// The output of the section stage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sections {
    pub layers: Vec<Layer>,
    /// Section chains that did not close (the mesh has holes); they are left out.
    pub open_chains: usize,
    /// Cells of the mesh its grid could not split (`refine`).
    #[serde(default)]
    pub unsplit_cells: usize,
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
    let slabs = layer_slabs(layering, hi[2] - lo[2].min(0.0))?;
    let tops: Vec<f64> = slabs.iter().map(|s| s.0).collect();
    let warps = match &layering.foliation {
        Some(f) => foliate::foliate(mesh, &tops, layering.layer_height, f)?,
        None => Vec::new(),
    };
    // Each layer's middle: halfway between the layer below's top, or the bed, and its own.
    let mut middles: Vec<Relief> = Vec::with_capacity(slabs.len());
    for (k, warp) in warps.iter().enumerate() {
        middles.push(match k.checked_sub(1).and_then(|b| warps.get(b)) {
            Some(below) => Relief::combine(below, warp, |a, b| (a + b) / 2.0)?,
            None => Relief::combine(warp, warp, |a, _| a / 2.0)?,
        });
    }
    // The mesh cut once along the lines every bent middle bends on, over the heights they reach.
    let mut lines = None;
    let mut reach = [f64::INFINITY, f64::NEG_INFINITY];
    for ((top, height), middle) in slabs.iter().zip(&middles) {
        let Some(l) = middle.lines() else { continue };
        let [dlo, dhi] = middle.range();
        reach = [reach[0].min(top - height / 2.0 + dlo), reach[1].max(top - height / 2.0 + dhi)];
        lines = Some(lines.map_or(l, |m: layercraft_geom::Lines| m.union(l)));
    }
    let (refined, unsplit_cells) = match (lines, middles.first()) {
        (Some(lines), Some(m)) => {
            let (refined, failed) = refine(mesh, &m.grid, &lines, reach);
            (Some(refined), failed)
        }
        _ => (None, 0),
    };
    let mut layers = Vec::new();
    let mut open_chains = 0;
    for (index, (top, height)) in slabs.into_iter().enumerate() {
        let h = top - height / 2.0;
        let (loops, open) = match (middles.get(index), &refined) {
            (Some(m), Some(refined)) if !m.is_zero() => section_loops(refined, |v| h + m.at([v[0], v[1]])),
            _ => section_loops(mesh, |_| h),
        };
        open_chains += open;
        let warp = warps.get(index).cloned().unwrap_or_default();
        layers.push(Layer { index, top, height, warp, region: winding_region(&loops)? });
    }
    Ok(Sections { layers, open_chains, unsplit_cells })
}

#[cfg(test)]
mod tests;
