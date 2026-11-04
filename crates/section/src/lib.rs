//! Stage 1: the exact section of a mesh by each layer's middle surface.
//!
//! A layer is the slab between two leaves (`foliate`): flat planes, unless the layering bends
//! them to follow the solid's gently sloped top surfaces. Each layer is cut at its middle, the
//! surface halfway between its bottom and top. A plane `z = h` cuts a triangle along a segment
//! whose ends lie on two of its edges; a bent leaf does too once the mesh is cut along the
//! triangles its leaves are linear on (`refine`). A vertex exactly on the surface counts as
//! above it (symbolic perturbation), so every crossing edge is cut exactly once and the two
//! triangles sharing it meet at the same point: segments stitch into closed loops with no
//! tolerance. Oriented by the outward normals, the loops wind once around the solid's inside;
//! the layer region is the set of positive winding number, which also merges overlapping bodies
//! and absorbs non-manifold edges. With several materials, each one's bodies are cut alone as
//! well; where they overlap, the material listed later is printed, so the materials' sections
//! partition the layer's.

mod cut;
mod envelope;
mod face;
mod foliate;
mod patches;
mod refine;
mod trace;
mod triangulation;

pub use cut::{Cut, section_loops};
pub use foliate::Foliation;

use crate::cut::section_over;
use crate::foliate::Leaves;
use crate::refine::refine;
use crate::trace::Net;
use layercraft_geom::{Carrier, GeomError, Region, Relief, difference, union, winding_region};
use layercraft_mesh::{Body, Mesh};
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
    /// Each material's part of `region`, by material; empty when there is one material.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub materials: Vec<Region>,
}

impl Layer {
    /// Each material's part of the layer, by material: the whole layer when there is one.
    pub fn by_material(&self) -> Vec<&Region> {
        if self.materials.is_empty() { vec![&self.region] } else { self.materials.iter().collect() }
    }
}

/// The output of the section stage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sections {
    pub layers: Vec<Layer>,
    /// Section chains that stopped at a crack in the mesh; each was joined to the chain whose start
    /// lay nearest its end.
    pub open_chains: usize,
    /// The widest crack a section was closed across, in mm.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub widest_crack: f64,
    /// Triangles of the mesh the leaves' triangles could not cut, fanned whole instead
    /// (`refine`).
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
    let leaves = match &layering.foliation {
        Some(f) => foliate::foliate(mesh, &tops, layering.layer_height, f)?,
        None => Leaves::default(),
    };
    let t = &leaves.triangulation;
    let inner = t.triangles.get(..t.inner).unwrap_or(&[]);
    // Each layer's top, a relief on the bent triangles, which every bent leaf shares and comes
    // back to flat along the edge of.
    let carrier = Carrier::new(t.points.clone(), inner.to_vec())?;
    let warps = leaves
        .values
        .into_iter()
        .map(|leaf| leaf.map_or(Ok(Relief::zero()), |values| carrier.relief(values)))
        .collect::<Result<Vec<Relief>, GeomError>>()?;
    // Each layer's middle at their corners, where bent: halfway between the top of the layer
    // below, or the bed, and its own.
    let middle = |k: usize| -> Option<Vec<f64>> {
        let (below, own) = (k.checked_sub(1).and_then(|j| warps.get(j)), warps.get(k));
        if below.is_none_or(Relief::is_zero) && own.is_none_or(Relief::is_zero) {
            return None;
        }
        let value = |r: Option<&Relief>, i: usize| r.and_then(|r| r.values().get(i)).copied().unwrap_or(0.0);
        let m: Vec<f64> = (0..t.points.len()).map(|i| (value(below, i) + value(own, i)) / 2.0).collect();
        m.iter().any(|&v| v != 0.0).then_some(m)
    };
    // The mesh cut once along the bent triangles, over the heights the bent middles reach.
    let mut heights = [f64::INFINITY, f64::NEG_INFINITY];
    for (k, &(top, height)) in slabs.iter().enumerate() {
        let Some(m) = middle(k) else { continue };
        let [dlo, dhi] = m.iter().fold([0.0f64; 2], |[lo, hi], &v| [lo.min(v), hi.max(v)]);
        let h = top - height / 2.0;
        heights = [heights[0].min(h + dlo), heights[1].max(h + dhi)];
    }
    let refined = if heights[0] <= heights[1] {
        let net = Net::new(&t.points, &t.triangles, t.inner)?;
        let mut area = [[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]];
        for p in inner.iter().flatten().filter_map(|&i| t.points.get(i as usize)) {
            area =
                [[area[0][0].min(p[0]), area[0][1].min(p[1])], [area[1][0].max(p[0]), area[1][1].max(p[1])]];
        }
        Some(refine(mesh, &net, area, heights))
    } else {
        None
    };
    let materials = mesh.materials.len();
    let mut layers = Vec::with_capacity(slabs.len());
    let (mut open_chains, mut widest_crack) = (0, 0.0_f64);
    for (index, (top, height)) in slabs.into_iter().enumerate() {
        let h = top - height / 2.0;
        // Each body's vertices' heights over the layer's middle.
        let (bodies, over): (&[Body], Vec<Vec<f64>>) = match (middle(index), &refined) {
            (Some(m), Some(r)) => (
                &r.mesh.bodies,
                r.mesh
                    .bodies
                    .iter()
                    .zip(&r.places)
                    .map(|(b, p)| {
                        b.vertices.iter().zip(p).map(|(v, place)| v[2] - h - place.value(&m)).collect()
                    })
                    .collect(),
            ),
            _ => (
                &mesh.bodies,
                mesh.bodies.iter().map(|b| b.vertices.iter().map(|v| v[2] - h).collect()).collect(),
            ),
        };
        let cut = |material: Option<usize>| {
            let chosen = bodies.iter().zip(&over).filter(|(b, _)| material.is_none_or(|k| b.material == k));
            section_over(chosen.map(|(b, o)| (b, o.as_slice())))
        };
        let whole = cut(None);
        open_chains += whole.open;
        widest_crack = widest_crack.max(whole.widest_gap);
        // Later materials win where bodies overlap: each keeps what no later one covers.
        let mut parts: Vec<Region> = Vec::new();
        if materials > 1 {
            let mut later: Region = Vec::new();
            for k in (0..materials).rev() {
                let own = winding_region(&cut(Some(k)).loops)?;
                parts.push(difference(&own, &later)?);
                later = union(&later, &own)?;
            }
            parts.reverse();
        }
        let warp = warps.get(index).cloned().unwrap_or_default();
        layers.push(Layer {
            index,
            top,
            height,
            warp,
            region: winding_region(&whole.loops)?,
            materials: parts,
        });
    }
    let unsplit_cells = refined.map_or(0, |r| r.unsplit);
    Ok(Sections { layers, open_chains, widest_crack, unsplit_cells })
}

#[cfg(test)]
mod tests;

fn is_zero(x: &f64) -> bool {
    *x == 0.0
}
