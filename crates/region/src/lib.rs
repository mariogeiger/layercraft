//! Stage 2: what each part of a layer becomes — walls, solid surfaces or sparse infill.
//!
//! With `R_i` the section of layer `i`, the walls are the centre lines of successive insets of
//! `R_i`, one bead spacing apart, and `A_i` is what lies inside the innermost wall. A point of
//! `A_i` is sparse infill when the solid continues for `top_layers` layers above it and
//! `bottom_layers` below it; otherwise it is solid. The part of `A_i` not covered by layer
//! `i + 1` is the top surface, the one exposed to the air. Solid or top that the layer below
//! does not hold up is a bridge, laid as hanging strands: over air, wherever it is wider than a
//! bead, reaching a bead's width onto the support at its ends; over sparse infill, only where it is wide enough to span a gap between the sparse
//! lines (`sparse_gap`) — a narrower band rests on them.
//! A piece of sparse infill smaller than `min_sparse_area` is solid instead. The first layer's
//! beads are wider, to grip the bed. With `one_wall_on_top`, inner walls stay only where the
//! layer above covers them, and the top surface reaches the outer wall.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use layercraft_geom::{
    GeomError, Region, bead_spacing, difference, intersection, offset, signed_area, union,
};
use layercraft_section::Sections;
use serde::{Deserialize, Serialize};

/// The rules that split a layer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RegionParams {
    /// Number of wall loops around every island.
    pub walls: usize,
    pub outer_wall_width: f64,
    pub inner_wall_width: f64,
    /// Solid layers under a top surface.
    pub top_layers: usize,
    /// Solid layers over a bottom surface.
    pub bottom_layers: usize,
    /// How much the first layer shrinks, against the squash of the first layer on the bed.
    pub first_layer_shrink: f64,
    /// The width of every bead of the first layer.
    pub first_layer_width: f64,
    /// A layer that is all top surface gets only its outer wall.
    pub one_wall_on_top: bool,
    /// The distance between the sparse infill's lines: the gap a strand over it must span.
    pub sparse_gap: f64,
    /// Pieces of sparse infill smaller than this are solid, in mm².
    #[serde(default)]
    pub min_sparse_area: f64,
}

/// One layer, split by role. Each wall is the region whose boundary is that wall's centre line.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RegionLayer {
    pub index: usize,
    pub top: f64,
    pub height: f64,
    /// Wall centre lines, outermost first.
    pub walls: Vec<Region>,
    /// Where inner walls are drawn, when not everywhere: under the layer above.
    #[serde(default)]
    pub inner_walls_within: Option<Region>,
    /// Inside the walls and exposed to the air above.
    pub top_surface: Region,
    /// Inside the walls, within the top or bottom shells, not exposed.
    pub solid: Region,
    /// Solid or top surface over air: strands hang from the edges of the gap.
    pub bridge: Region,
    /// Solid or top surface over sparse infill: strands hang from one sparse line to the next.
    #[serde(default)]
    pub bridge_on_sparse: Region,
    /// Inside the walls, deep in the solid.
    pub sparse: Region,
}

/// The output of the region stage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Regions {
    pub layers: Vec<RegionLayer>,
}

/// Why the region stage failed.
#[derive(Debug)]
pub enum RegionError {
    BadParams(String),
    Geometry(GeomError),
}

impl std::fmt::Display for RegionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RegionError::BadParams(e) => write!(f, "bad region parameters: {e}"),
            RegionError::Geometry(e) => write!(f, "region geometry: {e}"),
        }
    }
}

impl std::error::Error for RegionError {}

impl From<GeomError> for RegionError {
    fn from(e: GeomError) -> Self {
        RegionError::Geometry(e)
    }
}

fn check(params: &RegionParams) -> Result<(), RegionError> {
    let width = |w: f64| w.is_finite() && (0.05..=3.0).contains(&w);
    if !width(params.outer_wall_width) || !width(params.inner_wall_width) || !width(params.first_layer_width)
    {
        return Err(RegionError::BadParams("wall widths must be 0.05 to 3 mm".into()));
    }
    if !params.sparse_gap.is_finite() || params.sparse_gap < 0.0 {
        return Err(RegionError::BadParams("sparse gap must be finite and not negative".into()));
    }
    if params.walls > 50 || params.top_layers > 200 || params.bottom_layers > 200 {
        return Err(RegionError::BadParams("too many walls or shell layers".into()));
    }
    if !params.first_layer_shrink.is_finite() || !(0.0..=1.0).contains(&params.first_layer_shrink) {
        return Err(RegionError::BadParams("first layer shrink must be 0 to 1 mm".into()));
    }
    Ok(())
}

/// The widths of a layer's outer and inner wall beads.
#[derive(Clone, Copy)]
struct Widths {
    outer: f64,
    inner: f64,
}

/// The distance from the solid's edge to the centre line of wall `k` (0 = outer), on a layer
/// `h` high. The outer bead's side touches the edge; inward, beads sit their tiling spacing
/// apart (`bead_spacing`), so the walls fill their band without voids.
fn wall_centre(w: Widths, k: usize, h: f64) -> f64 {
    let (outer, inner) = (bead_spacing(w.outer, h), bead_spacing(w.inner, h));
    match k {
        0 => w.outer / 2.0,
        k => w.outer / 2.0 + outer / 2.0 + (k as f64 - 0.5) * inner,
    }
}

/// The distance from the solid's edge to where `n` walls' material ends, on a layer `h` high.
fn walls_depth(w: Widths, n: usize, h: f64) -> f64 {
    let (outer, inner) = (bead_spacing(w.outer, h), bead_spacing(w.inner, h));
    match n {
        0 => 0.0,
        n => w.outer / 2.0 + outer / 2.0 + (n - 1) as f64 * inner,
    }
}

/// The region without its parts narrower than `2r`: shrunk by `r`, then grown back.
fn opening(region: &Region, r: f64) -> Result<Region, GeomError> {
    if r <= 0.0 || region.is_empty() {
        return Ok(region.clone());
    }
    let core = offset(region, -r)?;
    intersection(&offset(&core, r)?, region)
}

/// The intersection of the sections `from..from + count`, empty when one is missing.
fn covered(sections: &[Region], from: isize, count: usize) -> Result<Region, GeomError> {
    let mut acc: Option<Region> = None;
    for k in 0..count {
        let i = from + k as isize;
        let Some(r) = usize::try_from(i).ok().and_then(|i| sections.get(i)) else { return Ok(Vec::new()) };
        acc = Some(match acc {
            None => r.clone(),
            Some(a) => intersection(&a, r)?,
        });
    }
    Ok(acc.unwrap_or_default())
}

/// Splits every layer of the sections by role.
pub fn split(sections: &Sections, params: &RegionParams) -> Result<Regions, RegionError> {
    check(params)?;
    let mut regions: Vec<Region> = Vec::with_capacity(sections.layers.len());
    for layer in &sections.layers {
        let shrink = if layer.index == 0 { params.first_layer_shrink } else { 0.0 };
        regions.push(offset(&layer.region, -shrink)?);
    }
    let mut layers: Vec<RegionLayer> = Vec::with_capacity(regions.len());
    for (i, (layer, region)) in sections.layers.iter().zip(&regions).enumerate() {
        let h = layer.height;
        let widths = if i == 0 {
            Widths { outer: params.first_layer_width, inner: params.first_layer_width }
        } else {
            Widths { outer: params.outer_wall_width, inner: params.inner_wall_width }
        };
        let i = i as isize;
        let next = covered(&regions, i + 1, 1)?;
        let mut walls = Vec::new();
        for k in 0..params.walls {
            let centre = offset(region, -wall_centre(widths, k, h))?;
            if centre.is_empty() {
                break;
            }
            walls.push(centre);
        }
        let inside = offset(region, -walls_depth(widths, params.walls, h))?;
        // Inner walls under the air would only show through the top: there, the top surface
        // runs out to the outer wall and the inner walls stop.
        let (top_reach, inner_walls_within) = if params.one_wall_on_top && params.walls > 1 {
            (offset(region, -walls_depth(widths, 1, h))?, Some(next.clone()))
        } else {
            (inside.clone(), None)
        };
        let above = covered(&regions, i + 1, params.top_layers)?;
        let below = covered(&regions, i - params.bottom_layers as isize, params.bottom_layers)?;
        let top_surface = difference(&top_reach, &next)?;
        // A small piece of sparse infill would be a line or two holding nothing up: solid.
        let deep: Region = intersection(&intersection(&inside, &above)?, &below)?
            .into_iter()
            .filter(|shape| shape.iter().map(|c| signed_area(c)).sum::<f64>() >= params.min_sparse_area)
            .collect();
        let shells = difference(&inside, &deep)?;
        let solid = difference(&shells, &top_surface)?;
        // What is not held up from below: over air (the layer below is absent) and over
        // its sparse infill. Only the parts wide enough to need hanging strands are bridges.
        let filled = union(&solid, &top_surface)?;
        let (bridge, bridge_on_sparse) =
            match (usize::try_from(i - 1).ok().and_then(|j| regions.get(j)), layers.last()) {
                (Some(below), Some(previous)) => {
                    let width = widths.inner.max(widths.outer);
                    // Each strand rests a bead's width on the support at either end.
                    let gap = opening(&difference(&filled, below)?, width / 2.0)?;
                    let over_air = intersection(&offset(&gap, width)?, &union(&filled, &deep)?)?;
                    let over_sparse =
                        opening(&intersection(&filled, &previous.sparse)?, params.sparse_gap / 2.0)?;
                    (over_air.clone(), difference(&over_sparse, &over_air)?)
                }
                _ => (Vec::new(), Vec::new()),
            };
        let hanging = union(&bridge, &bridge_on_sparse)?;
        let solid = difference(&solid, &hanging)?;
        let top_surface = difference(&top_surface, &hanging)?;
        let deep = difference(&deep, &hanging)?;
        layers.push(RegionLayer {
            index: layer.index,
            top: layer.top,
            height: h,
            walls,
            inner_walls_within,
            top_surface,
            solid,
            bridge,
            bridge_on_sparse,
            sparse: deep,
        });
    }
    Ok(Regions { layers })
}

#[cfg(test)]
mod tests;
