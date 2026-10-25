//! Stage 2: what each part of a layer becomes — walls, solid surfaces or sparse infill.
//!
//! With `R_i` the section of layer `i`, the walls are the centre lines of successive insets of
//! `R_i`, one bead width apart, and `A_i` is what lies inside the innermost wall. A point of
//! `A_i` is sparse infill when the solid continues for `top_layers` layers above it and
//! `bottom_layers` below it; otherwise it is solid. The part of `A_i` not covered by layer
//! `i + 1` is the top surface, the one exposed to the air.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use layercraft_geom::{GeomError, Region, difference, intersection, offset, union};
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
}

/// One layer, split by role. Each wall is the region whose boundary is that wall's centre line.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RegionLayer {
    pub index: usize,
    pub top: f64,
    pub height: f64,
    /// Wall centre lines, outermost first.
    pub walls: Vec<Region>,
    /// Inside the walls and exposed to the air above.
    pub top_surface: Region,
    /// Inside the walls, within the top or bottom shells, not exposed.
    pub solid: Region,
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
    if !width(params.outer_wall_width) || !width(params.inner_wall_width) {
        return Err(RegionError::BadParams("wall widths must be 0.05 to 3 mm".into()));
    }
    if params.walls > 50 || params.top_layers > 200 || params.bottom_layers > 200 {
        return Err(RegionError::BadParams("too many walls or shell layers".into()));
    }
    if !params.first_layer_shrink.is_finite() || !(0.0..=1.0).contains(&params.first_layer_shrink) {
        return Err(RegionError::BadParams("first layer shrink must be 0 to 1 mm".into()));
    }
    Ok(())
}

/// The distance from the solid's edge to the centre line of wall `k` (0 = outer).
fn wall_centre(params: &RegionParams, k: usize) -> f64 {
    if k == 0 {
        params.outer_wall_width / 2.0
    } else {
        params.outer_wall_width + (k as f64 - 0.5) * params.inner_wall_width
    }
}

/// The distance from the solid's edge to the inner edge of the innermost wall.
fn walls_depth(params: &RegionParams) -> f64 {
    match params.walls {
        0 => 0.0,
        n => params.outer_wall_width + (n - 1) as f64 * params.inner_wall_width,
    }
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
    let mut layers = Vec::with_capacity(regions.len());
    for (i, (layer, region)) in sections.layers.iter().zip(&regions).enumerate() {
        let mut walls = Vec::new();
        for k in 0..params.walls {
            let centre = offset(region, -wall_centre(params, k))?;
            if centre.is_empty() {
                break;
            }
            walls.push(centre);
        }
        let inside = offset(region, -walls_depth(params))?;
        let i = i as isize;
        let above = covered(&regions, i + 1, params.top_layers)?;
        let below = covered(&regions, i - params.bottom_layers as isize, params.bottom_layers)?;
        let next = covered(&regions, i + 1, 1)?;
        let top_surface = difference(&inside, &next)?;
        let deep = intersection(&intersection(&inside, &above)?, &below)?;
        let shells = difference(&inside, &deep)?;
        let solid = difference(&shells, &top_surface)?;
        layers.push(RegionLayer {
            index: layer.index,
            top: layer.top,
            height: layer.height,
            walls,
            top_surface,
            solid: union(&solid, &Vec::new())?,
            sparse: deep,
        });
    }
    Ok(Regions { layers })
}

#[cfg(test)]
mod tests;
