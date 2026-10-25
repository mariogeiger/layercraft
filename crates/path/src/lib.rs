//! Stage 3: the lines the nozzle draws, in printing order.
//!
//! Walls become closed loops, starting at the vertex nearest the island's back right corner
//! so seams line up from layer to layer. Solid areas are hatched at the beads' tiling spacing,
//! at ±45° alternating by layer; sparse areas get lines spaced for the requested density by
//! volume, also alternating, so no two beads cross on a layer. Bridges are strands one nozzle
//! wide, side by side. Hatch lines lie on a fixed grid, so layers stack; the move between
//! neighbouring lines is marked as staying inside the part, so it needs no retraction. Each island is printed whole — inner walls, outer
//! wall, then its infill — and islands follow each other nearest first. The first layer's beads
//! are all `first_layer_width` wide.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

mod hatch;
mod link;
mod order;

use layercraft_geom::{GeomError, Point};
use layercraft_region::Regions;
use serde::{Deserialize, Serialize};

/// What a path is for; speeds, accelerations and cooling are chosen by role.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    OuterWall,
    InnerWall,
    TopSurface,
    Solid,
    /// Strands hanging over sparse infill or air, round in section.
    Bridge,
    Sparse,
}

/// One bead: a polyline drawn `width` wide, closed back to its start when `closed`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Path {
    pub role: Role,
    pub width: f64,
    pub closed: bool,
    pub points: Vec<Point>,
    /// The move here from the previous bead stays inside the part: no need to retract or lift.
    #[serde(default)]
    pub travel_inside: bool,
}

/// The beads of one layer, in printing order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PathLayer {
    pub index: usize,
    pub top: f64,
    pub height: f64,
    pub paths: Vec<Path>,
}

/// The output of the path stage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Paths {
    pub layers: Vec<PathLayer>,
}

/// How areas are filled.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PathParams {
    pub outer_wall_width: f64,
    pub inner_wall_width: f64,
    pub top_surface_width: f64,
    pub solid_width: f64,
    pub sparse_width: f64,
    /// Fraction of the area the sparse grid covers, in (0, 1].
    pub sparse_density: f64,
    /// How far infill reaches into the innermost wall, as a fraction of the infill width.
    pub infill_wall_overlap: f64,
    /// Direction of the solid hatch on even layers, in degrees.
    pub infill_angle: f64,
    /// The width of every bead of the first layer.
    pub first_layer_width: f64,
    /// The width of a bridge strand: the nozzle's, as it hangs round from it.
    pub bridge_width: f64,
}

/// Why the path stage failed.
#[derive(Debug)]
pub enum PathError {
    BadParams(String),
    Geometry(GeomError),
}

impl std::fmt::Display for PathError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PathError::BadParams(e) => write!(f, "bad path parameters: {e}"),
            PathError::Geometry(e) => write!(f, "path geometry: {e}"),
        }
    }
}

impl std::error::Error for PathError {}

impl From<GeomError> for PathError {
    fn from(e: GeomError) -> Self {
        PathError::Geometry(e)
    }
}

fn check(p: &PathParams) -> Result<(), PathError> {
    let widths = [
        p.outer_wall_width,
        p.inner_wall_width,
        p.top_surface_width,
        p.solid_width,
        p.sparse_width,
        p.first_layer_width,
        p.bridge_width,
    ];
    if !widths.iter().all(|w| w.is_finite() && (0.05..=3.0).contains(w)) {
        return Err(PathError::BadParams("line widths must be 0.05 to 3 mm".into()));
    }
    if !p.sparse_density.is_finite() || !(0.0..=1.0).contains(&p.sparse_density) {
        return Err(PathError::BadParams("sparse density must be 0 to 1".into()));
    }
    if !p.infill_wall_overlap.is_finite() || !(0.0..=1.0).contains(&p.infill_wall_overlap) {
        return Err(PathError::BadParams("infill overlap must be 0 to 1".into()));
    }
    if !p.infill_angle.is_finite() {
        return Err(PathError::BadParams("infill angle must be finite".into()));
    }
    Ok(())
}

/// Turns every layer's regions into ordered beads.
pub fn trace(regions: &Regions, params: &PathParams) -> Result<Paths, PathError> {
    check(params)?;
    let mut layers = Vec::with_capacity(regions.layers.len());
    let mut at: Point = [0.0, 0.0];
    for layer in &regions.layers {
        let (islands, fills) = order::islands(layer, params)?;
        let paths = order::sequence(islands, &fills, &mut at);
        layers.push(PathLayer { index: layer.index, top: layer.top, height: layer.height, paths });
    }
    Ok(Paths { layers })
}

#[cfg(test)]
mod tests;
