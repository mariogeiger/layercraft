//! Stage 3: the lines the nozzle draws, in printing order.
//!
//! Walls become closed loops, starting at the vertex nearest the island's back right corner
//! so seams line up from layer to layer. Solid areas are hatched at the beads' tiling spacing,
//! at ±45° alternating by layer; sparse areas get lines spaced for the requested density by
//! volume, also alternating, so no two beads cross on a layer. Bridges are strands one nozzle
//! wide: side by side over air, where they hang; over sparse infill, where they rest, spaced to
//! fill their layer exactly. Hatch lines lie on a fixed grid, so layers stack; the move between
//! neighbouring lines is marked as staying inside the part, so it needs no retraction. Each
//! island is printed whole — inner walls, outer
//! wall, then its infill — and islands follow each other nearest first. The lines of a
//! `monotone` role are laid in sweeps, each against the one before. The first layer's beads
//! are all `first_layer_width` wide. Each wall segment carries the share of its bead that hangs
//! past the layer below (`overhang`). Each island is traced by the parameters at its centre.
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
mod overhang;
mod role;

pub use role::{Face, Kind, Role, Support};

use layercraft_geom::{GeomError, Point};
use layercraft_region::Regions;
use layercraft_tune::{Field, TuneError};
use serde::{Deserialize, Serialize};

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
    /// How far each segment hangs over air, as a fraction of the bead's width from 0 to 1,
    /// segment `i` running from point `i` to the next; empty where nothing hangs.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overhang: Vec<f64>,
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
    /// The faces whose dense lines are laid in sweeps across their area, each against the one
    /// before and all the same way, so the surface looks the same everywhere; the other lines
    /// go nearest first.
    #[serde(default)]
    pub monotone: Vec<Face>,
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

impl From<TuneError> for PathError {
    fn from(e: TuneError) -> Self {
        PathError::BadParams(e.to_string())
    }
}

impl From<GeomError> for PathError {
    fn from(e: GeomError) -> Self {
        PathError::Geometry(e)
    }
}

pub(crate) fn check(p: &PathParams) -> Result<(), PathError> {
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

/// Turns every layer's regions into ordered beads, each island by the parameters at its centre.
pub fn trace(regions: &Regions, field: &Field<PathParams>) -> Result<Paths, PathError> {
    check(field.base())?;
    let mut layers = Vec::with_capacity(regions.layers.len());
    let mut at: Point = [0.0, 0.0];
    for layer in &regions.layers {
        let mut islands = Vec::new();
        for (params, part) in order::parts(layer, field)? {
            islands.extend(order::islands(&part, &params)?);
        }
        let paths = order::sequence(islands, &mut at);
        layers.push(PathLayer { index: layer.index, top: layer.top, height: layer.height, paths });
    }
    Ok(Paths { layers })
}

#[cfg(test)]
mod tests;
