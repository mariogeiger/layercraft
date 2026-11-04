//! Stage 3: the lines the nozzle draws, in printing order.
//!
//! Walls become closed loops, starting at the concave corner nearest the seam of the layer
//! below (`seam`), so seams line up from layer to layer. Dense areas are laid at the beads' tiling spacing as
//! families of parallel curves with the fewest bead ends (`families`): a band narrower than
//! its beads' crossing width along itself, in insets of its edge, a wider area in straight lines
//! along or across its edges, crossing the dense beads of the layer below. Sparse areas get
//! lines spaced for the requested density by volume, turning a quarter turn each layer, so no
//! two beads cross on a layer, or convection rolls as far apart (`rolls`), running from wall to
//! wall and resting on those below. Bridges are strands one nozzle wide: side by side over air,
//! where they hang; over sparse infill, where they rest, spaced to fill their layer exactly.
//! Sparse lines lie on a fixed grid, so layers stack; the move between neighbouring lines is
//! marked as staying inside the part, so it needs no retraction, as is any move over what the
//! layer above covers. Each island is printed whole —
//! inner walls, outer wall, then its infill — and islands follow each other nearest first.
//! Within an island, the beads of each kind follow the tour that retracts least often, then
//! travels least; the lines of a `monotone` role are laid in sweeps instead, each against the
//! one before. The first layer's beads are all `first_layer_width` wide. Each wall segment
//! carries the share of its bead that hangs past the layer below (`overhang`). Each island is
//! traced by the parameters at its centre. The supports are islands of their own (`supports`).

mod families;
mod frame;
mod gap;
mod link;
mod order;
mod overhang;
mod role;
mod rolls;
mod seam;
mod solids;
mod supports;
mod tour;

pub use role::{Face, Kind, Role, Support};

use layercraft_geom::{Bands, GeomError, Point, Relief};
use layercraft_region::{RegionLayer, Regions};
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

impl Path {
    /// A bead along `points`, reached by an ordinary move and hanging nowhere.
    pub fn new(role: Role, width: f64, closed: bool, points: Vec<Point>) -> Path {
        Path { role, width, closed, points, travel_inside: false, overhang: Vec::new() }
    }

    /// Where the nozzle leaves the bead: back at its start when it is closed.
    pub fn end(&self) -> Option<Point> {
        if self.closed { self.points.first() } else { self.points.last() }.copied()
    }

    /// An open bead laid the other way, from its end, each segment's overhang with it; a loop
    /// keeps its way round and its start.
    pub fn reverse(&mut self) {
        if !self.closed {
            self.points.reverse();
            self.overhang.reverse();
        }
    }
}

/// The beads of one layer, in printing order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PathLayer {
    pub index: usize,
    pub top: f64,
    pub height: f64,
    /// The material this pass of the layer prints.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub material: usize,
    /// How far the layer's top lies above `top` over the plate: the beads lie on it.
    #[serde(default, skip_serializing_if = "Relief::is_zero")]
    pub warp: Relief,
    pub paths: Vec<Path>,
}

/// The output of the path stage: the layers' passes, in printing order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Paths {
    pub layers: Vec<PathLayer>,
}

fn is_zero(n: &usize) -> bool {
    *n == 0
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
    /// follow the cheapest tour.
    #[serde(default)]
    pub monotone: Vec<Face>,
    /// How wide a dense band must be for its lines to cross the layer below rather than run
    /// along it, by face (`families`).
    pub crossing: Crossing,
    /// The printer's resolution, in mm: a bead along a thin strip keeps one width while the
    /// strip's thickness changes by less.
    #[serde(default)]
    pub resolution: f64,
    /// What the sparse lines draw, at the same spacing.
    #[serde(default)]
    pub sparse_pattern: SparsePattern,
    /// How the supports' areas are filled; none where the regions hold no supports.
    #[serde(default)]
    pub support: Option<SupportLines>,
}

/// The supports' lines: `width` wide, the body's `gap` and the interface's `interface_gap` apart
/// past their tiling spacing, in mm; a gap of 0 lays them solid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SupportLines {
    pub width: f64,
    pub gap: f64,
    pub interface_gap: f64,
}

/// How wide a dense band must be, in mm, for its lines to cross those below it: `v²/a` of the
/// beads laying it, which across a narrower band would never reach their speed before stopping.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Crossing {
    /// The top surface's.
    pub top: f64,
    /// Inside, the solid infill's.
    pub inside: f64,
    /// The first layer's.
    pub first_layer: f64,
}

/// What a sparse area's lines draw.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum SparsePattern {
    /// Straight lines, turning a quarter turn from layer to layer.
    #[default]
    Lines,
    /// Convection rolls from wall to wall, meeting the walls `tilt` degrees off square, the
    /// other way every `ply` mm of height, each layer's resting on the one below.
    Rolls { tilt: f64, ply: f64 },
}

/// Why the path stage failed.
#[derive(Debug, thiserror::Error)]
pub enum PathError {
    #[error("bad path parameters: {0}")]
    BadParams(String),
    #[error("bad path parameters: {0}")]
    Tune(#[from] TuneError),
    #[error("path geometry: {0}")]
    Geometry(#[from] GeomError),
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
    let Crossing { top, inside, first_layer } = p.crossing;
    if ![top, inside, first_layer].iter().all(|w| w.is_finite() && *w >= 0.0) {
        return Err(PathError::BadParams("crossing widths must be finite and not negative".into()));
    }
    if !p.infill_angle.is_finite() {
        return Err(PathError::BadParams("infill angle must be finite".into()));
    }
    if let Some(s) = &p.support
        && !(s.width.is_finite()
            && (0.05..=3.0).contains(&s.width)
            && [s.gap, s.interface_gap].iter().all(|g| g.is_finite() && (0.0..=50.0).contains(g)))
    {
        return Err(PathError::BadParams("support lines 0.05 to 3 mm wide, 0 to 50 mm apart".into()));
    }
    if let SparsePattern::Rolls { tilt, ply } = p.sparse_pattern
        && !(tilt.is_finite() && tilt.abs() < 90.0 && ply.is_finite() && ply >= 0.0)
    {
        return Err(PathError::BadParams(
            "rolls tilt less than 90° off square, by plies not negative".into(),
        ));
    }
    Ok(())
}

/// Turns every layer's regions into ordered beads, each island by the parameters at its centre.
pub fn trace(regions: &Regions, field: &Field<PathParams>) -> Result<Paths, PathError> {
    check(field.base())?;
    let mut layers = Vec::with_capacity(regions.layers.len());
    let mut at: Point = [0.0, 0.0];
    // Made on the first layer whose sparse area asks for them, over all the layers.
    let mut rolls: Option<rolls::Rolls> = None;
    // The dense beads of the layer below, and of the passes laid so far at this height.
    let (mut beneath, mut laid, mut height) = (families::Beneath::default(), Vec::new(), None);
    let solids = solids::Solids::of(&regions.layers, RegionLayer::outline)?;
    let mut seams = seam::Seams::default();
    let columns =
        solids::Solids::of(&regions.layers, |l| l.supports.iter().chain(&l.support_interface).collect())?;
    for (pass, layer) in regions.layers.iter().enumerate() {
        if height != Some(layer.top) {
            beneath = families::Beneath::new(std::mem::take(&mut laid));
            height = Some(layer.top);
        }
        let parts = order::parts(layer, field)?;
        let asked = parts.iter().find_map(|(p, _)| match p.sparse_pattern {
            SparsePattern::Rolls { tilt, ply } if !layer.sparse.is_empty() => Some((p, tilt, ply)),
            _ => None,
        });
        let lines = match asked {
            Some((p, tilt, ply)) => {
                let (width, spacing) = order::sparse_bead(layer, p);
                let grow = p.infill_wall_overlap * width;
                if rolls.is_none() {
                    rolls = rolls::Rolls::new(&regions.layers, spacing, grow, tilt, ply);
                }
                let area = layercraft_geom::offset(&layer.sparse, grow)?;
                rolls.as_mut().map(|r| r.lines(layer, &area, spacing)).transpose()?
            }
            None => None,
        };
        let mut islands = Vec::new();
        for (params, part) in parts {
            islands.extend(order::islands(
                &part,
                &params,
                lines.as_deref(),
                &beneath,
                &solids,
                pass,
                &mut seams,
            )?);
        }
        seams.commit();
        islands.extend(supports::islands(layer, field.base(), &columns, pass)?);
        let hidden = Bands::new(&link::hidden(layer)?);
        let paths = link::hide(order::sequence(islands, &mut at, &hidden), &hidden);
        for path in paths.iter().filter(|p| p.role.kind == Kind::Solid) {
            let points = path.points.iter().chain(path.points.first().filter(|_| path.closed));
            laid.extend(points.clone().zip(points.skip(1)).map(|(a, b)| (*a, *b, path.width)));
        }
        layers.push(PathLayer {
            index: layer.index,
            top: layer.top,
            height: layer.height,
            material: layer.material,
            warp: layer.warp.clone(),
            paths,
        });
    }
    Ok(Paths { layers })
}

#[cfg(test)]
mod seam_tests;
#[cfg(test)]
mod tests;
