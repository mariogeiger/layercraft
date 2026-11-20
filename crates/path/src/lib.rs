//! Stage 3: the lines the nozzle draws, in printing order.
//!
//! Walls become closed loops, starting at the concave corner nearest the seam of the layer below
//! (`seam`), so seams line up from layer to layer. Dense areas are laid at the beads' tiling
//! spacing as families of parallel curves with the fewest bead ends (`families`): a band narrower
//! than its beads' crossing width along itself, in insets of its edge, a wider area in straight
//! lines along or across its edges, crossing the dense beads of the layer below. Sparse areas get
//! lines spaced for the requested density by volume, in the solid's own frame, turning a quarter
//! turn each layer, so no two beads cross on a layer, or convection rolls as far apart (`rolls`),
//! running from wall to wall and resting on those below. Bridges are strands one nozzle wide: side
//! by side over air, where they hang; over sparse infill, where they rest, spaced to fill their
//! layer exactly. Each island is printed whole, inner walls, outer wall, then its infill
//! (`island`), and islands follow each other nearest first; within an island, the beads of each
//! kind follow the tour that crosses laid beads least often, then travels least, and the lines of
//! a `monotone` face are laid in sweeps instead, each against the one before (`order`). A move
//! that crosses a bead already laid at its height is marked (`crossing`). The first layer's beads
//! are all `first_layer_width` wide. Each wall segment carries the share of its bead that hangs
//! past the layer below (`overhang`). Each island is traced by the parameters at its centroid. The
//! supports are islands of their own (`supports`).

mod crossing;
mod families;
mod frame;
mod gap;
mod island;
mod near;
mod order;
mod overhang;
mod role;
mod rolls;
mod seam;
mod solids;
mod supports;
mod tour;

pub use role::{Face, Kind, Rest, Role};

use layercraft_geom::{Carrier, GeomError, Point, Region, Warp, union};
use layercraft_region::{RegionPass, Regions};
use layercraft_tune::{Field, TuneError};
use serde::{Deserialize, Serialize};
use std::rc::Rc;

/// One bead: a polyline drawn `width` wide, closed back to its start when `closed`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Path {
    pub role: Role,
    pub width: f64,
    pub closed: bool,
    pub points: Vec<Point>,
    /// The move here crosses a bead already laid at this height: the nozzle goes over it, lifted.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub travel_crosses: bool,
    /// How far each segment hangs over air, as a fraction of the bead's width from 0 to 1,
    /// segment `i` running from point `i` to the next; empty where nothing hangs.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overhang: Vec<f64>,
}

impl Path {
    /// A bead along `points`, reached by an ordinary move and hanging nowhere.
    pub fn new(role: Role, width: f64, closed: bool, points: Vec<Point>) -> Path {
        Path { role, width, closed, points, travel_crosses: false, overhang: Vec::new() }
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

/// The beads of one pass of a layer, in printing order: the layer's beads of one material.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PathPass {
    pub index: usize,
    pub top: f64,
    pub height: f64,
    /// The material this pass of the layer prints.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub material: usize,
    /// How far the layer's top lies above `top` at each corner of the stage's carrier: the beads
    /// lie on it.
    #[serde(default, skip_serializing_if = "Warp::is_zero")]
    pub warp: Warp,
    pub paths: Vec<Path>,
}

/// The output of the path stage: the layers' passes, in printing order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Paths {
    /// The triangles the bent layers' warps are linear on, as the regions give them.
    #[serde(default, skip_serializing_if = "Carrier::is_empty")]
    pub carrier: Carrier,
    #[serde(alias = "layers")]
    pub passes: Vec<PathPass>,
}

impl Paths {
    /// The passes of each layer, layer by layer: the runs of passes sharing an index.
    pub fn layers(&self) -> impl Iterator<Item = &[PathPass]> {
        self.passes.chunk_by(|a, b| a.index == b.index)
    }
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
    /// Fraction of the area the sparse lines cover, from 0, none, to 1.
    pub sparse_density: f64,
    /// How far infill reaches into the innermost wall, as a fraction of the infill width.
    pub infill_wall_overlap: f64,
    /// The turn, in degrees, from each solid's own first axis of the straight lines of even
    /// layers: sparse lines and strands resting on sparse infill. Dense areas and bridges over
    /// air choose their own.
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
    /// How the supports' areas are filled.
    pub support: SupportLines,
}

/// The supports' lines: `width` wide, the interface's `interface_gap` farther apart than beads
/// that tile, in mm; a gap of 0 lays it solid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SupportLines {
    pub width: f64,
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

impl PathParams {
    /// The width of a bead `width` wide on layer `index`: the first layer's are all
    /// `first_layer_width`, to grip the bed.
    pub fn width_on(&self, index: usize, width: f64) -> f64 {
        if index == 0 { self.first_layer_width } else { width }
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
    let Crossing { top, inside, first_layer } = p.crossing;
    if ![top, inside, first_layer].iter().all(|w| w.is_finite() && *w >= 0.0) {
        return Err(PathError::BadParams("crossing widths must be finite and not negative".into()));
    }
    if !p.infill_angle.is_finite() {
        return Err(PathError::BadParams("infill angle must be finite".into()));
    }
    let s = &p.support;
    if !(s.width.is_finite()
        && (0.05..=3.0).contains(&s.width)
        && s.interface_gap.is_finite()
        && (0.0..=50.0).contains(&s.interface_gap))
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

/// Turns every layer's regions into ordered beads, each island by the parameters at its centroid.
pub fn trace(regions: &Regions, field: &Field<PathParams>) -> Result<Paths, PathError> {
    check(field.base())?;
    let solids = solids::Solids::of(&regions.passes, RegionPass::outline)?;
    let columns =
        solids::Solids::of(&regions.passes, |l| l.supports.iter().chain(&l.support_interface).collect())?;
    let mut rolls = rolls::SolidRolls::new(&regions.passes, &solids);
    let mut seams = seam::Seams::default();
    let mut at: Point = [0.0, 0.0];
    // The dense beads of the layer below, and its supports' interface.
    let (mut beneath, mut interface_below) = (families::Beneath::default(), Vec::new());
    let mut passes = Vec::with_capacity(regions.passes.len());
    for layer in regions.layers() {
        let first = passes.len();
        let parts: Vec<_> = layer.iter().map(|pass| island::parts(pass, field)).collect::<Result<_, _>>()?;
        let numbered: Vec<_> =
            layer.iter().zip(&parts).enumerate().map(|(k, (pass, parts))| (first + k, pass, parts)).collect();
        let lines = roll_lines(&mut rolls, &solids, &numbered)?;
        // Every bead of the passes laid so far at this height, their dense ones, and their
        // supports' interface.
        let (mut earlier, mut dense, mut interface) = (Vec::new(), Vec::new(), Vec::new());
        for (k, (pass, parts)) in layer.iter().zip(&parts).enumerate() {
            let index = first + k;
            interface = union(&interface, &pass.support_interface)?;
            let relief = regions.carrier.relief(&pass.warp)?;
            let mut islands = Vec::new();
            for (params, part) in parts {
                let rolls = lines.iter().find(|(q, _)| Rc::ptr_eq(q, params)).map(|(_, lines)| lines);
                let guides = island::Guides { rolls, beneath: &beneath, relief: &relief };
                islands.extend(island::islands(part, params, guides, &solids, index, &mut seams)?);
            }
            seams.commit();
            islands.extend(supports::islands(pass, field.base(), &columns, index, &interface_below)?);
            let paths = order::sequence(islands, &mut at, &earlier);
            earlier.extend(paths.iter().flat_map(crossing::segments));
            for path in paths.iter().filter(|p| p.role.kind == Kind::Solid) {
                dense.extend(crossing::segments(path).into_iter().map(|(a, b)| (a, b, path.width)));
            }
            passes.push(PathPass {
                index: pass.index,
                top: pass.top,
                height: pass.height,
                material: pass.material,
                warp: pass.warp.clone(),
                paths,
            });
        }
        beneath = families::Beneath::new(dense);
        interface_below = interface;
    }
    Ok(Paths { carrier: regions.carrier.clone(), passes })
}

/// The parts of a layer's passes, each pass with its number in the print.
type LayerParts<'a> = [(usize, &'a RegionPass, &'a Vec<(Rc<PathParams>, RegionPass)>)];

/// The sparse areas of a layer that ask for rolls by one set of parameters, each with the
/// number of its pass, grown by `grow` into the walls, the rolls `spacing` apart.
struct Ask {
    params: Rc<PathParams>,
    spacing: f64,
    grow: f64,
    areas: Vec<(usize, Region)>,
}

/// The rolls' lines of each set of parameters asking for them among `parts`, over the sparse
/// areas of all their passes at once, each grown into the walls as its infill reaches.
fn roll_lines(
    rolls: &mut rolls::SolidRolls,
    solids: &solids::Solids,
    parts: &LayerParts,
) -> Result<Vec<(Rc<PathParams>, rolls::RollLines)>, PathError> {
    let mut asks: Vec<Ask> = Vec::new();
    let mut at = None;
    for &(index, pass, parts) in parts {
        at = Some((pass.index, pass.top));
        for (p, part) in parts.iter().filter(|(_, part)| !part.sparse.is_empty()) {
            let SparsePattern::Rolls { .. } = p.sparse_pattern else { continue };
            let (width, spacing) = island::sparse_bead(part, p);
            let grow = p.infill_wall_overlap * width;
            let area = layercraft_geom::offset(&part.sparse, grow)?;
            match asks.iter_mut().find(|ask| Rc::ptr_eq(&ask.params, p)) {
                Some(ask) => ask.areas.push((index, area)),
                None => asks.push(Ask { params: p.clone(), spacing, grow, areas: vec![(index, area)] }),
            }
        }
    }
    let Some(at) = at else { return Ok(Vec::new()) };
    let mut out = Vec::with_capacity(asks.len());
    for Ask { params, spacing, grow, areas } in asks {
        let SparsePattern::Rolls { tilt, ply } = params.sparse_pattern else { continue };
        let areas: Vec<(usize, &Region)> = areas.iter().map(|(index, area)| (*index, area)).collect();
        let lines = rolls.lines(at, &params, (spacing, grow), (tilt, ply), &areas, solids)?;
        out.push((params, lines));
    }
    Ok(out)
}

#[cfg(test)]
mod tests;
