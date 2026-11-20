//! Stage 2: what each part of a layer becomes — walls, solid surfaces or sparse infill.
//!
//! With `R_i` the section of layer `i`, the walls are the centre lines of successive insets of
//! `R_i`, one bead spacing apart — an inner one only where its bead passes, the band it leaves
//! filled as inside — and `A_i` is what lies inside the innermost wall. A point of `A_i` is
//! sparse infill when the inside continues, in `A_j`, for `top_layers` layers above it,
//! and the solid, in `R_j`, for `bottom_layers` below it; otherwise it is solid. What is laid
//! above stands on solid unless it is infill itself: under a thin rib or the strokes of raised
//! letters, all walls and no inside, the top shells run on underneath. Below, only the air needs
//! a skin over it. The part of `A_i` not covered by layer `i + 1` is the top surface.
//! Solid or top that the layer below does not hold up is a bridge, laid as hanging strands: over
//! air wider than a bead, deeper than a strand hangs and on no column (`supports`), reaching a
//! bead's width onto the support at its ends; over sparse infill, only where it is wide enough
//! to span a gap between the sparse lines (`sparse_gap`) — a narrower band rests on them. Over
//! shallower air, as an engraving a layer deep, a strand would lie on the floor and fill it.
//! Sparse infill under `min_sparse_area` is solid; first-layer beads are wider, to grip the bed.
//! With `one_wall_on_top`, inner walls stay only under the layer above, the top reaching the
//! outer wall. A layer keeps the sections below and above it and its `supports`, each column in
//! its filament's pass (`filaments`); each shape is split by the parameters at its centroid
//! (`layercraft_tune`), each material in its own pass, walled alone, judged against the layer.

use layercraft_geom::{
    Carrier, GeomError, Moments, Region, Shape, Warp, bead_spacing, difference, intersection, offset,
    opening, signed_area, union,
};
use layercraft_section::{Layer, Sections};
use layercraft_tune::{Field, TuneError};
use serde::{Deserialize, Serialize};
use std::ops::Range;
use std::rc::Rc;

mod filaments;
mod supports;

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
    /// The diameter of a bridge strand, the nozzle's: it hangs that far below the layer's top.
    pub bridge_width: f64,
    /// Pieces of sparse infill smaller than this are solid, in mm².
    #[serde(default)]
    pub min_sparse_area: f64,
    /// How the parts of layers that nothing holds up are supported: always, wherever a layer
    /// hangs, so that no bead is laid over air with nothing to hold it.
    pub supports: SupportParams,
}

/// Where supports stand, by the rules of `supports`; distances in mm.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SupportParams {
    /// The longest strand held at both ends that bridges without support; none, any.
    pub max_bridge: Option<f64>,
    /// From a column's top up to the underside of what it holds.
    pub top_gap: f64,
    /// From the part up to the foot of a column standing on it.
    pub bottom_gap: f64,
    /// From the part across to a column's side.
    pub xy_gap: f64,
    /// The dense layers at a column's top.
    pub interface_layers: usize,
    /// Columns only from the plate, none standing on the part.
    pub on_plate_only: bool,
    /// The material every column is laid in, over its whole height; none, each its own: the one
    /// the part prints on the most of its layers (`filaments`).
    #[serde(default)]
    pub column_material: Option<usize>,
    /// The material the interface is laid in, when not its column's: one that does not weld to
    /// the part lets the column come off it.
    #[serde(default)]
    pub interface_material: Option<usize>,
}

/// One pass of a layer, split by role: the layer's shapes of one material. Each wall is the region
/// whose boundary is that wall's centre line.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RegionPass {
    pub index: usize,
    pub top: f64,
    pub height: f64,
    /// The material this pass of the layer prints.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub material: usize,
    /// How far the layer's top lies above `top` at each corner of the stage's carrier, as its
    /// section gives it.
    #[serde(default, skip_serializing_if = "Warp::is_zero")]
    pub warp: Warp,
    /// Wall centre lines, outermost first.
    pub walls: Vec<Region>,
    /// Where inner walls are drawn, when not everywhere: under the layer above.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub inner_walls_within: Option<Region>,
    /// Inside the walls and exposed to the air above.
    pub top_surface: Region,
    /// Inside the walls, within the top or bottom shells, not exposed.
    pub solid: Region,
    /// Solid or top surface over air: strands hang from the edges of the gap.
    pub bridge: Region,
    /// Solid or top surface over sparse infill: strands hang from one sparse line to the next.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bridge_on_sparse: Region,
    /// Inside the walls, deep in the solid.
    pub sparse: Region,
    /// The section of the layer below, all its materials, which holds this layer's walls up;
    /// none on the first layer, which the bed holds.
    #[serde(default)]
    pub below: Option<Region>,
    /// The section of the layer above, which covers this layer; empty on the last layer.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub above: Region,
    /// The supports' columns on this layer, outside the part, below their interface: each a
    /// hollow shell, its loop along its edge.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub supports: Region,
    /// The columns' top `interface_layers`, which span them and what they hold rests on.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub support_interface: Region,
}

impl RegionPass {
    /// The infill areas: top surface, solid, bridges over air and over sparse, then sparse.
    pub fn infill(&self) -> [&Region; 5] {
        [&self.top_surface, &self.solid, &self.bridge, &self.bridge_on_sparse, &self.sparse]
    }

    pub fn infill_mut(&mut self) -> [&mut Region; 5] {
        [
            &mut self.top_surface,
            &mut self.solid,
            &mut self.bridge,
            &mut self.bridge_on_sparse,
            &mut self.sparse,
        ]
    }

    /// The shapes that bound the pass's islands: the outer wall's, or without walls the infill's.
    pub fn outline(&self) -> Vec<&Shape> {
        match self.walls.first() {
            Some(outer) => outer.iter().collect(),
            None => self.infill().into_iter().flatten().collect(),
        }
    }
}

/// The output of the region stage: the layers' passes, in printing order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Regions {
    /// The triangles the bent layers' warps are linear on, as the sections give them.
    #[serde(default, skip_serializing_if = "Carrier::is_empty")]
    pub carrier: Carrier,
    #[serde(alias = "layers")]
    pub passes: Vec<RegionPass>,
}

impl Regions {
    /// The passes of each layer, layer by layer: the runs of passes sharing an index.
    pub fn layers(&self) -> impl Iterator<Item = &[RegionPass]> {
        self.passes.chunk_by(|a, b| a.index == b.index)
    }
}

fn is_zero(n: &usize) -> bool {
    *n == 0
}

/// Why the region stage failed.
#[derive(Debug, thiserror::Error)]
pub enum RegionError {
    #[error("bad region parameters: {0}")]
    BadParams(String),
    #[error("bad region parameters: {0}")]
    Tune(#[from] TuneError),
    #[error("region geometry: {0}")]
    Geometry(#[from] GeomError),
    /// A part of a layer hangs where no support can stand under it: printed, it would fall. The
    /// layer is counted from 1, as the printer counts it.
    #[error(
        "layer {layer} (top at z {z:.2} mm) hangs {area:.2} mm² over air where no support can stand \
         under it, so the print would fail there"
    )]
    Unheld { layer: usize, z: f64, area: f64 },
}

fn check(params: &RegionParams) -> Result<(), RegionError> {
    let width = |w: f64| w.is_finite() && (0.05..=3.0).contains(&w);
    if ![params.outer_wall_width, params.inner_wall_width, params.first_layer_width, params.bridge_width]
        .into_iter()
        .all(width)
    {
        return Err(RegionError::BadParams("wall and strand widths must be 0.05 to 3 mm".into()));
    }
    if !params.sparse_gap.is_finite() || params.sparse_gap < 0.0 {
        return Err(RegionError::BadParams("sparse gap must be finite and not negative".into()));
    }
    if params.walls > 50 || params.top_layers > 200 || params.bottom_layers > 200 {
        return Err(RegionError::BadParams("too many walls or shell layers".into()));
    }
    // The layer above decides what stands on solid, as a rib does, and only a skin a layer thick
    // keeps the sparse infill off the air below.
    if params.top_layers == 0 || params.bottom_layers == 0 {
        return Err(RegionError::BadParams("a part needs a top and a bottom shell layer at least".into()));
    }
    let s = &params.supports;
    let gap = |g: f64| g.is_finite() && (0.0..=10.0).contains(&g);
    if ![s.top_gap, s.bottom_gap, s.xy_gap].into_iter().all(gap)
        || s.max_bridge.is_some_and(|m| !(m.is_finite() && m >= 0.0))
        || s.interface_layers > 50
    {
        return Err(RegionError::BadParams(
            "support gaps must be 0 to 10 mm, the bridge finite, interface layers few".into(),
        ));
    }
    // A column is a hollow shell: only its interface spans it, under what it holds.
    if s.interface_layers == 0 {
        return Err(RegionError::BadParams("supports need an interface layer at least".into()));
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

/// The distance from the solid's edge to where `n` walls' material ends, on a layer `h` high.
/// The outer bead's side touches the edge; inward, beads sit their tiling spacing apart
/// (`bead_spacing`), so the walls fill their band without voids.
fn walls_depth(w: Widths, n: usize, h: f64) -> f64 {
    let (outer, inner) = (bead_spacing(w.outer, h), bead_spacing(w.inner, h));
    match n {
        0 => 0.0,
        n => w.outer / 2.0 + outer / 2.0 + (n - 1) as f64 * inner,
    }
}

/// The distance from the solid's edge to the centre line of wall `k` (0 = outer), on a layer
/// `h` high: half its spacing before its band ends.
fn wall_centre(w: Widths, k: usize, h: f64) -> f64 {
    let spacing = bead_spacing(if k == 0 { w.outer } else { w.inner }, h);
    walls_depth(w, k + 1, h) - spacing / 2.0
}

/// What the beads `2r` wide along the boundary of `centre` cover.
fn bead_cover(centre: &Region, r: f64) -> Result<Region, GeomError> {
    difference(&offset(centre, r)?, &offset(centre, -r)?)
}

/// Floors closer than this to a strand's bottom touch it, in mm.
const TOUCH_MM: f64 = 1e-6;

/// The part of `region`, on layer `i`, over air deeper than `depth` below the layer: outside
/// every section under it down to that depth. Empty where the bed is within it.
fn over_deep_air(
    region: &Region,
    areas: &[Region],
    heights: &[f64],
    i: usize,
    depth: f64,
) -> Result<Region, GeomError> {
    let mut air = region.clone();
    let mut reached = 0.0;
    let mut j = i;
    while reached <= depth + TOUCH_MM && !air.is_empty() {
        let Some(below) = j.checked_sub(1) else { return Ok(Vec::new()) };
        let (Some(section), Some(h)) = (areas.get(below), heights.get(below)) else {
            return Ok(Vec::new());
        };
        air = difference(&air, section)?;
        reached += h;
        j = below;
    }
    Ok(air)
}

/// The common part of the sections `range`, one or more; empty when the range runs past them.
fn common(sections: &[Region], range: Range<usize>) -> Result<Region, GeomError> {
    let mut acc: Option<Region> = None;
    for i in range {
        let Some(section) = sections.get(i) else { return Ok(Vec::new()) };
        acc = Some(match acc {
            None => section.clone(),
            Some(a) => intersection(&a, section)?,
        });
    }
    Ok(acc.unwrap_or_default())
}

/// Where the parameters of a shape are read, at height `z`: its centroid, which turns and moves
/// with it; its first point when it has no area.
fn centroid(shape: &Shape, z: f64) -> [f64; 3] {
    let at = Moments::of(std::iter::once((shape.as_slice(), 1.0))).map(|m| m.centroid);
    let [x, y] = at.or(shape.first().and_then(|c| c.first()).copied()).unwrap_or([0.0, 0.0]);
    [x, y, z]
}

/// The shapes of a layer grouped by their material and the parameters at their centroids, in
/// order of first use.
type Parts = Vec<(Rc<RegionParams>, usize, Region)>;

fn parts(layer: &Layer, field: &Field<RegionParams>) -> Result<Parts, RegionError> {
    let mut parts: Parts = Vec::new();
    for (material, region) in layer.by_material().into_iter().enumerate() {
        for shape in region {
            let p = field.at(centroid(shape, layer.top), check)?;
            match parts.iter_mut().find(|(q, m, _)| Rc::ptr_eq(q, &p) && *m == material) {
                Some((_, _, shapes)) => shapes.push(shape.clone()),
                None => parts.push((p, material, vec![shape.clone()])),
            }
        }
    }
    Ok(parts)
}

/// The materials of a layer's parts and `also`, starting with `first` when it has it, then in
/// order.
fn pass_order(parts: &Parts, also: impl IntoIterator<Item = usize>, first: Option<usize>) -> Vec<usize> {
    let mut order: Vec<usize> = parts.iter().map(|(_, m, _)| *m).chain(also).collect();
    order.sort_unstable();
    order.dedup();
    if let Some(at) = first.and_then(|f| order.iter().position(|&m| m == f)) {
        let head = order.remove(at);
        order.insert(0, head);
    }
    order
}

fn widths(p: &RegionParams, layer: usize) -> Widths {
    if layer == 0 {
        Widths { outer: p.first_layer_width, inner: p.first_layer_width }
    } else {
        Widths { outer: p.outer_wall_width, inner: p.inner_wall_width }
    }
}

/// No region: where nothing stands on a layer's columns.
const NOTHING: &Region = &Vec::new();

/// Splits every layer of the sections by role, each shape by the parameters at its centre.
/// Shapes that share their parameters are split together; the layer is their union.
pub fn split(sections: &Sections, field: &Field<RegionParams>) -> Result<Regions, RegionError> {
    let base = field.base();
    check(base)?;
    // Each layer's shapes, first-layer shrink applied, grouped by parameters.
    let mut layer_parts: Vec<Parts> = Vec::with_capacity(sections.layers.len());
    for layer in &sections.layers {
        let mut shrunk = Vec::new();
        for (p, material, shapes) in parts(layer, field)? {
            let shrink = if layer.index == 0 { p.first_layer_shrink } else { 0.0 };
            shrunk.push((p, material, offset(&shapes, -shrink)?));
        }
        layer_parts.push(shrunk);
    }
    // Each layer's area, all its parts together, and what lies inside each part's walls.
    let areas: Vec<Region> =
        layer_parts.iter().map(|parts| parts.iter().flat_map(|(_, _, r)| r.clone()).collect()).collect();
    let mut inside_parts: Vec<Vec<Region>> = Vec::with_capacity(areas.len());
    for (i, (layer, parts)) in sections.layers.iter().zip(&layer_parts).enumerate() {
        let mut insides = Vec::with_capacity(parts.len());
        for (p, _, region) in parts {
            insides.push(offset(region, -walls_depth(widths(p, i), p.walls, layer.height))?);
        }
        inside_parts.push(insides);
    }
    let insides: Vec<Region> =
        inside_parts.iter().map(|parts| parts.iter().flatten().cloned().collect()).collect();
    let heights: Vec<f64> = sections.layers.iter().map(|l| l.height).collect();
    // The supports, over the whole plate, by the base parameters: a column may stand under
    // several parts. What a column holds is laid on it, not hung over air.
    let tops: Vec<f64> = sections.layers.iter().map(|l| l.top).collect();
    let mut found = supports::supports(
        &areas,
        &tops,
        &heights,
        &base.supports,
        base.outer_wall_width,
        base.bridge_width,
    )?;
    if let Some((layer, loose)) = found.unheld.first() {
        let z = tops.get(*layer).copied().unwrap_or(f64::NAN);
        return Err(RegionError::Unheld { layer: layer + 1, z, area: layercraft_geom::area(loose) });
    }
    // Each column in the filament that lays it, by what the part prints on each layer.
    let printed: Vec<Vec<usize>> =
        layer_parts.iter().map(|parts| parts.iter().map(|(_, m, _)| *m).collect()).collect();
    let mut laid = filaments::by_filament(std::mem::take(&mut found.columns), &printed, &base.supports)?;
    let mut passes: Vec<RegionPass> = Vec::with_capacity(areas.len());
    let mut last_material = None;
    // The sparse infill of the layer below, all its materials, which a strand over it spans.
    let mut sparse_below: Option<Region> = None;
    for (i, ((layer, parts), part_insides)) in
        sections.layers.iter().zip(&layer_parts).zip(&inside_parts).enumerate()
    {
        let next = areas.get(i + 1).cloned().unwrap_or_default();
        let columns = laid.get_mut(i).map(std::mem::take).unwrap_or_default();
        let blank = RegionPass {
            index: layer.index,
            top: layer.top,
            height: layer.height,
            warp: layer.warp.clone(),
            below: i.checked_sub(1).and_then(|j| sections.layers.get(j)).map(|below| below.region.clone()),
            above: next.clone(),
            ..RegionPass::default()
        };
        // The columns take a pass of their filaments.
        let order = pass_order(parts, columns.iter().map(|(m, _)| *m), last_material);
        last_material = order.last().copied().or(last_material);
        let around = Around {
            next: &next,
            areas: &areas,
            heights: &heights,
            insides: &insides,
            held: found.held.get(i).unwrap_or(NOTHING),
            sparse_below: sparse_below.as_ref(),
        };
        // Each pass, with its parts whose inner walls run everywhere and whether others' stop
        // under the air.
        let mut own: Vec<(RegionPass, Region, bool)> = order
            .iter()
            .map(|&material| (RegionPass { material, ..blank.clone() }, Vec::new(), false))
            .collect();
        let mut sparse: Region = Vec::new();
        for ((p, material, region), inside) in parts.iter().zip(part_insides) {
            let mut piece = split_part(i, layer, p, region, inside, &around)?;
            let Some((pass, everywhere, clipped)) = own.iter_mut().find(|(l, _, _)| l.material == *material)
            else {
                continue;
            };
            sparse.extend(piece.sparse.iter().cloned());
            for (w, wall) in std::mem::take(&mut piece.walls).into_iter().enumerate() {
                match pass.walls.get_mut(w) {
                    Some(walls) => walls.extend(wall),
                    None => pass.walls.push(wall),
                }
            }
            for (all, part) in pass.infill_mut().into_iter().zip(piece.infill_mut()) {
                all.append(part);
            }
            match piece.inner_walls_within {
                Some(_) => *clipped = true,
                None => everywhere.extend(region.iter().cloned()),
            }
        }
        let mut own: Vec<RegionPass> = own
            .into_iter()
            .map(|(mut pass, everywhere, clipped)| {
                if clipped {
                    pass.inner_walls_within =
                        Some(if everywhere.is_empty() { next.clone() } else { union(&next, &everywhere)? });
                }
                Ok(pass)
            })
            .collect::<Result<_, RegionError>>()?;
        if own.is_empty() {
            own.push(blank);
        }
        for (material, column) in columns {
            if let Some(pass) = own.iter_mut().find(|pass| pass.material == material) {
                pass.supports = column.body;
                pass.support_interface = column.interface;
            }
        }
        sparse_below = Some(sparse);
        passes.extend(own);
    }
    Ok(Regions { carrier: sections.carrier.clone(), passes })
}

/// The layers around the one being split, whole.
struct Around<'a> {
    /// The section of the layer above.
    next: &'a Region,
    /// Every layer's area, height and what lies inside its walls.
    areas: &'a [Region],
    heights: &'a [f64],
    insides: &'a [Region],
    /// What of the layer stands on the supports' columns, a gap above them.
    held: &'a Region,
    /// The sparse infill of the layer below; none on the first layer.
    sparse_below: Option<&'a Region>,
}

/// Splits one part of layer `i`, `region` with `inside` inside its walls, by its parameters `p`.
fn split_part(
    i: usize,
    layer: &Layer,
    p: &RegionParams,
    region: &Region,
    inside: &Region,
    around: &Around,
) -> Result<RegionPass, RegionError> {
    let Around { next, areas, heights, insides, held, sparse_below } = *around;
    let h = layer.height;
    let widths = widths(p, i);
    // The outer wall follows the whole outline, the surface that shows. An inner wall runs only
    // where its bead passes: through a neck of its inset narrower than the bead, or along a cusp
    // where two rounded holes nearly meet, its loop would run in and back over its own plastic.
    // Corners keep their point, the offsets being mitred. The band it leaves uncovered, at least
    // half a bead wide, is filled with the inside.
    let mut walls = Vec::new();
    let mut inner_cover: Region = Vec::new();
    for k in 0..p.walls {
        let mut centre = offset(region, -wall_centre(widths, k, h))?;
        if k > 0 {
            centre = opening(&centre, widths.inner / 2.0)?;
            inner_cover = union(&inner_cover, &bead_cover(&centre, widths.inner / 2.0)?)?;
        }
        if centre.is_empty() {
            break;
        }
        walls.push(centre);
    }
    let inside = &if p.walls > 1 {
        let band = difference(&offset(region, -walls_depth(widths, 1, h))?, inside)?;
        let gap = opening(&difference(&band, &inner_cover)?, widths.inner / 4.0)?;
        union(inside, &gap)?
    } else {
        inside.clone()
    };
    // Inner walls under the air would only show through the top: there, the top surface
    // runs out to the outer wall and the inner walls stop.
    let (top_reach, inner_walls_within) = if p.one_wall_on_top && p.walls > 1 {
        (offset(region, -walls_depth(widths, 1, h))?, Some(next.clone()))
    } else {
        (inside.clone(), None)
    };
    // Sparse where the inside goes on for `top_layers` above and the section for
    // `bottom_layers` below.
    let above = common(insides, i + 1..i + 1 + p.top_layers)?;
    let below = match i.checked_sub(p.bottom_layers) {
        Some(from) => common(areas, from..i)?,
        None => Vec::new(),
    };
    let top_surface = difference(&top_reach, next)?;
    // A small piece of sparse infill would be a line or two holding nothing up: solid.
    let deep: Region = intersection(&intersection(inside, &above)?, &below)?
        .into_iter()
        .filter(|shape| shape.iter().map(|c| signed_area(c)).sum::<f64>() >= p.min_sparse_area)
        .collect();
    let shells = difference(inside, &deep)?;
    let solid = difference(&shells, &top_surface)?;
    // What is not held up from below: over air (the layer below is absent) and over
    // its sparse infill. Only the parts wide enough to need hanging strands are bridges, and
    // over air only where it is deep enough for them to hang: a strand reaches its diameter
    // below the layer's top, and a shallower floor holds it up.
    let filled = union(&solid, &top_surface)?;
    let (bridge, bridge_on_sparse) = match sparse_below {
        Some(sparse_below) => {
            let width = widths.inner.max(widths.outer);
            let air = over_deep_air(&filled, areas, heights, i, (p.bridge_width - h).max(0.0))?;
            let air = difference(&air, held)?;
            // Each strand rests a bead's width on the support at either end.
            let gap = opening(&air, width / 2.0)?;
            let over_air = intersection(&offset(&gap, width)?, &union(&filled, &deep)?)?;
            let over_sparse = opening(&intersection(&filled, sparse_below)?, p.sparse_gap / 2.0)?;
            let on_sparse = difference(&over_sparse, &over_air)?;
            (over_air, on_sparse)
        }
        None => (Vec::new(), Vec::new()),
    };
    let hanging = union(&bridge, &bridge_on_sparse)?;
    Ok(RegionPass {
        index: layer.index,
        top: layer.top,
        height: h,
        walls,
        inner_walls_within,
        top_surface: difference(&top_surface, &hanging)?,
        solid: difference(&solid, &hanging)?,
        bridge,
        bridge_on_sparse,
        sparse: difference(&deep, &hanging)?,
        ..RegionPass::default()
    })
}

#[cfg(test)]
mod tests;
