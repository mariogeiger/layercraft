//! Stage 2: what each part of a layer becomes — walls, solid surfaces or sparse infill.
//!
//! With `R_i` the section of layer `i`, the walls are the centre lines of successive insets of
//! `R_i`, one bead spacing apart — an inner one only where its bead passes, the band it leaves
//! filled as inside — and `A_i` is what lies inside the innermost wall. A point of
//! `A_i` is sparse infill when the inside continues, in `A_j`, for `top_layers` layers above it,
//! and the solid, in `R_j`, for `bottom_layers` below it; otherwise it is solid. What is laid
//! above stands on solid unless it is infill itself: under a thin rib or the strokes of raised
//! letters, all walls and no inside, the top shells run on underneath. Below, only the air
//! needs a skin over it: what rests on walls is held up.
//! The part of `A_i` not covered by layer `i + 1` is the top surface, the one exposed to the air.
//! Solid or top that the layer below does not hold up is a bridge, laid as hanging strands: over
//! air, wherever it is wider than a bead and deeper than a strand hangs below the layer, reaching
//! a bead's width onto the support at its ends; over sparse infill, only where it is wide enough
//! to span a gap between the sparse lines (`sparse_gap`) — a narrower band rests on them. Over
//! shallower air, as an engraving a layer deep, a strand would lie on the floor and fill it.
//! A piece of sparse infill smaller than `min_sparse_area` is solid instead. The first layer's
//! beads are wider, to grip the bed. With `one_wall_on_top`, inner walls stay only where the
//! layer above covers them, and the top surface reaches the outer wall. A layer keeps the sections
//! below and above it, and each shape is split by the parameters at its centre (`layercraft_tune`).
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
use layercraft_section::{Layer, Sections};
use layercraft_tune::{Field, TuneError, centre};
use serde::{Deserialize, Serialize};
use std::rc::Rc;

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
    /// The section of the layer below, which holds this layer's walls up; none on the first
    /// layer, which the bed holds.
    #[serde(default)]
    pub support: Option<Region>,
    /// The section of the layer above, which covers this layer; empty on the last layer.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub above: Region,
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

impl From<TuneError> for RegionError {
    fn from(e: TuneError) -> Self {
        RegionError::BadParams(e.to_string())
    }
}

impl From<GeomError> for RegionError {
    fn from(e: GeomError) -> Self {
        RegionError::Geometry(e)
    }
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
    regions: &[Region],
    heights: &[f64],
    i: usize,
    depth: f64,
) -> Result<Region, GeomError> {
    let mut air = region.clone();
    let mut reached = 0.0;
    let mut j = i;
    while reached <= depth + TOUCH_MM && !air.is_empty() {
        let Some(below) = j.checked_sub(1) else { return Ok(Vec::new()) };
        let (Some(section), Some(h)) = (regions.get(below), heights.get(below)) else {
            return Ok(Vec::new());
        };
        air = difference(&air, section)?;
        reached += h;
        j = below;
    }
    Ok(air)
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

/// The shapes of a layer grouped by the parameters at their centres, in order of first use.
type Parts = Vec<(Rc<RegionParams>, Region)>;

fn parts(region: &Region, z: f64, field: &Field<RegionParams>) -> Result<Parts, RegionError> {
    let mut parts: Parts = Vec::new();
    for shape in region {
        let p = field.at(centre(shape.first().into_iter().flatten(), z), check)?;
        match parts.iter_mut().find(|(q, _)| Rc::ptr_eq(q, &p)) {
            Some((_, shapes)) => shapes.push(shape.clone()),
            None => parts.push((p, vec![shape.clone()])),
        }
    }
    Ok(parts)
}

fn widths(p: &RegionParams, layer: usize) -> Widths {
    if layer == 0 {
        Widths { outer: p.first_layer_width, inner: p.first_layer_width }
    } else {
        Widths { outer: p.outer_wall_width, inner: p.inner_wall_width }
    }
}

/// Splits every layer of the sections by role, each shape by the parameters at its centre.
/// Shapes that share their parameters are split together; the layer is their union.
pub fn split(sections: &Sections, field: &Field<RegionParams>) -> Result<Regions, RegionError> {
    check(field.base())?;
    // Each layer's shapes, first-layer shrink applied, grouped by parameters.
    let mut layer_parts: Vec<Parts> = Vec::with_capacity(sections.layers.len());
    for layer in &sections.layers {
        let mut shrunk = Vec::new();
        for (p, shapes) in parts(&layer.region, layer.top, field)? {
            let shrink = if layer.index == 0 { p.first_layer_shrink } else { 0.0 };
            shrunk.push((p, offset(&shapes, -shrink)?));
        }
        layer_parts.push(shrunk);
    }
    let regions: Vec<Region> =
        layer_parts.iter().map(|parts| parts.iter().flat_map(|(_, r)| r.clone()).collect()).collect();
    // What lies inside each part's walls.
    let mut inside_parts: Vec<Vec<Region>> = Vec::with_capacity(regions.len());
    for (i, (layer, parts)) in sections.layers.iter().zip(&layer_parts).enumerate() {
        let mut insides = Vec::with_capacity(parts.len());
        for (p, region) in parts {
            insides.push(offset(region, -walls_depth(widths(p, i), p.walls, layer.height))?);
        }
        inside_parts.push(insides);
    }
    let insides: Vec<Region> =
        inside_parts.iter().map(|parts| parts.iter().flatten().cloned().collect()).collect();
    let heights: Vec<f64> = sections.layers.iter().map(|l| l.height).collect();
    let mut layers: Vec<RegionLayer> = Vec::with_capacity(regions.len());
    for (i, ((layer, parts), part_insides)) in
        sections.layers.iter().zip(&layer_parts).zip(&inside_parts).enumerate()
    {
        let next = covered(&regions, i as isize + 1, 1)?;
        let mut out = RegionLayer {
            index: layer.index,
            top: layer.top,
            height: layer.height,
            walls: Vec::new(),
            inner_walls_within: None,
            top_surface: Vec::new(),
            solid: Vec::new(),
            bridge: Vec::new(),
            bridge_on_sparse: Vec::new(),
            sparse: Vec::new(),
            support: i.checked_sub(1).and_then(|j| sections.layers.get(j)).map(|below| below.region.clone()),
            above: next.clone(),
        };
        // Parts whose inner walls run everywhere, when others' stop under the air.
        let mut walls_everywhere: Region = Vec::new();
        let mut walls_clipped = false;
        for ((p, region), inside) in parts.iter().zip(part_insides) {
            let around = Around {
                next: &next,
                regions: &regions,
                heights: &heights,
                insides: &insides,
                below: layers.last(),
            };
            let piece = split_part(i, layer, p, region, inside, &around)?;
            for (k, wall) in piece.walls.into_iter().enumerate() {
                match out.walls.get_mut(k) {
                    Some(walls) => walls.extend(wall),
                    None => out.walls.push(wall),
                }
            }
            match piece.inner_walls_within {
                Some(_) => walls_clipped = true,
                None => walls_everywhere.extend(region.iter().cloned()),
            }
            out.top_surface.extend(piece.top_surface);
            out.solid.extend(piece.solid);
            out.bridge.extend(piece.bridge);
            out.bridge_on_sparse.extend(piece.bridge_on_sparse);
            out.sparse.extend(piece.sparse);
        }
        if walls_clipped {
            out.inner_walls_within =
                Some(if walls_everywhere.is_empty() { next } else { union(&next, &walls_everywhere)? });
        }
        layers.push(out);
    }
    Ok(Regions { layers })
}

/// The layers around the one being split, whole.
struct Around<'a> {
    /// The section of the layer above.
    next: &'a Region,
    /// Every layer's section, height and what lies inside its walls.
    regions: &'a [Region],
    heights: &'a [f64],
    insides: &'a [Region],
    /// The layer below, split.
    below: Option<&'a RegionLayer>,
}

/// Splits one part of layer `i`, `region` with `inside` inside its walls, by its parameters `p`.
fn split_part(
    i: usize,
    layer: &Layer,
    p: &RegionParams,
    region: &Region,
    inside: &Region,
    around: &Around,
) -> Result<RegionLayer, RegionError> {
    let Around { next, regions, heights, insides, below: previous } = *around;
    let h = layer.height;
    let widths = widths(p, i);
    let i = i as isize;
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
    let above = covered(insides, i + 1, p.top_layers)?;
    let below = covered(regions, i - p.bottom_layers as isize, p.bottom_layers)?;
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
    let (bridge, bridge_on_sparse) = match (usize::try_from(i), previous) {
        (Ok(index), Some(previous)) => {
            let width = widths.inner.max(widths.outer);
            let air = over_deep_air(&filled, regions, heights, index, (p.bridge_width - h).max(0.0))?;
            // Each strand rests a bead's width on the support at either end.
            let gap = opening(&air, width / 2.0)?;
            let over_air = intersection(&offset(&gap, width)?, &union(&filled, &deep)?)?;
            let over_sparse = opening(&intersection(&filled, &previous.sparse)?, p.sparse_gap / 2.0)?;
            (over_air.clone(), difference(&over_sparse, &over_air)?)
        }
        _ => (Vec::new(), Vec::new()),
    };
    let hanging = union(&bridge, &bridge_on_sparse)?;
    Ok(RegionLayer {
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
        support: None,
        above: Vec::new(),
    })
}

#[cfg(test)]
mod tests;
