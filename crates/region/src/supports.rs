//! Supports: what holds up the parts of a layer the layer below does not, and comes off after.
//! The gap over a column is air under the roof's lowest strand, so the roof does not weld to it.
//!
//! A bead stands while its centre line lies over the layer below; its outer side may then reach
//! half its width `w` past it, the slope a bead's own shape allows. Beyond `w/2` past the
//! section below, a layer hangs. Strands across a hanging piece, a nozzle wide, are laid the way
//! that leaves the fewest of their ends over air, as the path stage lays a bridge's: a strand held
//! at both ends, and no longer than `max_bridge`, bridges; any other needs holding, and the band
//! a strand wide on either side of it, which reaches the edge past the outermost line, is a roof. Under each roof stands a column, its top `top_gap` below the roof's
//! underside, its sides `xy_gap` off the part, down to the plate or onto the part, ending
//! `bottom_gap` above it, or only to the plate (`on_plate_only`). The top `interface_layers` of
//! a column are its interface, dense, which the roof rests on; the rest is its body.

use crate::SupportParams;
use layercraft_geom::{
    Bands, GeomError, Moments, Point, Region, bridge_angle, difference, hatch, intersection, offset, opening,
    phase_through, polyline_length, union, winding_region,
};

/// Heights closer than this are the same, in mm.
const EPS: f64 = 1e-6;

/// Every layer's supports: the body of the columns, then their interface.
pub type LayerSupports = (Region, Region);

/// The band `2·half` wide along the segment `a`–`b`, counterclockwise.
fn band(a: Point, b: Point, half: f64) -> Option<Vec<Point>> {
    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
    if length.is_nan() || length <= 0.0 {
        return None;
    }
    let n = [-(b[1] - a[1]) / length * half, (b[0] - a[0]) / length * half];
    Some(vec![
        [a[0] - n[0], a[1] - n[1]],
        [b[0] - n[0], b[1] - n[1]],
        [b[0] + n[0], b[1] + n[1]],
        [a[0] + n[0], a[1] + n[1]],
    ])
}

/// The part of `section` that the section `below` does not hold up, for beads `w` wide and
/// strands `strand` wide.
pub fn roof(
    section: &Region,
    below: &Region,
    w: f64,
    strand: f64,
    max_bridge: Option<f64>,
) -> Result<Region, GeomError> {
    let held = offset(below, w / 2.0)?;
    let hang = opening(&difference(section, &held)?, w / 2.0)?;
    let max_bridge = max_bridge.unwrap_or(f64::INFINITY);
    if max_bridge <= 0.0 || hang.is_empty() {
        return Ok(hang);
    }
    let rests = Bands::new(&held);
    let mut loose = Vec::new();
    for shape in &hang {
        // The strands reach a bead's width onto what holds them.
        let piece = intersection(&offset(&vec![shape.clone()], w)?, section)?;
        // The piece's own direction and centre, not the plate's, lay its strands.
        let own = Moments::of(piece.iter().map(|shape| (shape.as_slice(), 1.0)));
        let (preferred, centre) = own.map_or((0.0, [0.0, 0.0]), |m| {
            (m.axes.map_or(0.0, |a| a.first[1].atan2(a.first[0]).to_degrees()), m.centroid)
        });
        let phase = phase_through(preferred, centre);
        let angle = bridge_angle(&piece, strand, preferred, phase, |p| !rests.covers(p))?;
        let phase = phase_through(angle, centre);
        for line in hatch(&piece, strand, angle, phase)? {
            let ends = [line.first(), line.last()];
            let anchored = ends.iter().flatten().all(|&&p| rests.covers(p));
            if anchored && polyline_length(&line) <= max_bridge {
                continue;
            }
            loose.extend(line.windows(2).filter_map(|s| match s {
                [a, b] => band(*a, *b, strand),
                _ => None,
            }));
        }
    }
    opening(&intersection(&hang, &winding_region(&loose)?)?, w / 2.0)
}

/// The supports of every layer, from the part's `sections`, each layer's `tops` and `heights`,
/// for beads `w` wide and strands `strand` wide.
pub fn supports(
    sections: &[Region],
    tops: &[f64],
    heights: &[f64],
    p: &SupportParams,
    w: f64,
    strand: f64,
) -> Result<Vec<LayerSupports>, GeomError> {
    let n = sections.len().min(tops.len()).min(heights.len());
    // The roofs whose columns start on each layer: the highest whose top lies `top_gap` under
    // the roof's underside.
    let mut starting: Vec<Region> = vec![Vec::new(); n];
    for i in 1..n {
        let (Some(section), Some(below), Some(&top), Some(&h)) =
            (sections.get(i), sections.get(i - 1), tops.get(i), heights.get(i))
        else {
            continue;
        };
        let r = roof(section, below, w, strand, p.max_bridge)?;
        if r.is_empty() {
            continue;
        }
        // The roof's first layer is laid as a bridge: round strands a strand wide, hanging from
        // the nozzle at the layer's top, reach below the layer when they are thicker than it.
        let underside = top - h.max(strand) - p.top_gap;
        let start = (0..i).rev().find(|&j| tops.get(j).is_some_and(|&t| t <= underside + EPS));
        if let Some(slot) = start.and_then(|j| starting.get_mut(j)) {
            *slot = union(slot, &r)?;
        }
    }
    // With `on_plate_only`, a column stops wherever the part lies under it, all the way down.
    let mut shadows: Vec<Region> = Vec::with_capacity(n);
    if p.on_plate_only {
        let mut shadow: Region = Vec::new();
        for section in sections.iter().take(n) {
            shadows.push(shadow.clone());
            shadow = union(&shadow, section)?;
        }
    }
    let mut out: Vec<LayerSupports> = vec![(Vec::new(), Vec::new()); n];
    let mut column: Region = Vec::new();
    for j in (0..n).rev() {
        let (Some(section), Some(&top), Some(&h)) = (sections.get(j), tops.get(j), heights.get(j)) else {
            continue;
        };
        if let Some(start) = starting.get(j) {
            column = union(&column, start)?;
        }
        if column.is_empty() {
            continue;
        }
        column = difference(&column, &offset(section, p.xy_gap)?)?;
        if let Some(shadow) = shadows.get(j) {
            column = difference(&column, shadow)?;
        }
        // The part within `bottom_gap` under the layer: the column ends above it.
        let floor = top - h - p.bottom_gap;
        let mut body = column.clone();
        for k in (0..j).rev().take_while(|&k| tops.get(k).is_some_and(|&t| t > floor + EPS)) {
            if let Some(under) = sections.get(k) {
                body = difference(&body, under)?;
            }
        }
        let body = opening(&body, w / 2.0)?;
        // The top layers of each column: within `interface_layers` of where it starts.
        let mut fresh: Region = Vec::new();
        for start in starting.iter().skip(j).take(p.interface_layers) {
            fresh = union(&fresh, start)?;
        }
        let interface = intersection(&body, &fresh)?;
        if let Some(slot) = out.get_mut(j) {
            *slot = (difference(&body, &interface)?, interface);
        }
    }
    Ok(out)
}
