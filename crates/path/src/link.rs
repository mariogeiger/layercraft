//! Marking the moves from bead to bead that stay inside the part, so the nozzle goes on without
//! retracting or lifting. The move lays no plastic: an area's density stays what its spacing
//! gives, with nothing doubled along its edge.
//!
//! A move qualifies when it stays inside a region: it may run along its edge, never across a
//! hole or out of a concave corner. Between neighbouring lines of one infill area, the region is
//! the area, and the move short: a few line spacings, as between neighbouring hatch lines. Any
//! move qualifies over the part of the layer that the layer above covers, bridges over air
//! aside: what oozes on the way is buried there, out of sight.

use crate::Path;
use crate::Role;
use layercraft_geom::{Bands, GeomError, Point, Region, difference, intersection};
use layercraft_region::RegionLayer;

/// One infill area of a layer: its role, the region its lines fill, and the longest join.
pub struct Fill {
    pub role: Role,
    pub reach: Region,
    pub max_link: f64,
    /// Whether its lines are laid in sweeps, each against the one before.
    pub monotone: bool,
}

/// Lengths below this are a point, in mm.
const EPS: f64 = 1e-6;

fn distance(a: Point, b: Point) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

/// Whether a move from `a` to `b` stays over `hidden`, or goes nowhere.
pub fn hides(hidden: &Bands, a: Point, b: Point) -> bool {
    distance(a, b) < EPS || hidden.holds(a, b)
}

/// Whether a move from `a` to `b` joins two lines of `fill`, whose area is `reach`: short, and
/// inside it.
pub fn joins(fill: &Fill, reach: &Bands, a: Point, b: Point) -> bool {
    let gap = distance(a, b);
    gap <= fill.max_link && (gap < EPS || reach.holds(a, b))
}

/// The part of a layer where a move needs no retraction: inside its outer wall (inside its
/// areas, with none), covered by the layer above, and not hanging over air.
pub fn hidden(layer: &RegionLayer) -> Result<Region, GeomError> {
    let outline: Region = layer.outline().into_iter().cloned().collect();
    difference(&intersection(&outline, &layer.above)?, &layer.bridge)
}

/// Marks the beads of a layer, in printing order, reached from the bead before by a move over
/// `hidden`.
pub fn hide(mut paths: Vec<Path>, hidden: &Bands) -> Vec<Path> {
    let mut end: Option<Point> = None;
    for path in &mut paths {
        if let (Some(a), Some(&b)) = (end, path.points.first())
            && !path.travel_inside
        {
            path.travel_inside = hides(hidden, a, b);
        }
        end = path.end();
    }
    paths
}

/// Marks the lines of `paths` (in printing order, all of `fill`'s role, its area `reach`)
/// reached from the line before by a move that joins them.
pub fn link(mut paths: Vec<Path>, fill: &Fill, reach: &Bands) -> Vec<Path> {
    let mut end: Option<Point> = None;
    for path in &mut paths {
        if let (Some(a), Some(&b)) = (end, path.points.first()) {
            path.travel_inside = joins(fill, reach, a, b);
        }
        end = path.end();
    }
    paths
}
