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
use layercraft_geom::{
    GeomError, Point, Region, difference, edges, encloses, intersection, segment_distance,
};
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

fn cross(o: Point, a: Point, b: Point) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

/// Whether segments `ab` and `cd` cross at a point inside both (touching or running along
/// each other does not count).
fn crosses(a: Point, b: Point, c: Point, d: Point) -> bool {
    let (d1, d2) = (cross(a, b, c), cross(a, b, d));
    let (d3, d4) = (cross(c, d, a), cross(c, d, b));
    let scale = ((b[0] - a[0]).hypot(b[1] - a[1])).max((d[0] - c[0]).hypot(d[1] - c[1])) * EPS;
    (d1 > scale && d2 < -scale || d1 < -scale && d2 > scale)
        && (d3 > scale && d4 < -scale || d3 < -scale && d4 > scale)
}

fn region_edges(region: &Region) -> impl Iterator<Item = (Point, Point)> + '_ {
    region.iter().flatten().flat_map(|c| edges(c))
}

/// Whether a point is inside the region or on its edge (even-odd over all its contours).
fn covers(region: &Region, p: Point) -> bool {
    region_edges(region).any(|(a, b)| segment_distance(p, a, b) < 1e-4) || encloses(region_edges(region), p)
}

fn inside(region: &Region, a: Point, b: Point) -> bool {
    !region_edges(region).any(|(c, d)| crosses(a, b, c, d))
        && covers(region, [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0])
}

/// The part of a layer where a move needs no retraction: inside its outer wall (inside its
/// areas, with none), covered by the layer above, and not hanging over air.
pub fn hidden(layer: &RegionLayer) -> Result<Region, GeomError> {
    let outline: Region = layer.outline().into_iter().cloned().collect();
    difference(&intersection(&outline, &layer.above)?, &layer.bridge)
}

/// Marks the beads of a layer, in printing order, reached from the bead before by a move over
/// `hidden`.
pub fn hide(mut paths: Vec<Path>, hidden: &Region) -> Vec<Path> {
    let mut end: Option<Point> = None;
    for path in &mut paths {
        if let (Some(a), Some(&b)) = (end, path.points.first())
            && !path.travel_inside
        {
            path.travel_inside = inside(hidden, a, b);
        }
        end = path.end();
    }
    paths
}

/// Marks the lines of `paths` (in printing order, all of `fill`'s role) reached from the line
/// before by a move inside the area.
pub fn link(mut paths: Vec<Path>, fill: &Fill) -> Vec<Path> {
    let mut end: Option<Point> = None;
    for path in &mut paths {
        if let (Some(a), Some(&b)) = (end, path.points.first()) {
            let gap = (b[0] - a[0]).hypot(b[1] - a[1]);
            path.travel_inside = gap <= fill.max_link && (gap < EPS || inside(&fill.reach, a, b));
        }
        end = path.end();
    }
    paths
}
