//! Joining consecutive infill lines into one bead along the area's edge, so the nozzle draws a
//! zigzag instead of lifting, travelling and landing between every pair of lines.
//!
//! Two lines join when the gap between the end of one and the start of the next is short (a
//! few line spacings, as between neighbouring hatch lines) and the joining segment stays inside
//! the area: it may run along its edge, never across a hole or out of a concave corner.

use crate::Path;
use crate::Role;
use layercraft_geom::{Point, Region};

/// One infill area of a layer: its role, the region its lines fill, and the longest join.
pub struct Fill {
    pub role: Role,
    pub reach: Region,
    pub max_link: f64,
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

fn distance_to_segment(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let len2 = d[0] * d[0] + d[1] * d[1];
    let t =
        if len2 > 0.0 { (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
}

fn edges(region: &Region) -> impl Iterator<Item = (Point, Point)> + '_ {
    region.iter().flatten().flat_map(|c| {
        let n = c.len();
        (0..n).filter_map(move |i| Some((*c.get(i)?, *c.get((i + 1) % n)?)))
    })
}

/// Whether a point is inside the region or on its edge (even-odd over all its contours).
fn covers(region: &Region, p: Point) -> bool {
    let mut inside = false;
    for (a, b) in edges(region) {
        if distance_to_segment(p, a, b) < 1e-4 {
            return true;
        }
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    inside
}

fn inside(region: &Region, a: Point, b: Point) -> bool {
    !edges(region).any(|(c, d)| crosses(a, b, c, d))
        && covers(region, [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0])
}

/// Joins consecutive lines of `paths` (in printing order, all of `fill`'s role) where it can.
pub fn link(paths: Vec<Path>, fill: &Fill) -> Vec<Path> {
    let mut out: Vec<Path> = Vec::with_capacity(paths.len());
    for path in paths {
        let joined = match out.last_mut() {
            Some(previous) if !previous.closed && !path.closed && previous.width == path.width => {
                match (previous.points.last().copied(), path.points.first().copied()) {
                    (Some(a), Some(b)) => {
                        let gap = (b[0] - a[0]).hypot(b[1] - a[1]);
                        if gap <= fill.max_link && (gap < EPS || inside(&fill.reach, a, b)) {
                            let skip = usize::from(gap < EPS);
                            previous.points.extend(path.points.iter().skip(skip).copied());
                            true
                        } else {
                            false
                        }
                    }
                    _ => false,
                }
            }
            _ => false,
        };
        if !joined {
            out.push(path);
        }
    }
    out
}
