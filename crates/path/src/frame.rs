//! Lines laid in each shape's own frame (`Frame`), which turn, move and mirror with the solid the
//! shape belongs to.

use layercraft_geom::{Frame, GeomError, Point, Polyline, Region, Shape, bounds, hatch};

/// A point inside `shape`: the middle of the first line across it, at the widest spacing that
/// finds one.
pub(crate) fn point_inside(shape: &Shape) -> Option<Point> {
    let region = vec![shape.clone()];
    let [lo, hi] = bounds(&region)?;
    let mut spacing = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    while spacing > f64::EPSILON {
        let lines = hatch(&region, spacing, 0.0, 0.0).ok()?;
        if let Some([a, b]) = lines.iter().find_map(|l| Some([*l.first()?, *l.last()?])) {
            return Some([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]);
        }
        spacing /= 2.0;
    }
    None
}

/// Lines across `region` in each of its shapes' own frame, `frame_of` giving the frame holding
/// a point inside the shape: `Frame::hatch` of each.
pub fn hatch_in_frames(
    region: &Region,
    spacing: f64,
    turn: f64,
    frame_of: impl Fn(Point) -> Frame,
) -> Result<Vec<Polyline>, GeomError> {
    let mut lines = Vec::new();
    for shape in region {
        let frame = point_inside(shape).map(&frame_of).unwrap_or_default();
        lines.extend(frame.hatch(&vec![shape.clone()], spacing, turn)?);
    }
    Ok(lines)
}
