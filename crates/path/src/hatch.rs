//! Parallel lines on a fixed grid, clipped to a region.

use layercraft_geom::{GeomError, Polyline, Region, bounds, clip_lines};

/// The most hatch lines drawn across one region.
const MAX_LINES: f64 = 1_000_000.0;

/// Lines at `angle` degrees, `spacing` apart on the grid `v = k · spacing` of the rotated frame,
/// clipped to `region`.
pub fn hatch(region: &Region, spacing: f64, angle: f64) -> Result<Vec<Polyline>, GeomError> {
    let Some([lo, hi]) = bounds(region) else { return Ok(Vec::new()) };
    if !(spacing.is_finite() && spacing > 0.0) {
        return Ok(Vec::new());
    }
    let (s, c) = angle.to_radians().sin_cos();
    let corners = [[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]];
    let along = |p: &[f64; 2]| p[0] * c + p[1] * s;
    let across = |p: &[f64; 2]| -p[0] * s + p[1] * c;
    let (u0, u1) = corners.iter().map(along).fold((f64::MAX, f64::MIN), |(a, b), u| (a.min(u), b.max(u)));
    let (v0, v1) = corners.iter().map(across).fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v), b.max(v)));
    let (k0, k1) = ((v0 / spacing).ceil(), (v1 / spacing).floor());
    if !(k0.is_finite() && k1.is_finite()) || k1 - k0 > MAX_LINES {
        return Ok(Vec::new());
    }
    let to_xy = |u: f64, v: f64| [u * c - v * s, u * s + v * c];
    let mut lines = Vec::new();
    let mut k = k0;
    while k <= k1 {
        let v = k * spacing;
        lines.push(vec![to_xy(u0 - 1.0, v), to_xy(u1 + 1.0, v)]);
        k += 1.0;
    }
    clip_lines(&lines, region)
}
