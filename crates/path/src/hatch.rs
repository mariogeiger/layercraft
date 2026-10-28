//! Parallel lines on a fixed grid, clipped to a region.

use layercraft_geom::{GeomError, Point, Polyline, Region, bounds, clip_lines, edges, polyline_length};

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

/// The most candidate directions tried for one bridge.
const MAX_CANDIDATES: usize = 72;

/// The direction, in degrees, for strands across `region`, `spacing` apart. A strand hangs from
/// its two ends, so it must rest on the layer below at both: the direction chosen leaves the
/// fewest ends over air (`hangs`), and among those its longest strand, the weakest, is shortest.
/// Both optima lie across an edge of the region, so the directions tried are those perpendicular
/// to its edges, plus every 15°; ties keep `preferred`.
pub fn bridge_angle(
    region: &Region,
    spacing: f64,
    preferred: f64,
    hangs: impl Fn(Point) -> bool,
) -> Result<f64, GeomError> {
    let across = region.iter().flatten().flat_map(|c| edges(c));
    let candidates = std::iter::once(preferred)
        .chain(across.map(|(a, b)| (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees() + 90.0));
    let normalize = |a: f64| a.rem_euclid(180.0);
    let mut seen: Vec<f64> = Vec::new();
    for a in candidates.map(normalize).chain((0..12).map(|k| 15.0 * k as f64)) {
        if seen.len() < MAX_CANDIDATES && !seen.iter().any(|s| (s - a).abs() < 0.5) {
            seen.push(a);
        }
    }
    let mut best = ((usize::MAX, f64::INFINITY), normalize(preferred));
    for angle in seen {
        let lines = hatch(region, spacing, angle)?;
        let loose = lines
            .iter()
            .flat_map(|line| [line.first(), line.last()])
            .flatten()
            .filter(|&&p| hangs(p))
            .count();
        let longest = lines.iter().map(|line| polyline_length(line)).fold(0.0, f64::max);
        let ((best_loose, best_longest), _) = best;
        if loose < best_loose || (loose == best_loose && longest < best_longest - 1e-6) {
            best = ((loose, longest), angle);
        }
    }
    Ok(best.1)
}
