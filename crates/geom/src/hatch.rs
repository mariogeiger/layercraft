//! Parallel lines clipped to a region, on a grid whose phase the caller anchors; and the strands
//! across a gap, as both the region and the path stage lay them (`bridge_strands`): in the
//! gap's own frame, where their ends rest (`Rests`).

use crate::{
    Bands, Frame, GeomError, Point, Polyline, Region, bounds, clip_lines, disc_offset, edges, polyline_length,
};

/// The most hatch lines drawn across one region.
const MAX_LINES: f64 = 1_000_000.0;

/// Lines at `angle` degrees, `spacing` apart on the grid `v = phase + k · spacing` of the rotated
/// frame, `v` the distance across them from the origin, clipped to `region`.
pub fn hatch(region: &Region, spacing: f64, angle: f64, phase: f64) -> Result<Vec<Polyline>, GeomError> {
    let Some([lo, hi]) = bounds(region) else { return Ok(Vec::new()) };
    if !(spacing.is_finite() && spacing > 0.0 && phase.is_finite()) {
        return Ok(Vec::new());
    }
    let (s, c) = angle.to_radians().sin_cos();
    let corners = [[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]];
    let along = |p: &[f64; 2]| p[0] * c + p[1] * s;
    let across = |p: &[f64; 2]| -p[0] * s + p[1] * c;
    let (u0, u1) = corners.iter().map(along).fold((f64::MAX, f64::MIN), |(a, b), u| (a.min(u), b.max(u)));
    let (v0, v1) = corners.iter().map(across).fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v), b.max(v)));
    let (k0, k1) = (((v0 - phase) / spacing).ceil(), ((v1 - phase) / spacing).floor());
    if !(k0.is_finite() && k1.is_finite()) || k1 - k0 > MAX_LINES {
        return Ok(Vec::new());
    }
    let to_xy = |u: f64, v: f64| [u * c - v * s, u * s + v * c];
    let mut lines = Vec::new();
    let mut k = k0;
    while k <= k1 {
        let v = phase + k * spacing;
        lines.push(vec![to_xy(u0 - 1.0, v), to_xy(u1 + 1.0, v)]);
        k += 1.0;
    }
    clip_lines(&lines, region)
}

/// The phase of the grid of `hatch` at `angle` degrees that has a line through `point`.
pub fn phase_through(angle: f64, point: Point) -> f64 {
    let (s, c) = angle.to_radians().sin_cos();
    -point[0] * s + point[1] * c
}

/// The most candidate directions tried for one bridge.
const MAX_CANDIDATES: usize = 72;

/// The step, in degrees, of the fan of directions tried round `preferred`, a half turn in all.
const FAN_STEP_DEG: f64 = 15.0;

/// Directions closer than this, in degrees, are one.
const SAME_ANGLE_DEG: f64 = 0.5;

/// Strands closer in length than this, in mm, are as long.
const SAME_LENGTH_MM: f64 = 1e-6;

/// How far apart two directions of lines are, in degrees: within a quarter turn.
fn apart(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(180.0);
    d.min(180.0 - d)
}

/// The strands of one direction tried for a bridge, and what they are judged by.
struct Strands {
    /// Their ends over air.
    loose: usize,
    /// The length of the longest, the weakest.
    longest: f64,
    /// How far, in degrees, they turn from the frame's first axis.
    off: f64,
    angle: f64,
    lines: Vec<Polyline>,
}

impl Strands {
    /// Whether they are better than `other`: fewer ends loose, then a shorter longest strand,
    /// then nearer the frame's first axis.
    fn beats(&self, other: &Strands) -> bool {
        let as_long = (self.longest - other.longest).abs() <= SAME_LENGTH_MM;
        self.loose < other.loose
            || (self.loose == other.loose && self.longest < other.longest - SAME_LENGTH_MM)
            || (self.loose == other.loose && as_long && self.off < other.off)
    }
}

/// Where the ends of strands `width` wide rest: on the section below, or within half their width
/// past its edge, where a bead's centre line may lie and the bead still stand.
pub struct Rests {
    on: Bands,
}

impl Rests {
    /// Where strands `width` wide rest on the section `below`; nowhere for no section.
    pub fn on(below: &Region, width: f64) -> Result<Rests, GeomError> {
        Ok(Rests { on: Bands::new(&disc_offset(below, width / 2.0)?) })
    }

    /// Whether an end at `p` hangs: rests on nothing.
    pub fn hangs(&self, p: Point) -> bool {
        !self.on.covers(p)
    }
}

/// The strands across the gap `region`, `spacing` apart, and their direction in degrees, in the
/// gap's own `frame`. A strand hangs from its two ends, so it must rest on the layer below at both:
/// the direction chosen leaves the fewest ends over air (`rests`), and among those its longest
/// strand, the weakest, is shortest. Both optima lie across an edge of the region, so the
/// directions tried are those perpendicular to its edges, plus a fan every `FAN_STEP_DEG` from
/// the frame's first axis; of directions as good, the one nearest that axis wins. Every direction
/// is tried on the grid of `hatch` through the frame's origin, where its strands lie, so the
/// choice turns, moves and mirrors with the gap.
pub fn bridge_strands(
    region: &Region,
    spacing: f64,
    frame: &Frame,
    rests: &Rests,
) -> Result<(f64, Vec<Polyline>), GeomError> {
    let (preferred, anchor) = (frame.angle(), frame.origin());
    let across = region.iter().flatten().flat_map(|c| edges(c));
    let normals = across.map(|(a, b)| (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees() + 90.0);
    let fan = (0..(180.0 / FAN_STEP_DEG) as usize).map(|k| preferred + FAN_STEP_DEG * k as f64);
    let mut seen: Vec<f64> = Vec::new();
    for a in std::iter::once(preferred).chain(normals).chain(fan).map(|a| a.rem_euclid(180.0)) {
        if seen.len() < MAX_CANDIDATES && !seen.iter().any(|&s| apart(s, a) < SAME_ANGLE_DEG) {
            seen.push(a);
        }
    }
    let mut best: Option<Strands> = None;
    for angle in seen {
        let lines = hatch(region, spacing, angle, phase_through(angle, anchor))?;
        let ends = lines.iter().flat_map(|line| [line.first(), line.last()]).flatten();
        let loose = ends.filter(|&&p| rests.hangs(p)).count();
        let longest = lines.iter().map(|line| polyline_length(line)).fold(0.0, f64::max);
        let here = Strands { loose, longest, off: apart(angle, preferred), angle, lines };
        if best.as_ref().is_none_or(|b| here.beats(b)) {
            best = Some(here);
        }
    }
    Ok(best.map_or_else(|| (preferred.rem_euclid(180.0), Vec::new()), |b| (b.angle, b.lines)))
}
