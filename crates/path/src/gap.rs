//! The strips of an area too thin for its lines, each filled by one bead along its centre line
//! as wide as the strip is thick.
//!
//! A strip `t` thick holds one bead whose spacing is `t` (`bead_spacing`): `w = t + h·(1 − π/4)`
//! wide on a layer `h` high, which fills it exactly, side to side and along, where lines across
//! it would be short, all ends. The strips of an area are what an opening by its line spacing
//! `s` takes away: the parts narrower than two lines. Between its sides the bead holds the
//! strip's plastic however thin it is; only a strip thinner than the printer's resolution is
//! left. A bead keeps one width while the strip's thickness changes by less than the
//! resolution, and one role while it stays in one area: the moves stay few, and the plastic is
//! the strip's to within what the printer can tell apart.

use crate::{Path, Role};
use layercraft_geom::{GeomError, Point, Region, chordal_axis, difference, opening};
use std::f64::consts::PI;

/// The strips of `region` narrower than two lines `s` apart, and the rest of it.
pub fn strips(region: &Region, s: f64) -> Result<(Region, Region), GeomError> {
    let wide = opening(region, s)?;
    Ok((difference(region, &wide)?, wide))
}

/// The beads filling the strips `thin`, on a layer `h` high, at most two lines `s` apart thick,
/// each of one width to within `resolution` and of the role `role_at` gives along it.
pub fn gap_fill(
    thin: &Region,
    role_at: impl Fn(Point) -> Role,
    s: f64,
    h: f64,
    resolution: f64,
) -> Result<Vec<Path>, GeomError> {
    let mut out = Vec::new();
    // Sides sampled closer than the narrowest bead of rounded sides, a quarter turn of the
    // layer's height across: a thinner strip's line still runs between its sides.
    for line in chordal_axis(thin, PI * h / 4.0, resolution)? {
        let mut points = line.points.clone();
        let mut thickness = line.thickness.clone();
        if line.closed {
            points.extend(line.points.first().copied());
            thickness.extend(line.thickness.first().copied());
        }
        // Each segment as thick as the strip at its middle; the printable runs of them.
        let mut run: Vec<Segment> = Vec::new();
        let segments = points.windows(2).zip(thickness.windows(2));
        for (p, t) in segments {
            let (Some(&a), Some(&b), Some(&ta), Some(&tb)) = (p.first(), p.get(1), t.first(), t.get(1))
            else {
                continue;
            };
            let t = ((ta + tb) / 2.0).min(2.0 * s);
            if t > resolution {
                let role = role_at([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]);
                run.push(Segment { from: a, to: b, width: t + h * (1.0 - PI / 4.0), role });
            } else {
                beads(&std::mem::take(&mut run), resolution, &mut out);
            }
        }
        let whole = line.closed && run.len() + 1 == points.len();
        let before = out.len();
        beads(&run, resolution, &mut out);
        // A loop laid as one bead closes.
        if whole
            && out.len() == before + 1
            && let Some(bead) = out.last_mut()
        {
            bead.points.pop();
            bead.closed = true;
        }
    }
    Ok(out)
}

/// A stretch of a centre line and the bead it takes.
struct Segment {
    from: Point,
    to: Point,
    width: f64,
    role: Role,
}

/// A run of segments as beads, each of one role and of one width to within `resolution`: the
/// mean of its segments' by length, which holds their plastic.
fn beads(run: &[Segment], resolution: f64, out: &mut Vec<Path>) {
    let mut rest = run;
    while let Some(first) = rest.first() {
        let (mut lo, mut hi) = (first.width, first.width);
        let count = rest
            .iter()
            .take_while(|s| {
                (lo, hi) = (lo.min(s.width), hi.max(s.width));
                s.role == first.role && hi - lo <= resolution
            })
            .count()
            .max(1);
        let (piece, next) = rest.split_at(count.min(rest.len()));
        rest = next;
        let length = |s: &Segment| (s.to[0] - s.from[0]).hypot(s.to[1] - s.from[1]);
        let total: f64 = piece.iter().map(length).sum();
        if total <= 0.0 {
            continue;
        }
        let width = piece.iter().map(|s| length(s) * s.width).sum::<f64>() / total;
        let points = std::iter::once(first.from).chain(piece.iter().map(|s| s.to)).collect();
        out.push(Path::new(first.role, width, false, points));
    }
}
