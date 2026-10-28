//! Each piece of a dense area hatched at the angle that fills it best.
//!
//! A piece may be hatched at the layer's angle, across it, or along any of its longest edges,
//! where lines end least often. A bead `s` apart on a layer `h` high holds the plastic `s·h·L`
//! of its line of length `L`; its ends are round, as the plastic spreads about the nozzle where
//! it stops, so its footprint is a stadium whose caps are centred `π·s/8` inside its ends:
//! `s·(L − 2·π·s/8) + π·s²/4 = s·L`. Beads of footprints `Bᵢ` laid in a piece `A` leave the
//! void `|A \ ∪Bᵢ|` and push plastic where there is no room for it, `Σ|Bᵢ| − |A ∩ ∪Bᵢ|`: both,
//! times `h`, are the interior's error. Each way is ranked by that error, then by its plastic,
//! each counted in whole quanta `ε³` of the printer's resolution `ε`: what it cannot tell apart
//! ties, and the earlier way wins. Both sum over the pieces, so the best layer is each piece's
//! best way.

use crate::PathError;
use crate::hatch::hatch;
use layercraft_geom::{Contour, GeomError, Point, Polyline, Region, area, intersection, union};
use std::f64::consts::PI;

/// The longest edges whose directions a piece is tried along.
const EDGES_TRIED: usize = 4;

/// The angles, in degrees, a piece may be hatched at: `preferred`, across it, and along its
/// longest edges, each once.
fn angles(piece: &Region, preferred: f64) -> Vec<f64> {
    let mut edges: Vec<(f64, f64)> = piece
        .iter()
        .flatten()
        .flat_map(|c| {
            let n = c.len();
            (0..n).filter_map(move |i| {
                let (a, b) = (c.get(i)?, c.get((i + 1) % n)?);
                Some(((b[0] - a[0]).hypot(b[1] - a[1]), (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees()))
            })
        })
        .collect();
    edges.sort_by(|x, y| y.0.total_cmp(&x.0));
    let mut out: Vec<f64> = Vec::new();
    for a in [preferred, preferred + 90.0]
        .into_iter()
        .chain(edges.iter().map(|e| e.1))
        .map(|a| a.rem_euclid(180.0))
    {
        let close = |s: &f64| ((s - a).rem_euclid(180.0)).min((a - s).rem_euclid(180.0)) < 0.5;
        if !out.iter().any(close) {
            out.push(a);
        }
        if out.len() >= 2 + EDGES_TRIED {
            break;
        }
    }
    out
}

/// The footprint of a bead along `a`–`b`, `s` wide, round at its ends, its caps' chords within
/// `tolerance` of their arcs and holding their area.
fn stadium(a: Point, b: Point, s: f64, tolerance: f64) -> Option<Contour> {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l = d[0].hypot(d[1]);
    let r = s / 2.0;
    let inset = PI * s / 8.0;
    if l <= 2.0 * inset || r <= 0.0 {
        return None;
    }
    let u = [d[0] / l, d[1] / l];
    let (p, q) = ([a[0] + inset * u[0], a[1] + inset * u[1]], [b[0] - inset * u[0], b[1] - inset * u[1]]);
    // Chords of a half turn, each within `tolerance` of its arc.
    let step = 2.0 * (1.0 - (tolerance / r).min(1.0)).acos();
    let n = ((PI / step.max(1e-3)).ceil() as usize).clamp(2, 64);
    let heading = u[1].atan2(u[0]);
    // Vertices a little outside the arc, so each fan of chords holds the area of its sector.
    let theta = PI / n as f64;
    let r = r * (theta / theta.sin()).sqrt();
    let mut contour = Vec::with_capacity(2 * n + 2);
    for (centre, start) in [(q, heading - PI / 2.0), (p, heading + PI / 2.0)] {
        for k in 0..=n {
            let t = start + PI * k as f64 / n as f64;
            contour.push([centre[0] + r * t.cos(), centre[1] + r * t.sin()]);
        }
    }
    Some(contour)
}

/// The rank of `lines`, beads `s` apart on a layer `h` high, laid in `piece`: its interior error
/// and its plastic, in whole `ε³`.
pub(crate) fn rank(
    piece: &Region,
    lines: &[Polyline],
    s: f64,
    h: f64,
    eps: f64,
) -> Result<(u64, u64), GeomError> {
    let mut footprints: Region = Vec::new();
    let mut plastic = 0.0;
    for w in lines.iter().flat_map(|line| line.windows(2)) {
        let (Some(&a), Some(&b)) = (w.first(), w.get(1)) else { continue };
        plastic += s * (b[0] - a[0]).hypot(b[1] - a[1]);
        footprints.extend(stadium(a, b, s, eps).map(|c| vec![c]));
    }
    let held = area(&intersection(&union(&footprints, &Vec::new())?, piece)?);
    let void = (area(piece) - held).max(0.0);
    let excess = (plastic - held).max(0.0);
    let quanta = |v: f64| (v * h / eps.powi(3)).round().clamp(0.0, u64::MAX as f64) as u64;
    Ok((quanta(void + excess), quanta(plastic)))
}

/// The lines filling `reach`, `spacing` apart on a layer `height` high, each of its pieces at
/// its best angle, for a printer of resolution `resolution`.
pub fn best_hatch(
    reach: &Region,
    spacing: f64,
    height: f64,
    preferred: f64,
    resolution: f64,
) -> Result<Vec<Polyline>, PathError> {
    let mut lines = Vec::new();
    for shape in reach {
        let piece = vec![shape.clone()];
        let mut best: Option<((u64, u64), Vec<Polyline>)> = None;
        for angle in angles(&piece, preferred) {
            let way = hatch(&piece, spacing, angle)?;
            let r = rank(&piece, &way, spacing, height, resolution)?;
            if best.as_ref().is_none_or(|(b, _)| r < *b) {
                best = Some((r, way));
            }
        }
        lines.extend(best.map(|(_, way)| way).unwrap_or_default());
    }
    Ok(lines)
}
