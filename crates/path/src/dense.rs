//! Each piece of a dense area hatched at the angle the objective prefers.
//!
//! A piece may be hatched at the layer's angle, across it, or along any of its longest edges,
//! where lines end least often. Each way is costed by the plastic it leaves out or pushes where
//! there is no room (`layercraft_choose::quality`), then by its plastic; the e-graph takes,
//! piece by piece, the cheapest (`layercraft_choose`).

use crate::PathError;
use crate::hatch::hatch;
use layercraft_choose::{choose_fills, quality::lines_cost};
use layercraft_cost::{Cost, Quanta};
use layercraft_geom::{Polyline, Region};

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
    let candidates = [preferred, preferred + 90.0].into_iter().chain(edges.iter().map(|e| e.1));
    for a in candidates.map(|a| a.rem_euclid(180.0)) {
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

/// The lines filling `reach`, `spacing` apart on a layer `height` high, each of its pieces at
/// its best angle, by the quanta of a printer of resolution `resolution`.
pub fn best_hatch(
    reach: &Region,
    spacing: f64,
    height: f64,
    preferred: f64,
    resolution: f64,
) -> Result<Vec<Polyline>, PathError> {
    let quanta = Quanta::new(resolution);
    let pieces: Vec<Region> = reach.iter().map(|shape| vec![shape.clone()]).collect();
    let tried: Vec<Vec<f64>> = pieces.iter().map(|p| angles(p, preferred)).collect();
    let hatched = |p: usize, a: usize| -> Result<Vec<Polyline>, PathError> {
        let (Some(piece), Some(angle)) = (pieces.get(p), tried.get(p).and_then(|t| t.get(a))) else {
            return Ok(Vec::new());
        };
        Ok(hatch(piece, spacing, *angle)?)
    };
    let counts: Vec<usize> = tried.iter().map(Vec::len).collect();
    let cost = |p: usize, a: usize| {
        let (Ok(lines), Some(piece)) = (hatched(p, a), pieces.get(p)) else { return Cost::INFINITE };
        lines_cost(piece, &lines, spacing, height, &quanta).unwrap_or(Cost::INFINITE)
    };
    let (chosen, _) = choose_fills(&counts, cost).map_err(|e| PathError::BadParams(e.to_string()))?;
    let mut lines = Vec::new();
    for (p, a) in chosen.into_iter().enumerate() {
        lines.extend(hatched(p, a)?);
    }
    Ok(lines)
}
