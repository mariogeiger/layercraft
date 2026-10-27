//! The quality of beads laid in a piece of a layer, from their geometry.
//!
//! A bead `w` wide on a layer `h` high fills, tiled with its neighbours, a band its spacing `s`
//! wide (`bead_spacing`): its section `s·h`. Its ends are round, as the plastic spreads about
//! the nozzle where it stops; holding the plastic of a line of length `L`, `s·h·L`, its
//! footprint is a stadium whose caps are centred `π·s/8` inside its ends: `s·(L − 2·π·s/8) +
//! π·s²/4 = s·L`. Laid in a piece `A`, beads of footprints `Bᵢ` leave a void `|A \ ∪Bᵢ|` and
//! push plastic where there is no room for it, `Σ|Bᵢ| − |A ∩ ∪Bᵢ|`: both, times `h`, are the
//! interior's error. The plastic is `Σ|Bᵢ|·h`.

use layercraft_cost::{Cost, Quanta};
use layercraft_geom::{Contour, GeomError, Point, Polyline, Region, area, intersection, union};

/// The footprint of a bead along `a`–`b`, `s` wide, round at its ends, its caps' chords within
/// `tolerance` of their arcs and holding their area.
fn stadium(a: Point, b: Point, s: f64, tolerance: f64) -> Option<Contour> {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l = d[0].hypot(d[1]);
    let r = s / 2.0;
    let inset = std::f64::consts::PI * s / 8.0;
    if l <= 2.0 * inset || r <= 0.0 {
        return None;
    }
    let u = [d[0] / l, d[1] / l];
    let (p, q) = ([a[0] + inset * u[0], a[1] + inset * u[1]], [b[0] - inset * u[0], b[1] - inset * u[1]]);
    // Chords of a half turn, each within `tolerance` of its arc.
    let step = 2.0 * (1.0 - (tolerance / r).min(1.0)).acos();
    let n = ((std::f64::consts::PI / step.max(1e-3)).ceil() as usize).clamp(2, 64);
    let heading = u[1].atan2(u[0]);
    // Vertices a little outside the arc, so each fan of chords holds the area of its sector.
    let theta = std::f64::consts::PI / n as f64;
    let r = r * (theta / theta.sin()).sqrt();
    let mut contour = Vec::with_capacity(2 * n + 2);
    for (centre, start) in
        [(q, heading - std::f64::consts::FRAC_PI_2), (p, heading + std::f64::consts::FRAC_PI_2)]
    {
        for k in 0..=n {
            let t = start + std::f64::consts::PI * k as f64 / n as f64;
            contour.push([centre[0] + r * t.cos(), centre[1] + r * t.sin()]);
        }
    }
    Some(contour)
}

/// The cost of laying `lines`, beads at spacing `s` on a layer `h` high, in the piece `piece`:
/// the void and excess they leave, the plastic they push. The time is the motion stage's.
pub fn lines_cost(
    piece: &Region,
    lines: &[Polyline],
    s: f64,
    h: f64,
    quanta: &Quanta,
) -> Result<Cost, GeomError> {
    let mut footprints: Region = Vec::new();
    let mut plastic = 0.0;
    for line in lines {
        for w in line.windows(2) {
            let (Some(&a), Some(&b)) = (w.first(), w.get(1)) else { continue };
            plastic += s * (b[0] - a[0]).hypot(b[1] - a[1]);
            if let Some(c) = stadium(a, b, s, quanta.length) {
                footprints.push(vec![c]);
            }
        }
    }
    let laid = union(&footprints, &Vec::new())?;
    let held = area(&intersection(&laid, piece)?);
    let void = (area(piece) - held).max(0.0);
    let excess = (plastic - held).max(0.0);
    Ok(quanta.cost(0.0, (void + excess) * h, plastic * h, 0.0))
}
