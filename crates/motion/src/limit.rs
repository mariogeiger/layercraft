//! Holding each move to what the machine can do, over any program: compiled, read from another
//! slicer's G-code, merged or edited. A move that pushes `e` mm of filament of section `A` over
//! a path `L` long at speed `v` asks `e·A·v/L` mm³/s of the hot end, and runs no faster than its
//! filament's melt limit allows (`Filament`, each pass its material's); one that climbs or falls
//! `Δz` over it drives the Z axis at `v·|Δz|/L`, and runs no faster than that axis allows. Held
//! once the beads are written as they go (`fit`): on a bent layer the slab, and so the plastic,
//! changes along a bead, and the bead climbs.

use crate::moves::{chained, end};
use crate::{Filament, Program, disc_area};

/// Slows every move of `program` that asks more than its material's filament (`filaments`, the
/// last given standing for those past them) or the Z axis's `max_speed_z` allow; how many moves
/// it slowed to the melt limit.
pub fn limit(program: &mut Program, filaments: &[Filament], max_speed_z: f64) -> usize {
    let Some(last) = filaments.last() else { return 0 };
    let mut slowed = 0;
    let mut at = None;
    for pass in &mut program.passes {
        let filament = filaments.get(pass.material).unwrap_or(last);
        let area = disc_area(filament.diameter);
        let start = at;
        at = end(&pass.moves, at);
        let held: Vec<(f64, f64)> = chained(&pass.moves, start)
            .map(|(from, m)| {
                let (length, dz) = match (from, m.to) {
                    (Some(from), Some(to)) => (m.length(from), (to[2] - from[2]).abs()),
                    _ => (0.0, 0.0),
                };
                let melt = if m.e > 0.0 && length > 0.0 {
                    filament.max_volumetric_speed * length / (m.e * area)
                } else {
                    f64::INFINITY
                };
                let climb = if dz > 0.0 { max_speed_z * length / dz } else { f64::INFINITY };
                (melt, climb)
            })
            .collect();
        for (m, (melt, climb)) in pass.moves.iter_mut().zip(held) {
            // A move held to a limit already, to rounding, is left as it is.
            let fastest = melt.min(climb);
            if m.speed > fastest * (1.0 + 1e-9) {
                m.speed = fastest;
                slowed += usize::from(melt <= climb);
            }
        }
    }
    slowed
}
