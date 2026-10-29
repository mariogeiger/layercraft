//! Holding each move to what the machine can do, over any program: compiled, read from another
//! slicer's G-code, merged or edited. A move that pushes `e` mm of filament of section `A` over
//! a path `L` long at speed `v` asks `e·A·v/L` mm³/s of the hot end, and runs no faster than its
//! melt limit allows; one that climbs or falls `Δz` over it drives the Z axis at `v·|Δz|/L`, and
//! runs no faster than that axis allows. Held once the beads are written as they go (`fit`):
//! on a bent layer the slab, and so the plastic, changes along a bead, and the bead climbs.

use crate::Program;
use crate::moves::chained;

/// The machine's limits a move is held to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveLimits {
    /// The hot end's melt limit, in mm³/s.
    pub max_volumetric_speed: f64,
    pub filament_diameter: f64,
    /// The fastest the Z axis moves, in mm/s.
    pub max_speed_z: f64,
}

/// Slows every move of `program` that asks more than `limits` allow; how many beads it slowed
/// to the melt limit.
pub fn limit(program: &mut Program, limits: &MoveLimits) -> usize {
    let area = crate::disc_area(limits.filament_diameter);
    let mut slowed = 0;
    let mut at = None;
    for layer in &mut program.layers {
        let start = at;
        at = layer.moves.iter().rev().find_map(|m| m.to).or(at);
        let held: Vec<(f64, f64)> = chained(&layer.moves, start)
            .map(|(from, m)| {
                let (length, dz) = match (from, m.to) {
                    (Some(from), Some(to)) => (m.length(from), (to[2] - from[2]).abs()),
                    _ => (0.0, 0.0),
                };
                let melt = if m.e > 0.0 && length > 0.0 {
                    limits.max_volumetric_speed * length / (m.e * area)
                } else {
                    f64::INFINITY
                };
                let climb = if dz > 0.0 { limits.max_speed_z * length / dz } else { f64::INFINITY };
                (melt, climb)
            })
            .collect();
        for (m, (melt, climb)) in layer.moves.iter_mut().zip(held) {
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
