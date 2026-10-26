//! Holding each bead to what the hot end can melt, over any program: a move that pushes `e`
//! mm of filament of section `A` over a path `L` long at speed `v` asks `e·A·v/L` mm³/s, and
//! runs no faster than the melt limit allows. Compiled beads are held to it already; G-code
//! from another slicer, which reckons its own limit, may not be.

use crate::Program;
use crate::moves::chained;

/// Slows every bead of `program` that would ask more than `max_volumetric_speed` mm³/s of
/// filament `filament_diameter` wide; how many it slowed.
pub fn melt(program: &mut Program, max_volumetric_speed: f64, filament_diameter: f64) -> usize {
    let area = std::f64::consts::PI * (filament_diameter / 2.0).powi(2);
    let mut slowed = 0;
    let mut at = None;
    for layer in &mut program.layers {
        let lengths: Vec<f64> =
            chained(&layer.moves, at).map(|(from, m)| from.map_or(0.0, |f| m.length(f))).collect();
        at = layer.moves.iter().rev().find_map(|m| m.to).or(at);
        for (m, length) in layer.moves.iter_mut().zip(lengths) {
            if m.e <= 0.0 || length <= 0.0 {
                continue;
            }
            let fastest = max_volumetric_speed * length / (m.e * area);
            // A bead held to the limit already, to rounding, is left as it is.
            if m.speed > fastest * (1.0 + 1e-9) {
                m.speed = fastest;
                slowed += 1;
            }
        }
    }
    slowed
}
