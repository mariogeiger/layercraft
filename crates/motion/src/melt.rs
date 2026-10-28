//! Holding each bead to what the hot end can melt, over any program: a move that pushes `e`
//! mm of filament of section `A` over a path `L` long at speed `v` asks `e·A·v/L` mm³/s, and
//! runs no faster than the melt limit allows. Compiled beads are held to it already; G-code
//! from another slicer, which reckons its own limit, may not be.

use crate::Program;
use crate::moves::lengths;

/// Slows every bead of `program` that would ask more than `max_volumetric_speed` mm³/s of
/// filament `filament_diameter` wide; how many it slowed.
pub fn melt(program: &mut Program, max_volumetric_speed: f64, filament_diameter: f64) -> usize {
    let area = crate::disc_area(filament_diameter);
    let mut slowed = 0;
    let mut at = None;
    for layer in &mut program.layers {
        let lengths = lengths(&layer.moves, &mut at);
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
