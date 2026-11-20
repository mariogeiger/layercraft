//! What a program does, read from its moves, for people and for comparisons: its layers and its
//! height, the filament each material pushes, the plastic each layer lays (`e` times its
//! filament's section), the time the planner gives it (`estimate`), and where its first layer
//! lays plastic. Nothing is stored: a program merged or edited by hand summarizes the same way.

use crate::moves::chained;
use crate::{Filament, Pass, Program, disc_area};
use serde::{Deserialize, Serialize};

/// What the program does.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Summary {
    pub layers: usize,
    pub max_z: f64,
    /// Filament pushed, in mm.
    pub filament_mm: f64,
    /// Filament pushed of each material, in mm, by material.
    #[serde(default)]
    pub filament_mm_by_material: Vec<f64>,
    /// Plastic pushed per layer, all its passes, in mm³.
    pub volume_per_layer: Vec<f64>,
    /// Time of the moves, in seconds (the machine's own start and end sequences not counted).
    pub motion_seconds: f64,
    /// Time of the moves up to the end of the first layer, in seconds.
    pub first_layer_seconds: f64,
    /// The first layer's extent `[min, max]`, for bed levelling.
    pub first_layer_bounds: [[f64; 2]; 2],
}

/// Grows the box `b` to hold the box `c` widened by `r` on every side.
fn grow(b: [[f64; 2]; 2], c: [[f64; 2]; 2], r: f64) -> [[f64; 2]; 2] {
    [
        [b[0][0].min(c[0][0] - r), b[0][1].min(c[0][1] - r)],
        [b[1][0].max(c[1][0] + r), b[1][1].max(c[1][1] + r)],
    ]
}

/// Where the first layer lays plastic, all its passes: its beads' paths grown by their half
/// widths; nowhere, the origin.
pub(crate) fn first_layer_bounds(program: &Program) -> [[f64; 2]; 2] {
    let mut b = [[f64::MAX, f64::MAX], [f64::MIN, f64::MIN]];
    for pass in program.layers().next().into_iter().flatten() {
        for (from, m) in chained(&pass.moves, None) {
            let (Some(from), Some(bead)) = (from, m.bead) else { continue };
            b = grow(b, m.bounds(from), bead.width / 2.0);
        }
    }
    if b[0][0] > b[1][0] { [[0.0, 0.0], [0.0, 0.0]] } else { b }
}

/// The filament a pass's beads push, in mm.
fn pushed(pass: &Pass) -> f64 {
    pass.moves.iter().filter(|m| m.bead.is_some()).map(|m| m.e).sum()
}

/// What `program` does, its moves timed at `times` (`estimate`), each material pushing its
/// `filaments`' own, the last given standing for those past them.
pub fn summarize(program: &Program, times: &[f64], filaments: &[Filament]) -> Summary {
    let area =
        |material: usize| filaments.get(material).or(filaments.last()).map_or(0.0, |f| disc_area(f.diameter));
    let total = times.last().copied().unwrap_or(0.0);
    let mut layers = program.layers();
    // The first layer is all the passes before the second begins.
    let first_moves: usize = layers.next().into_iter().flatten().map(|p| p.moves.len()).sum();
    let first_layer_seconds = if layers.next().is_some() {
        first_moves.checked_sub(1).and_then(|i| times.get(i)).copied().unwrap_or(0.0)
    } else {
        total
    };
    let mut by_material = vec![0.0; program.materials()];
    for pass in &program.passes {
        if let Some(slot) = by_material.get_mut(pass.material) {
            *slot += pushed(pass);
        }
    }
    let volume_per_layer: Vec<f64> =
        program.layers().map(|layer| layer.iter().map(|p| pushed(p) * area(p.material)).sum()).collect();
    Summary {
        layers: volume_per_layer.len(),
        max_z: program.highest(),
        filament_mm: by_material.iter().sum(),
        filament_mm_by_material: by_material,
        volume_per_layer,
        motion_seconds: total,
        first_layer_seconds,
        first_layer_bounds: first_layer_bounds(program),
    }
}
