//! Slowing a layer down so it lasts long enough to cool, over any program: compiled, read from
//! another slicer's G-code, merged or edited.
//!
//! A layer whose beads take `busy` seconds at their speeds, less than `slow_down_layer_time`
//! `T`, has every bead slowed by `busy / T`, but never below `slow_down_min_speed` unless the
//! bead was slower already. Travels, wipes and lifts keep their speed. The layer's parameters
//! are those at its centre.

use crate::moves::chained;
use crate::plan::check;
use crate::{MotionError, MotionParams, Program};
use layercraft_tune::{Field, centre};

/// Slows each layer of `program` that would print faster than it can cool.
pub fn cool(program: &mut Program, field: &Field<MotionParams>) -> Result<(), MotionError> {
    let mut at = None;
    for layer in &mut program.layers {
        let ends: Vec<[f64; 2]> = layer
            .moves
            .iter()
            .filter(|m| m.bead.is_some())
            .filter_map(|m| m.to)
            .map(|p| [p[0], p[1]])
            .collect();
        let p = field.at(centre(&ends, layer.z), check)?;
        let lengths: Vec<f64> =
            chained(&layer.moves, at).map(|(from, m)| from.map_or(0.0, |f| m.length(f))).collect();
        if let Some(end) = layer.moves.iter().rev().find_map(|m| m.to) {
            at = Some(end);
        }
        let busy: f64 = layer
            .moves
            .iter()
            .zip(&lengths)
            .filter(|(m, _)| m.bead.is_some() && m.speed > 0.0)
            .map(|(m, l)| l / m.speed)
            .sum();
        let goal = p.slow_down_layer_time;
        if !(busy > 0.0 && busy < goal) {
            continue;
        }
        let slow = busy / goal;
        for m in layer.moves.iter_mut().filter(|m| m.bead.is_some()) {
            m.speed = (m.speed * slow).max(p.slow_down_min_speed.min(m.speed));
        }
    }
    Ok(())
}
