//! Slowing a layer down so it lasts long enough to cool, over any program: compiled, read from
//! another slicer's G-code, merged or edited.
//!
//! A layer whose beads take less than `slow_down_layer_time` `T` at their speeds is slowed until
//! they take `T`: the hidden beads first, every one by one factor, down to `slow_down_min_speed`
//! (a bead slower already keeps its speed); then, when that is not enough, the beads that show
//! too. The surface keeps one speed from layer to layer, where a layer slowed whole would lay it
//! slower than its neighbours, and show as a band. Travels, wipes and lifts keep their speed.
//! The layer's parameters are those at its centre.

use crate::moves::chained;
use crate::plan::check;
use crate::{MotionError, MotionParams, Program};
use layercraft_path::Face;
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
        let floor = p.slow_down_min_speed;
        let shows: Vec<bool> =
            layer.moves.iter().map(|m| m.bead.is_some_and(|b| b.role.face != Face::Inside)).collect();
        // The time of the beads that show (`visible`) or not, each slowed by `s` above its floor.
        let time = |visible: bool, s: f64| -> f64 {
            layer
                .moves
                .iter()
                .zip(&lengths)
                .zip(&shows)
                .filter(|((m, _), v)| m.bead.is_some() && m.speed > 0.0 && **v == visible)
                .map(|((m, l), _)| l / slowed(m.speed, s, floor))
                .sum()
        };
        // Hidden beads first; the visible ones only once the hidden are all at their floor.
        let (hidden, visible) = if time(false, 0.0) + time(true, 1.0) >= goal {
            (factor(|s| time(false, s) + time(true, 1.0), goal), 1.0)
        } else {
            (0.0, factor(|s| time(false, 0.0) + time(true, s), goal))
        };
        for (m, &v) in layer.moves.iter_mut().zip(&shows).filter(|(m, _)| m.bead.is_some()) {
            m.speed = slowed(m.speed, if v { visible } else { hidden }, floor);
        }
    }
    Ok(())
}

/// A bead's speed `v` slowed by the factor `s`, held at `floor` unless it was slower already.
fn slowed(v: f64, s: f64, floor: f64) -> f64 {
    (v * s).max(floor.min(v))
}

/// Halvings of the factor's interval: below `f64` resolution after 60.
const HALVINGS: usize = 60;

/// The largest factor `s` in `[0, 1]` whose time `t(s)`, falling as `s` grows, reaches `goal`;
/// 0 when even that falls short.
fn factor(t: impl Fn(f64) -> f64, goal: f64) -> f64 {
    let (mut lo, mut hi) = (0.0, 1.0);
    if t(lo) < goal {
        return 0.0;
    }
    for _ in 0..HALVINGS {
        let mid = (lo + hi) / 2.0;
        if t(mid) >= goal { lo = mid } else { hi = mid }
    }
    lo
}
