//! Slowing a layer down so it lasts long enough to cool, over any program: compiled, read from
//! another slicer's G-code, merged or edited.
//!
//! How long a layer must last, `T`, comes from how the print cools (`layercraft_physics`): the
//! shortest time after which the next layer squeezes it by no more whole resolutions than it
//! would after any longer wait; or, without the physics, the profile's `slow_down_layer_time`.
//! The print's heat is followed layer by layer, each laid over the time it now takes.
//! A layer whose beads take less than `T` at their speeds is slowed until
//! they take `T`: the hidden beads first, every one by one factor, down to `slow_down_min_speed`
//! (a bead slower already keeps its speed); then, when that is not enough, the beads that show
//! too. The surface keeps one speed from layer to layer, where a layer slowed whole would lay it
//! slower than its neighbours, and show as a band. Travels, wipes and lifts keep their speed.
//! The layer's parameters are those at its centre.

use crate::moves::chained;
use crate::plan::check;
use crate::{MotionError, MotionParams, Program};
use layercraft_path::{Face, Kind};
use layercraft_physics::Physics;
use layercraft_physics::column::{Column, Layer as Slab};
use layercraft_tune::{Field, centre};

/// Slows each layer of `program` that would print faster than it can cool.
pub fn cool(program: &mut Program, field: &Field<MotionParams>) -> Result<(), MotionError> {
    let mut at = None;
    let mut column: Option<Column> = None;
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
        // The rest of the layer: travels, and filament pushed or pulled in place.
        let other: f64 = layer
            .moves
            .iter()
            .zip(&lengths)
            .filter(|(m, _)| m.bead.is_none() && m.speed > 0.0)
            .map(|(m, l)| if *l > 0.0 { l / m.speed } else { m.e.abs() / m.speed })
            .sum();
        let filament_area = std::f64::consts::PI * (p.filament_diameter / 2.0).powi(2);
        let slab = Slab {
            volume: layer
                .moves
                .iter()
                .filter(|m| m.bead.is_some() && m.e > 0.0)
                .map(|m| m.e * filament_area)
                .sum(),
            height: layer.height,
            perimeter: layer
                .moves
                .iter()
                .zip(&lengths)
                .filter(|(m, _)| {
                    m.bead.is_some_and(|b| b.role.kind == Kind::Wall && b.role.face == Face::Outside)
                })
                .map(|(_, l)| l)
                .sum(),
            fan: layer.moves.iter().filter(|m| m.bead.is_some()).map(|m| m.fan).fold(0.0, f64::max),
        };
        let goal = match &p.physics {
            Some(physics) => {
                let column = column.get_or_insert_with(|| Column::new(physics.clone()));
                needed(column, &slab, busy + other, p.resolution, physics) - other
            }
            None => p.slow_down_layer_time,
        };
        if !(busy > 0.0 && busy < goal) {
            if let Some(column) = &mut column {
                column.lay(&slab, busy + other);
            }
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
        let laid = time(false, hidden) + time(true, visible);
        for (m, &v) in layer.moves.iter_mut().zip(&shows).filter(|(m, _)| m.bead.is_some()) {
            m.speed = slowed(m.speed, if v { visible } else { hidden }, floor);
        }
        if let Some(column) = &mut column {
            column.lay(&slab, laid + other);
        }
    }
    Ok(())
}

/// Doublings of a layer's time while waiting longer still helps: past a millionfold, nothing
/// waits that long.
const DOUBLINGS: usize = 20;

/// The shortest time from `t0` on, in s, for `slab` laid on `column` to be squeezed by the next
/// layer by as few whole resolutions as any longer wait would leave.
fn needed(column: &Column, slab: &Slab, t0: f64, resolution: f64, physics: &Physics) -> f64 {
    if !(resolution > 0.0 && t0 > 0.0) {
        return t0;
    }
    let quanta = |t: f64| {
        let temperature = column.clone().lay(slab, t);
        (physics.squeeze(slab.height, temperature) / resolution).floor()
    };
    let at_t0 = quanta(t0);
    if at_t0 == 0.0 {
        return t0;
    }
    // The least squeeze waiting reaches, and a time that reaches it.
    let (mut hi, mut least) = (t0, at_t0);
    for _ in 0..DOUBLINGS {
        let q = quanta(2.0 * hi);
        if q >= least {
            break;
        }
        (hi, least) = (2.0 * hi, q);
        if q == 0.0 {
            break;
        }
    }
    let mut lo = t0;
    for _ in 0..HALVINGS {
        let mid = (lo + hi) / 2.0;
        if quanta(mid) <= least { hi = mid } else { lo = mid }
    }
    hi
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
