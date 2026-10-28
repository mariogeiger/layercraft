//! Giving each layer the time to cool before the next lands on it, over any program: compiled,
//! read from another slicer's G-code, merged or edited.
//!
//! A layer whose beads take less than `slow_down_layer_time` `T` at their speeds is slowed until
//! they take `T`: the hidden beads first, every one by one factor, down to `slow_down_min_speed`
//! (a bead slower already keeps its speed); then, when that is not enough, the beads that show
//! too. The surface keeps one speed from layer to layer, where a layer slowed whole would lay it
//! slower than its neighbours, and show as a band. A layer still shorter than `T` at that floor,
//! as small as a chimney's, waits out the rest before the next lands on it: from its hop over
//! the layer the nozzle goes `CLEARANCE` past the layer's beads, the nearest way that stays on
//! the plate, where the fan cools them and the hot end no longer warms them. Nothing lands on
//! the last layer: it does not wait. Travels, wipes and lifts keep their speed. The layer's
//! parameters are those at its centre.

use crate::moves::{chained, lengths};
use crate::plan::check;
use crate::{Layer, MotionError, MotionParams, Move, Program};
use layercraft_path::Face;
use layercraft_tune::{Field, centre};

/// How far past a layer's beads its nozzle waits, in mm: a hot end some 20 mm across warms what
/// lies under it.
const CLEARANCE: f64 = 10.0;

/// Gives each layer of `program` the time to cool before the next, on a plate whose printable
/// area is `area`, `[min, max]`.
pub fn cool(
    program: &mut Program,
    field: &Field<MotionParams>,
    area: [[f64; 2]; 2],
) -> Result<(), MotionError> {
    let mut at = None;
    let count = program.layers.len();
    for (k, layer) in program.layers.iter_mut().enumerate() {
        let ends: Vec<[f64; 2]> = layer
            .moves
            .iter()
            .filter(|m| m.bead.is_some())
            .filter_map(|m| m.to)
            .map(|p| [p[0], p[1]])
            .collect();
        let p = field.at(centre(&ends, layer.z), check)?;
        let start = at;
        let lengths = lengths(&layer.moves, &mut at);
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
        let slowest = time(false, 0.0) + time(true, 0.0);
        // Hidden beads first; the visible ones only once the hidden are all at their floor.
        let (hidden, visible) = if time(false, 0.0) + time(true, 1.0) >= goal {
            (factor(|s| time(false, s) + time(true, 1.0), goal), 1.0)
        } else {
            (0.0, factor(|s| time(false, 0.0) + time(true, s), goal))
        };
        for (m, &v) in layer.moves.iter_mut().zip(&shows).filter(|(m, _)| m.bead.is_some()) {
            m.speed = slowed(m.speed, if v { visible } else { hidden }, floor);
        }
        if slowest < goal && k + 1 < count {
            wait_off(layer, start, &p, area, goal - slowest);
        }
    }
    Ok(())
}

/// Ends `layer`, entered from `start`, with the nozzle `seconds` off its beads: up to its hop,
/// over to `CLEARANCE` past their extent the nearest way within `area`, and there, still. With
/// nowhere on the plate that far off, it waits over the layer at its hop.
fn wait_off(layer: &mut Layer, start: Option<[f64; 3]>, p: &MotionParams, area: [[f64; 2]; 2], seconds: f64) {
    let extent = chained(&layer.moves, start)
        .filter(|(_, m)| m.bead.is_some())
        .filter_map(|(from, m)| Some(m.bounds(from?)))
        .reduce(|a, b| {
            [[a[0][0].min(b[0][0]), a[0][1].min(b[0][1])], [a[1][0].max(b[1][0]), a[1][1].max(b[1][1])]]
        });
    let end = layer.moves.iter().rev().find_map(|m| m.to).or(start);
    let fan = layer.moves.last().map_or(p.fan_speed, |m| m.fan);
    let go = |to: [f64; 3]| Move {
        to: Some(to),
        arc: None,
        e: 0.0,
        speed: p.travel.speed,
        accel: p.travel.accel,
        fan,
        bead: None,
        wait: 0.0,
    };
    if let Some(end) = end {
        let z = end[2].max(layer.z + p.z_hop);
        if end[2] < z {
            layer.moves.push(go([end[0], end[1], z]));
        }
        let inside = |q: [f64; 2]| {
            (area[0][0]..=area[1][0]).contains(&q[0]) && (area[0][1]..=area[1][1]).contains(&q[1])
        };
        let off = extent.into_iter().flat_map(|[lo, hi]| {
            let c = CLEARANCE;
            [[lo[0] - c, end[1]], [hi[0] + c, end[1]], [end[0], lo[1] - c], [end[0], hi[1] + c]]
        });
        let distance = |q: &[f64; 2]| (q[0] - end[0]).hypot(q[1] - end[1]);
        if let Some(q) = off.filter(|&q| inside(q)).min_by(|a, b| distance(a).total_cmp(&distance(b))) {
            layer.moves.push(go([q[0], q[1], z]));
        }
    }
    layer.moves.push(Move { to: None, wait: seconds, ..go([0.0; 3]) });
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
