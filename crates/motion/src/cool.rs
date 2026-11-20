//! Giving each layer the time to cool before the next lands on it, over any program: compiled,
//! read from another slicer's G-code, merged or edited.
//!
//! A layer, all its passes, whose beads take less than `slow_down_layer_time` `T` at their speeds
//! is slowed until they take `T`: the hidden beads first, every one by one factor, down to its
//! material's `slow_down_min_speed` (a bead slower already keeps its speed); then, when that is not
//! enough, the beads that show too. The surface keeps one speed from layer to layer, where a layer
//! slowed whole would lay it slower than its neighbours, and show as a band. A layer still shorter
//! than `T` at that floor, as small as a chimney's, waits out the rest after its last pass: from
//! its hop over the layer the nozzle goes `CLEARANCE` past every bead of the layer, the nearest way
//! that stays on the plate, where the fan cools them and the hot end no longer warms them. Nothing
//! lands on the last layer: it does not wait. Travels, wipes and lifts keep their speed. Each pass
//! takes its material's parameters at the layer's centre, the layer the longest `T` among them.

use crate::moves::{Point3, chained, end, lengths};
use crate::park::clear_of;
use crate::{MotionError, MotionParams, Move, Pass, Program};
use layercraft_geom::Point;
use layercraft_path::Face;
use layercraft_tune::{Field, centre};
use std::rc::Rc;

/// How far past a layer's beads its nozzle waits, in mm: a hot end some 20 mm across warms what
/// lies under it.
const CLEARANCE: f64 = 10.0;

/// The most a chord of an arc may fall short of it, in mm, when a bead's arc is sampled: far
/// under the clearance, so the beads' hull is theirs to that.
const ARC_SAMPLING_MM: f64 = 1e-3;

/// Points along bead `m`, laid from `from`, whose hull holds it: its ends, and for an arc chords
/// within `ARC_SAMPLING_MM` of it.
fn sampled(m: &Move, from: Point3) -> Vec<Point> {
    let steps = m.chords(from, ARC_SAMPLING_MM);
    (0..=steps).map(|k| m.point(from, k as f64 / steps as f64)).map(|p| [p[0], p[1]]).collect()
}

/// One bead of a layer, as its cooling sees it.
struct Laid {
    /// Its pass in the layer and its place among the pass's moves.
    pass: usize,
    index: usize,
    length: f64,
    speed: f64,
    /// Whether it shows on the part's surface.
    shows: bool,
    /// The slowest its material lays it.
    floor: f64,
}

/// Gives each layer of `program` the time to cool before the next, each pass with its material's
/// `fields` (the last given standing for those past them), on a plate whose printable area is
/// `area`, `[min, max]`.
pub fn cool(
    program: &mut Program,
    fields: &[Field<MotionParams>],
    area: [[f64; 2]; 2],
) -> Result<(), MotionError> {
    let last = fields.last().ok_or_else(|| MotionError::BadParams("no material's parameters".into()))?;
    let mut at = None;
    let count = program.layers().count();
    for (k, layer) in program.layers_mut().enumerate() {
        let ends: Vec<Point> = layer
            .iter()
            .flat_map(|p| &p.moves)
            .filter(|m| m.bead.is_some())
            .filter_map(|m| m.to)
            .map(|p| [p[0], p[1]])
            .collect();
        let middle = centre(&ends, layer.first().map_or(0.0, |p| p.z));
        let params = layer
            .iter()
            .map(|p| fields.get(p.material).unwrap_or(last).at(middle, MotionParams::check))
            .collect::<Result<Vec<Rc<MotionParams>>, MotionError>>()?;
        let Some(leaving) = params.last().cloned() else { continue };
        // Where the nozzle enters each pass.
        let mut starts = Vec::with_capacity(layer.len());
        let mut beads = Vec::new();
        for (i, (pass, p)) in layer.iter().zip(&params).enumerate() {
            starts.push(at);
            let laid = pass.moves.iter().zip(lengths(&pass.moves, at)).enumerate();
            for (index, (m, length)) in laid.filter(|(_, (m, _))| m.bead.is_some() && m.speed > 0.0) {
                let shows = m.bead.is_some_and(|b| b.role.face != Face::Inside);
                beads.push(Laid {
                    pass: i,
                    index,
                    length,
                    speed: m.speed,
                    shows,
                    floor: p.slow_down_min_speed,
                });
            }
            at = end(&pass.moves, at);
        }
        let goal = params.iter().map(|p| p.slow_down_layer_time).fold(0.0, f64::max);
        // The time of the beads that show (`visible`) or not, each slowed by `s` above its floor.
        let time = |visible: bool, s: f64| -> f64 {
            beads.iter().filter(|b| b.shows == visible).map(|b| b.length / slowed(b.speed, s, b.floor)).sum()
        };
        let busy: f64 = beads.iter().map(|b| b.length / b.speed).sum();
        if !(busy > 0.0 && busy < goal) {
            continue;
        }
        let slowest = time(false, 0.0) + time(true, 0.0);
        // Hidden beads first; the visible ones only once the hidden are all at their floor.
        let (hidden, visible) = if time(false, 0.0) + time(true, 1.0) >= goal {
            (factor(|s| time(false, s) + time(true, 1.0), goal), 1.0)
        } else {
            (0.0, factor(|s| time(false, 0.0) + time(true, s), goal))
        };
        for b in &beads {
            let speed = slowed(b.speed, if b.shows { visible } else { hidden }, b.floor);
            if let Some(m) = layer.get_mut(b.pass).and_then(|pass| pass.moves.get_mut(b.index)) {
                m.speed = speed;
            }
        }
        if slowest < goal && k + 1 < count {
            wait_off(layer, &starts, &leaving, area, goal - slowest);
            at = layer.last().and_then(|pass| end(&pass.moves, None)).or(at);
        }
    }
    Ok(())
}

/// Ends `layer`, its passes entered from `starts`, with the nozzle `seconds` off its beads: up to
/// its hop, over to the nearest point `CLEARANCE` from every bead within `area` (`park`), and
/// there, still, with the last pass's parameters `p`. With nowhere on the plate that far off, it
/// waits over the layer at its hop.
fn wait_off(
    layer: &mut [Pass],
    starts: &[Option<Point3>],
    p: &MotionParams,
    area: [[f64; 2]; 2],
    seconds: f64,
) {
    let mut beads: Vec<Point> = Vec::new();
    let mut heading = [0.0, 0.0];
    for (pass, &start) in layer.iter().zip(starts) {
        for (from, m) in chained(&pass.moves, start).filter(|(_, m)| m.bead.is_some()) {
            let Some(from) = from else { continue };
            beads.extend(sampled(m, from));
            let (_, arrival) = m.directions(from);
            let along = arrival[0].hypot(arrival[1]);
            if along > 0.0 {
                heading = [arrival[0] / along, arrival[1] / along];
            }
        }
    }
    let (Some(pass), Some(&start)) = (layer.last_mut(), starts.last()) else { return };
    let stop = end(&pass.moves, start);
    let fan = pass.moves.last().map_or(p.fan_speed, |m| m.fan);
    let go = |to: Point3| Move {
        to: Some(to),
        arc: None,
        e: 0.0,
        speed: p.travel.speed,
        accel: p.travel.accel,
        fan,
        bead: None,
        wait: 0.0,
    };
    if let Some(stop) = stop {
        let z = stop[2].max(pass.highest() + p.z_hop);
        if stop[2] < z {
            pass.moves.push(go([stop[0], stop[1], z]));
        }
        if let Some(q) = clear_of(&beads, [stop[0], stop[1]], heading, CLEARANCE, area) {
            pass.moves.push(go([q[0], q[1], z]));
        }
    }
    pass.moves.push(Move { to: None, wait: seconds, ..go([0.0; 3]) });
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
