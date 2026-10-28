//! G-code read back into a program, the writer's inverse: from layercraft or another slicer.
//!
//! Only the moves are kept; the machine's own G-code is the frame, which the program gets anew
//! when it is written: the start before the first layer marker, the end after
//! `MACHINE_END_GCODE_START`, the printer's conditional blocks (`M622 J…`, `M623`), skippable
//! blocks (timelapse), and a closed list of machine commands. Any other command is an error.
//! Positions, filament (absolute or relative), feed, acceleration and fan are followed as the
//! printer would; an axis the G-code has not named yet takes the next value it is given. A
//! layer starts at its first travel to a new point or its first extrusion: the wipe,
//! retraction and lift before it leave the layer below, at its height. An arc keeps its
//! centre and the angle it turns; `; FEATURE` and `; LINE_WIDTH` name the beads.

use layercraft_motion::{Arc, Bead, Layer, Move, Program};
use layercraft_path::Role;

/// Why G-code could not be read.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[error("G-code line {line}: {reason}")]
pub struct ReadError {
    pub line: usize,
    pub reason: String,
}

/// Machine commands that belong to the frame: progress, notices, auxiliary fans, chamber,
/// detectors, power-loss recovery, and the printer's conditional flags.
const FRAME: [&str; 11] =
    ["M73", "M991", "M1002", "M622.1", "M971", "M1004", "M1003", "M142", "M981", "M400", "M9711"];

/// A move as written, its axes possibly unknown yet.
struct Raw {
    line: usize,
    /// Filament pushed or pulled where the nozzle is, naming no axis.
    in_place: bool,
    to: [Option<f64>; 3],
    arc: Option<(bool, [f64; 2], bool)>,
    e: f64,
    speed: f64,
    accel: f64,
    fan: f64,
    bead: Option<Bead>,
}

/// The number a G-code command gives after `letter`.
pub(crate) fn word(code: &str, letter: char) -> Option<f64> {
    code.split_whitespace().skip(1).find_map(|w| w.strip_prefix(letter)?.parse().ok())
}

fn has(code: &str, letter: char) -> bool {
    code.split_whitespace().skip(1).any(|w| w.starts_with(letter))
}

/// The angle turned about `c` from `a` to `b`, counter-clockwise when `ccw`, a full turn when
/// they meet.
fn turn(a: [f64; 2], b: [f64; 2], c: [f64; 2], ccw: bool) -> f64 {
    let tau = std::f64::consts::TAU;
    let (a0, a1) = ((a[1] - c[1]).atan2(a[0] - c[0]), (b[1] - c[1]).atan2(b[0] - c[0]));
    let sweep = if ccw { (a1 - a0).rem_euclid(tau) } else { (a0 - a1).rem_euclid(tau) };
    let sweep = if sweep < 1e-9 { tau } else { sweep };
    if ccw { sweep } else { -sweep }
}

/// Reads the moves of G-code text into a program.
pub fn read(text: &str) -> Result<Program, ReadError> {
    let err = |line: usize, reason: String| ReadError { line, reason };
    let mut layers: Vec<(Layer, Vec<Raw>)> = Vec::new();
    // A layer marker read, its height and thickness still to come.
    let mut pending: Option<(f64, f64)> = None;
    let (mut started, mut ended) = (false, false);
    let (mut skipping, mut conditional) = (0usize, 0usize);
    let (mut absolute, mut absolute_e, mut last_e) = (true, false, 0.0);
    let mut at: [Option<f64>; 3] = [None; 3];
    let (mut feed, mut accel, mut fan) = (None::<f64>, None::<f64>, 0.0);
    let (mut feature, mut width): (Option<String>, Option<f64>) = (None, None);
    // In a new layer's prelude: its moves still leave the layer below.
    let mut prelude = false;
    let mut raws: Vec<Raw> = Vec::new();
    for (n, full) in text.lines().enumerate() {
        let line = n + 1;
        let (code, comment) = match full.split_once(';') {
            Some((c, m)) => (c.trim(), m.trim()),
            None => (full.trim(), ""),
        };
        ended |= comment.starts_with("MACHINE_END_GCODE_START");
        match comment {
            "SKIPPABLE_START" => skipping += 1,
            "SKIPPABLE_END" => skipping = skipping.saturating_sub(1),
            "CHANGE_LAYER" if !ended => {
                started = true;
                pending = Some((f64::NAN, f64::NAN));
                continue;
            }
            _ => {}
        }
        if let Some((z, h)) = pending.as_mut() {
            if !code.is_empty() {
                return Err(err(line, "G-code between a layer marker and its height".into()));
            }
            if let Some(v) = comment.strip_prefix("Z_HEIGHT:") {
                *z = v.trim().parse().map_err(|_| err(line, format!("layer height `{v}`")))?;
            } else if let Some(v) = comment.strip_prefix("LAYER_HEIGHT:") {
                *h = v.trim().parse().map_err(|_| err(line, format!("layer thickness `{v}`")))?;
            }
            if z.is_finite() && h.is_finite() {
                let index = layers.len();
                if let Some((_, moves)) = layers.last_mut() {
                    moves.append(&mut raws);
                }
                layers.push((Layer { index, z: *z, height: *h, moves: Vec::new() }, Vec::new()));
                prelude = index > 0;
                pending = None;
            }
            continue;
        }
        let head = code.split_whitespace().next().unwrap_or("");
        match head {
            "M622" => conditional += 1,
            "M623" => conditional = conditional.saturating_sub(1),
            _ => {}
        }
        if conditional > 0 || skipping > 0 || head == "M623" {
            continue;
        }
        // The machine's state, in the frame as in the moves.
        match head {
            "G90" => absolute = true,
            "G91" => absolute = false,
            "M82" => absolute_e = true,
            "M83" => absolute_e = false,
            "G92" => last_e = word(code, 'E').unwrap_or(last_e),
            "M204" => accel = word(code, 'S').or(accel),
            "M106" if !has(code, 'P') => fan = (word(code, 'S').unwrap_or(0.0) / 255.0).clamp(0.0, 1.0),
            _ => {}
        }
        let body = started && !ended;
        if !body {
            continue;
        }
        if let Some(name) = comment.strip_prefix("FEATURE:") {
            feature = Some(name.trim().to_string());
        } else if let Some(w) = comment.strip_prefix("LINE_WIDTH:") {
            width = w.trim().parse().ok();
        }
        match head {
            "" | "G90" | "G91" | "M82" | "M83" | "G92" | "M204" | "M106" | "G17" | "G21" => continue,
            _ if FRAME.contains(&head) => continue,
            "G0" | "G1" | "G2" | "G3" => {}
            other => return Err(err(line, format!("an unknown command `{other}`"))),
        }
        if let Some(f) = word(code, 'F') {
            feed = Some(f / 60.0);
        }
        let named = [word(code, 'X'), word(code, 'Y'), word(code, 'Z')];
        let arc = matches!(head, "G2" | "G3");
        if named.iter().all(Option::is_none) && !has(code, 'E') && !arc {
            continue;
        }
        let mut to = at;
        for (k, v) in named.into_iter().enumerate() {
            if let (Some(v), Some(slot)) = (v, to.get_mut(k)) {
                *slot = Some(if absolute { v } else { at.get(k).copied().flatten().unwrap_or(0.0) + v });
            }
        }
        let e = match word(code, 'E') {
            Some(v) if absolute_e => {
                let d = v - last_e;
                last_e = v;
                d
            }
            Some(v) => v,
            None => 0.0,
        };
        let arc = if arc {
            let (Some(x), Some(y)) = (at[0], at[1]) else {
                return Err(err(line, "an arc from where the nozzle is not known".into()));
            };
            if word(code, 'P').is_some_and(|p| p > 1.0) {
                return Err(err(line, "an arc of more than one turn".into()));
            }
            let centre = [x + word(code, 'I').unwrap_or(0.0), y + word(code, 'J').unwrap_or(0.0)];
            let full = has(code, 'P') && named[0].is_none() && named[1].is_none();
            Some((head == "G3", centre, full))
        } else {
            None
        };
        let moves_xy = to[0] != at[0] || to[1] != at[1];
        // Naming a point, even the one it is at, it prints; naming none, it primes.
        let extrudes = e > 0.0 && (named[0].is_some() || named[1].is_some() || arc.is_some());
        if prelude && ((moves_xy && e == 0.0 && arc.is_none()) || extrudes) {
            // The new layer's first travel or bead: the prelude leaves the layer below.
            if let Some((_, moves)) = layers.iter_mut().rev().nth(1) {
                moves.append(&mut raws);
            }
            prelude = false;
        }
        let bead = if extrudes {
            let name =
                feature.as_deref().ok_or_else(|| err(line, "an extrusion before any feature".into()))?;
            let role =
                Role::from_name(name).ok_or_else(|| err(line, format!("an unknown feature `{name}`")))?;
            let width = width.ok_or_else(|| err(line, "an extrusion before any line width".into()))?;
            Some(Bead { role, width })
        } else {
            None
        };
        let speed = feed.ok_or_else(|| err(line, "a move before any feed rate".into()))?;
        let accel = accel.ok_or_else(|| err(line, "a move before any acceleration".into()))?;
        let in_place = named.iter().all(Option::is_none) && arc.is_none();
        raws.push(Raw { line, in_place, to, arc, e, speed, accel, fan, bead });
        at = to;
    }
    if let Some((_, moves)) = layers.last_mut() {
        moves.append(&mut raws);
    }
    resolve(layers)
}

/// Fills each axis not named yet with the next value it is given, and turns raw moves into
/// the program's: a move that goes nowhere pushes or pulls in place, one that does nothing is
/// dropped.
fn resolve(mut layers: Vec<(Layer, Vec<Raw>)>) -> Result<Program, ReadError> {
    // Backwards, the next value each axis is given.
    let mut next: [Option<f64>; 3] = [None; 3];
    for (_, moves) in layers.iter_mut().rev() {
        for m in moves.iter_mut().rev().filter(|m| !m.in_place) {
            for (slot, n) in m.to.iter_mut().zip(next.iter_mut()) {
                match slot {
                    Some(v) => *n = Some(*v),
                    None => *slot = *n,
                }
            }
        }
    }
    let mut at: Option<[f64; 3]> = None;
    let mut out: Vec<Layer> = Vec::with_capacity(layers.len());
    for (mut layer, moves) in layers {
        for m in moves {
            let (to, arc, bead) = if m.in_place {
                (None, None, None)
            } else {
                let [Some(x), Some(y), Some(z)] = m.to else {
                    return Err(ReadError { line: m.line, reason: "a move to an axis never named".into() });
                };
                let arc = match (m.arc, at) {
                    (Some((ccw, centre, full)), Some(from)) => {
                        let tau = std::f64::consts::TAU;
                        let t = match (full, ccw) {
                            (true, true) => tau,
                            (true, false) => -tau,
                            (false, _) => turn([from[0], from[1]], [x, y], centre, ccw),
                        };
                        Some(Arc { centre, turn: t })
                    }
                    _ => None,
                };
                let goes = at != Some([x, y, z]) || arc.is_some();
                if !goes && m.e == 0.0 {
                    continue;
                }
                (goes.then_some([x, y, z]), arc, m.bead)
            };
            layer.moves.push(Move { to, arc, e: m.e, speed: m.speed, accel: m.accel, fan: m.fan, bead });
            at = to.or(at);
        }
        out.push(layer);
    }
    Ok(Program { layers: out })
}
