//! Stage 6: the program as G-code text, in the block layout Bambu Lab printers read.
//!
//! The text has a header block (time, layers, filament), a configuration block (every setting,
//! one per line) and the executable block. Positions are absolute and extrusion relative
//! (`M83`); a move names only the axes that change and a feed rate only when it changes, and a
//! move that changes nothing at the precision written is left out. A bead's filament is written
//! to `FILAMENT_DECIMALS` with the rounding carried to the next bead, so the total stays exact; a bead
//! too short to be written carries all of its filament on. Each move carries its whole
//! state; the acceleration (`M204`), the fan (`M106`) and the feature comments are written when
//! they change, and again after the machine's own G-code. An arc is `G2`/`G3` with its centre
//! relative to the nozzle, a full turn back over its start `G3 Z… I… J… P1`. The printer's
//! progress (`M73`) is the planner's time at each pass's start. Positions are written to
//! `POSITION_DECIMALS` (`layercraft_motion`). `stats` reads G-code back, from layercraft or another
//! slicer, to compare them.

mod read;
mod stats;

pub use read::{ReadError, read};
pub use stats::{LayerStats, Stats, stats};

use layercraft_motion::{Arc, Frame, Move, POSITION_DECIMALS, Program, Summary, disc_area};
use layercraft_path::Role;
use std::fmt::Write;

/// The decimals a bead's filament is written to, in mm.
const FILAMENT_DECIMALS: usize = 5;

/// What the header block states besides the program's own summary.
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    pub generator: String,
    /// Each material's filament diameter, in mm, by material.
    pub filament_diameters: Vec<f64>,
    /// Each material's filament density, in g/cm³, by material.
    pub filament_densities: Vec<f64>,
    /// Settings written to the configuration block, as `key = value`.
    pub settings: Vec<(String, String)>,
}

impl Header {
    /// Material `k`'s filament diameter and density, the last given standing for those past them.
    fn filament(&self, k: usize) -> (f64, f64) {
        let of = |list: &[f64]| list.get(k).or(list.last()).copied().unwrap_or(0.0);
        (of(&self.filament_diameters), of(&self.filament_densities))
    }
}

/// A number with at most `decimals` decimals and no trailing zeros.
pub fn number(v: f64, decimals: usize) -> String {
    let text = format!("{v:.decimals$}");
    let text = if text.contains('.') { text.trim_end_matches('0').trim_end_matches('.') } else { &text };
    if text == "-0" { "0".to_string() } else { text.to_string() }
}

/// `v` as written at `POSITION_DECIMALS`, read back.
fn written(v: f64) -> Option<f64> {
    number(v, POSITION_DECIMALS).parse().ok()
}

/// `v` written as a position.
fn position(v: f64) -> String {
    number(v, POSITION_DECIMALS)
}

fn duration(seconds: f64) -> String {
    let s = seconds.max(0.0).round() as u64;
    match (s / 3600, (s / 60) % 60, s % 60) {
        (0, 0, s) => format!("{s}s"),
        (0, m, s) => format!("{m}m {s}s"),
        (h, m, s) => format!("{h}h {m}m {s}s"),
    }
}

#[derive(Default)]
struct Writer {
    out: String,
    at: [Option<f64>; 3],
    feed: Option<f64>,
    /// Whether arcs are known to be in the XY plane (`G17`).
    plane: bool,
    accel: Option<f64>,
    fan: Option<f64>,
    feature: Option<(Role, f64)>,
    /// The beads' filament not written yet, in mm: rounding, and beads too short to write.
    carry: f64,
}

impl Writer {
    fn line(&mut self, text: &str) {
        self.out.push_str(text);
        self.out.push('\n');
    }

    /// The machine's own G-code, after which nothing is known.
    fn machine(&mut self, text: &str) {
        self.out.push_str(text);
        if !text.ends_with('\n') {
            self.out.push('\n');
        }
        *self = Writer { out: std::mem::take(&mut self.out), carry: self.carry, ..Writer::default() };
    }

    fn feed(&mut self, speed: f64, words: &mut String) {
        let f = (speed * 60.0).round();
        if self.feed != Some(f) {
            let _ = write!(words, " F{}", number(f, 0));
            self.feed = Some(f);
        }
    }

    /// The axes of `to` that differ from where the nozzle is, as written, and records them.
    fn axes(&mut self, to: [f64; 3], words: &mut String) {
        for (axis, (name, v)) in ["X", "Y", "Z"].into_iter().zip(to).enumerate() {
            let rounded = written(v).unwrap_or(v);
            if self.at.get(axis).copied().flatten() != Some(rounded) {
                let _ = write!(words, " {name}{}", position(v));
                if let Some(slot) = self.at.get_mut(axis) {
                    *slot = Some(rounded);
                }
            }
        }
    }

    fn state(&mut self, m: &Move) {
        if let Some(bead) = m.bead
            && self.feature != Some((bead.role, bead.width))
        {
            self.line(&format!("; FEATURE: {}", bead.role.name()));
            self.line(&format!("; LINE_WIDTH: {}", position(bead.width)));
            self.feature = Some((bead.role, bead.width));
        }
        let fan = (m.fan * 255.0).round();
        if self.fan != Some(fan) {
            self.line(&format!("M106 S{}", number(fan, 0)));
            self.fan = Some(fan);
        }
        if self.accel != Some(m.accel) {
            self.line(&format!("M204 S{}", number(m.accel, 0)));
            self.accel = Some(m.accel);
        }
    }

    /// The words of an arc to `to` about `arc` from (`x`, `y`), as written, and records where
    /// it ends: a full turn back over its start is `P1` with no end named.
    fn arc_words(&mut self, to: [f64; 3], arc: Arc, x: f64, y: f64) -> String {
        let (i, j) = (position(arc.centre[0] - x), position(arc.centre[1] - y));
        let full = (arc.turn.abs() - std::f64::consts::TAU).abs() < 1e-9;
        let mut words = String::from(if arc.turn > 0.0 { "G3" } else { "G2" });
        let z = written(to[2]);
        if full {
            let _ = write!(words, " Z{} I{i} J{j} P1", position(to[2]));
        } else {
            let _ = write!(words, " X{} Y{}", position(to[0]), position(to[1]));
            if self.at[2] != z {
                let _ = write!(words, " Z{}", position(to[2]));
            }
            let _ = write!(words, " I{i} J{j}");
            self.at[0] = written(to[0]);
            self.at[1] = written(to[1]);
        }
        self.at[2] = z;
        words
    }

    /// Writes a move, and the state it needs before it, unless it changes nothing at the
    /// precision written; then its wait, as the printer's own dwell, in milliseconds.
    fn write_move(&mut self, m: &Move) {
        self.write_motion(m);
        let ms = (m.wait * 1000.0).round();
        if ms > 0.0 {
            self.state(m);
            self.line(&format!("M400 P{}", number(ms, 0)));
        }
    }

    fn write_motion(&mut self, m: &Move) {
        // A bead's filament with what earlier beads carried; the rest carries on.
        let wanted = if m.bead.is_some() { m.e + self.carry } else { m.e };
        let e = Some(number(wanted, FILAMENT_DECIMALS)).filter(|e| e != "0");
        let words = match (m.to, m.arc, self.at) {
            (None, _, _) => e.as_ref().map(|e| format!("G1 E{e}")),
            // An arc from nowhere known, which `verify` refuses, is never written as one.
            (Some(to), Some(arc), [Some(x), Some(y), _]) => Some(self.arc_words(to, arc, x, y)),
            (Some(to), _, _) => {
                let mut words = String::from("G1");
                self.axes(to, &mut words);
                (words != "G1" || e.is_some()).then_some(words)
            }
        };
        let Some(mut words) = words.filter(|w| m.bead.is_none() || w.contains(['X', 'Y', 'I'])) else {
            if m.bead.is_some() {
                self.carry = wanted;
            }
            return;
        };
        if m.bead.is_some() {
            self.carry = wanted - e.as_deref().and_then(|e| e.parse::<f64>().ok()).unwrap_or(0.0);
        }
        if m.to.is_some()
            && let Some(e) = &e
        {
            let _ = write!(words, " E{e}");
        }
        self.state(m);
        if (words.starts_with("G2") || words.starts_with("G3")) && !self.plane {
            self.line("G17");
            self.plane = true;
        }
        self.feed(m.speed, &mut words);
        self.line(&words);
    }
}

/// Writes the program, in its frame, as G-code: `summary` is what it does (`summarize`), and
/// `times` the planner's time at the end of each move (`estimate`), for the printer's progress.
pub fn write(program: &Program, frame: &Frame, s: &Summary, times: &[f64], header: &Header) -> String {
    let mut w = Writer::default();
    w.line("; HEADER_BLOCK_START");
    w.line(&format!("; {}", header.generator));
    let time = duration(s.motion_seconds);
    w.line(&format!("; model printing time: {time}; total estimated time: {time}"));
    w.line(&format!("; total layer number: {}", s.layers));
    // One entry per material, as the printer lists its filaments.
    let n = s.filament_mm_by_material.len().max(1);
    let each = |f: &dyn Fn(usize, f64) -> String| {
        let mm = |k: usize| s.filament_mm_by_material.get(k).copied().unwrap_or(0.0);
        (0..n).map(|k| f(k, mm(k))).collect::<Vec<_>>().join(",")
    };
    let area = |k: usize| disc_area(header.filament(k).0);
    w.line(&format!("; total filament length [mm] : {}", each(&|_, mm| number(mm, 2))));
    w.line(&format!("; total filament volume [cm^3] : {}", each(&|k, mm| number(mm * area(k) / 1000.0, 2))));
    w.line(&format!(
        "; total filament weight [g] : {}",
        each(&|k, mm| number(mm * area(k) * header.filament(k).1 / 1000.0, 2))
    ));
    w.line(&format!("; filament_density: {}", each(&|k, _| number(header.filament(k).1, 3))));
    w.line(&format!("; filament_diameter: {}", each(&|k, _| number(header.filament(k).0, 3))));
    w.line(&format!("; max_z_height: {}", number(s.max_z, 2)));
    w.line(&format!("; filament: {}", (1..=n).map(|k| k.to_string()).collect::<Vec<_>>().join(",")));
    w.line("; HEADER_BLOCK_END");
    w.line("");
    w.line("; CONFIG_BLOCK_START");
    for (key, value) in &header.settings {
        w.line(&format!("; {key} = {}", value.replace('\n', "\\n")));
    }
    w.line("; CONFIG_BLOCK_END");
    w.line("");
    w.line("; EXECUTABLE_BLOCK_START");
    w.machine(&frame.start);
    let total = times.last().copied().unwrap_or(0.0);
    let mut done = 0usize;
    let mut texts = frame.passes.iter();
    for layer in program.layers() {
        // A layer printed in several passes is announced once, before its first.
        if let Some(first) = layer.first() {
            w.line("; CHANGE_LAYER");
            w.line(&format!("; Z_HEIGHT: {}", position(first.z)));
            w.line(&format!("; LAYER_HEIGHT: {}", position(first.height)));
        }
        for pass in layer {
            if let Some(text) = texts.next() {
                w.machine(text);
            }
            let t = done.checked_sub(1).and_then(|i| times.get(i)).copied().unwrap_or(0.0);
            let percent = if total > 0.0 { (100.0 * t / total).floor() as u32 } else { 0 };
            let left = ((total - t) / 60.0).ceil().max(0.0) as u32;
            w.line(&format!("M73 P{percent} R{left}"));
            for m in &pass.moves {
                w.write_move(m);
            }
            done += pass.moves.len();
        }
    }
    w.machine(&frame.end);
    w.line("M73 P100 R0");
    w.line("; EXECUTABLE_BLOCK_END");
    w.out
}

#[cfg(test)]
mod tests;
