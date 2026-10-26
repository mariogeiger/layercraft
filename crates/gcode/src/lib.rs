//! Stage 6: the program as G-code text, in the block layout Bambu Lab printers read.
//!
//! The text has a header block (time, layers, filament), a configuration block (every setting,
//! one per line) and the executable block. Positions are absolute and extrusion relative
//! (`M83`); a move names only the axes that change and a feed rate only when it changes, and a
//! move that changes nothing at the precision written is left out. A bead's filament is written
//! to five decimals with the rounding carried to the next bead, so the total stays exact; a bead
//! too short to be written carries all of its filament on. Each move carries its whole
//! state; the acceleration (`M204`), the fan (`M106`) and the feature comments are written when
//! they change, and again after the machine's own G-code. An arc is `G2`/`G3` with its centre
//! relative to the nozzle, a full turn back over its start `G3 Z… I… J… P1`. The printer's
//! progress (`M73`) is the planner's time at each layer's start.
//! `stats` reads G-code back, from layercraft or another slicer, to compare them.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

mod read;
pub mod stats;

pub use read::{ReadError, read};

use layercraft_motion::{Arc, Frame, Limits, Move, Program, estimate, finish};
use layercraft_path::Role;
use std::fmt::Write;

/// What the header block states besides the program's own summary.
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    pub generator: String,
    pub filament_diameter: f64,
    /// In g/cm³.
    pub filament_density: f64,
    /// Settings written to the configuration block, as `key = value`.
    pub settings: Vec<(String, String)>,
}

/// A number with at most `decimals` decimals and no trailing zeros.
pub fn number(v: f64, decimals: usize) -> String {
    let text = format!("{v:.decimals$}");
    let text = if text.contains('.') { text.trim_end_matches('0').trim_end_matches('.') } else { &text };
    if text == "-0" { "0".to_string() } else { text.to_string() }
}

/// The name a role goes by in the G-code's feature comments.
pub fn feature_name(role: Role) -> &'static str {
    role.name()
}

fn duration(seconds: f64) -> String {
    let s = seconds.max(0.0).round() as u64;
    match (s / 3600, (s / 60) % 60, s % 60) {
        (0, 0, s) => format!("{s}s"),
        (0, m, s) => format!("{m}m {s}s"),
        (h, m, s) => format!("{h}h {m}m {s}s"),
    }
}

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
        self.at = [None; 3];
        self.feed = None;
        self.plane = false;
        self.accel = None;
        self.fan = None;
        self.feature = None;
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
            let rounded: f64 = number(v, 3).parse().unwrap_or(v);
            if self.at.get(axis).copied().flatten() != Some(rounded) {
                let _ = write!(words, " {name}{}", number(v, 3));
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
            self.line(&format!("; FEATURE: {}", feature_name(bead.role)));
            self.line(&format!("; LINE_WIDTH: {}", number(bead.width, 3)));
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
        let (i, j) = (number(arc.centre[0] - x, 3), number(arc.centre[1] - y, 3));
        let full = (arc.turn.abs() - std::f64::consts::TAU).abs() < 1e-9;
        let mut words = String::from(if arc.turn > 0.0 { "G3" } else { "G2" });
        let z: Option<f64> = number(to[2], 3).parse().ok();
        if full {
            let _ = write!(words, " Z{} I{i} J{j} P1", number(to[2], 3));
        } else {
            let _ = write!(words, " X{} Y{}", number(to[0], 3), number(to[1], 3));
            if self.at[2] != z {
                let _ = write!(words, " Z{}", number(to[2], 3));
            }
            let _ = write!(words, " I{i} J{j}");
            self.at[0] = number(to[0], 3).parse().ok();
            self.at[1] = number(to[1], 3).parse().ok();
        }
        self.at[2] = z;
        words
    }

    /// Writes a move, and the state it needs before it, unless it changes nothing at the
    /// precision written.
    fn write_move(&mut self, m: &Move) {
        // A bead's filament with what earlier beads carried; the rest carries on.
        let wanted = if m.bead.is_some() { m.e + self.carry } else { m.e };
        let e = Some(number(wanted, 5)).filter(|e| e != "0");
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

/// Writes the program, in its frame, as G-code; `limits` time it for the printer's progress.
pub fn write(program: &Program, frame: &Frame, limits: &Limits, header: &Header) -> String {
    let s = finish(program, limits, header.filament_diameter);
    let area = std::f64::consts::PI * (header.filament_diameter / 2.0).powi(2);
    let grams = s.filament_mm * area * header.filament_density / 1000.0;
    let mut w = Writer {
        out: String::new(),
        at: [None; 3],
        feed: None,
        plane: false,
        accel: None,
        fan: None,
        feature: None,
        carry: 0.0,
    };
    w.line("; HEADER_BLOCK_START");
    w.line(&format!("; {}", header.generator));
    let time = duration(s.motion_seconds);
    w.line(&format!("; model printing time: {time}; total estimated time: {time}"));
    w.line(&format!("; total layer number: {}", s.layers));
    w.line(&format!("; total filament length [mm] : {}", number(s.filament_mm, 2)));
    w.line(&format!("; total filament volume [cm^3] : {}", number(s.filament_mm * area / 1000.0, 2)));
    w.line(&format!("; total filament weight [g] : {}", number(grams, 2)));
    w.line(&format!("; filament_density: {}", number(header.filament_density, 3)));
    w.line(&format!("; filament_diameter: {}", number(header.filament_diameter, 3)));
    w.line(&format!("; max_z_height: {}", number(s.max_z, 2)));
    w.line("; filament: 1");
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
    let times = estimate(program, limits);
    let total = times.last().copied().unwrap_or(0.0);
    let mut done = 0usize;
    for (k, layer) in program.layers.iter().enumerate() {
        w.line("; CHANGE_LAYER");
        w.line(&format!("; Z_HEIGHT: {}", number(layer.z, 3)));
        w.line(&format!("; LAYER_HEIGHT: {}", number(layer.height, 3)));
        if let Some(text) = frame.layers.get(k) {
            w.machine(text);
        }
        let t = done.checked_sub(1).and_then(|i| times.get(i)).copied().unwrap_or(0.0);
        let percent = if total > 0.0 { (100.0 * t / total).floor() as u32 } else { 0 };
        let left = ((total - t) / 60.0).ceil().max(0.0) as u32;
        w.line(&format!("M73 P{percent} R{left}"));
        for m in &layer.moves {
            w.write_move(m);
        }
        done += layer.moves.len();
    }
    w.machine(&frame.end);
    w.line("M73 P100 R0");
    w.line("; EXECUTABLE_BLOCK_END");
    w.out
}

#[cfg(test)]
mod tests;
