//! Stage 6: the program as G-code text, in the block layout Bambu Lab printers read.
//!
//! The text has a header block (time, layers, filament), a configuration block (every setting,
//! one per line) and the executable block. Positions are absolute and extrusion relative
//! (`M83`); a move names only the axes that change and a feed rate only when it changes.
//! `stats` reads G-code back, from layercraft or another slicer, to compare them.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod stats;

use layercraft_motion::{Op, Program};
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
    match role {
        Role::OuterWall => "Outer wall",
        Role::InnerWall => "Inner wall",
        Role::TopSurface => "Top surface",
        Role::Solid => "Internal solid infill",
        Role::Sparse => "Sparse infill",
    }
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
}

impl Writer {
    fn line(&mut self, text: &str) {
        self.out.push_str(text);
        self.out.push('\n');
    }

    fn feed(&mut self, speed: f64, words: &mut String) {
        let f = (speed * 60.0).round();
        if self.feed != Some(f) {
            let _ = write!(words, " F{}", number(f, 0));
            self.feed = Some(f);
        }
    }

    fn mov(&mut self, to: [Option<f64>; 3], e: Option<f64>, speed: f64) {
        let mut words = String::from("G1");
        for (axis, (name, value)) in ["X", "Y", "Z"].into_iter().zip(to).enumerate() {
            let Some(v) = value else { continue };
            let rounded: f64 = number(v, 3).parse().unwrap_or(v);
            if self.at.get(axis).copied().flatten() != Some(rounded) {
                let _ = write!(words, " {name}{}", number(v, 3));
                if let Some(slot) = self.at.get_mut(axis) {
                    *slot = Some(rounded);
                }
            }
        }
        if let Some(e) = e {
            let _ = write!(words, " E{}", number(e, 5));
        }
        if words == "G1" {
            return;
        }
        self.feed(speed, &mut words);
        self.line(&words);
    }
}

/// Writes the program as G-code.
pub fn write(program: &Program, header: &Header) -> String {
    let s = &program.summary;
    let area = std::f64::consts::PI * (header.filament_diameter / 2.0).powi(2);
    let grams = s.filament_mm * area * header.filament_density / 1000.0;
    let mut w = Writer { out: String::new(), at: [None; 3], feed: None };
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
    for op in &program.ops {
        match op {
            Op::Raw { text } => {
                w.out.push_str(text);
                if !text.ends_with('\n') {
                    w.out.push('\n');
                }
                // The machine's own code may move anywhere: forget where we are.
                w.at = [None; 3];
                w.feed = None;
            }
            Op::Layer { z, height, .. } => {
                w.line("; CHANGE_LAYER");
                w.line(&format!("; Z_HEIGHT: {}", number(*z, 3)));
                w.line(&format!("; LAYER_HEIGHT: {}", number(*height, 3)));
            }
            Op::Feature { role, width } => {
                w.line(&format!("; FEATURE: {}", feature_name(*role)));
                w.line(&format!("; LINE_WIDTH: {}", number(*width, 3)));
            }
            Op::Travel { to, speed } => w.mov(to.map(Some), None, *speed),
            Op::Extrude { to, e, speed } => w.mov(to.map(Some), Some(*e), *speed),
            Op::Lift { z, speed } => w.mov([None, None, Some(*z)], None, *speed),
            Op::Retract { e, speed } => {
                let mut words = format!("G1 E{}", number(*e, 5));
                w.feed(*speed, &mut words);
                w.line(&words);
            }
            Op::Fan { speed } => w.line(&format!("M106 S{}", number((speed * 255.0).round(), 0))),
            Op::Accel { value } => w.line(&format!("M204 S{}", number(*value, 0))),
            Op::Progress { percent, minutes_left } => w.line(&format!("M73 P{percent} R{minutes_left}")),
        }
    }
    w.line("M73 P100 R0");
    w.line("; EXECUTABLE_BLOCK_END");
    w.out
}

#[cfg(test)]
mod tests;
