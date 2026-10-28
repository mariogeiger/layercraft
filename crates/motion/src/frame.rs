//! The machine's own G-code around the layers, and what a program does, read from its moves.
//!
//! The frame is the maker's templates expanded for the program as it stands: the start (the
//! machine's limits, its start sequence, the filament's, a prologue, absolute positions and
//! relative extrusion), one layer change before each layer, and the end. The first layer's
//! extent and the highest layer come from the moves themselves, so a program merged from
//! several, or edited by hand, gets the frame it needs.

use crate::moves::chained;
use crate::time::{Limits, estimate};
use crate::{MotionError, Program, Summary, Templates, disc_area};
use layercraft_machine::{Scope, Value, expand};

/// The machine's G-code around a program's layers.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub start: String,
    /// One per layer, before it.
    pub layers: Vec<String>,
    pub end: String,
}

fn expand_text(which: &str, template: &str, scope: &Scope) -> Result<String, MotionError> {
    let text = expand(template, scope).map_err(|e| MotionError::Template(which.to_string(), e))?;
    if text.contains('\0') {
        return Err(MotionError::BadParams(format!("the machine's {which} G-code holds a NUL byte")));
    }
    Ok(text)
}

/// `text` ending with a line break.
fn lines(text: &str) -> String {
    if text.ends_with('\n') { text.to_string() } else { format!("{text}\n") }
}

/// Grows the box `b` by a bead `r` around its centre point `p`.
fn grow(b: [[f64; 2]; 2], p: [f64; 3], r: f64) -> [[f64; 2]; 2] {
    [[b[0][0].min(p[0] - r), b[0][1].min(p[1] - r)], [b[1][0].max(p[0] + r), b[1][1].max(p[1] + r)]]
}

/// Where the first layer lays plastic: its beads' paths grown by their half widths.
fn first_layer_bounds(program: &Program) -> [[f64; 2]; 2] {
    let mut b = [[f64::MAX, f64::MAX], [f64::MIN, f64::MIN]];
    if let Some(layer) = program.layers.first() {
        for (from, m) in chained(&layer.moves, None) {
            let (Some(from), Some(bead)) = (from, m.bead) else { continue };
            let swept = m.bounds(from);
            let r = bead.width / 2.0;
            b = grow(grow(b, [swept[0][0], swept[0][1], 0.0], r), [swept[1][0], swept[1][1], 0.0], r);
        }
    }
    if b[0][0] > b[1][0] { [[0.0, 0.0], [0.0, 0.0]] } else { b }
}

/// What a program does, read from its moves: plastic is what the extrusions push, `e` times
/// the filament's section.
pub fn finish(program: &Program, limits: &Limits, filament_diameter: f64) -> Summary {
    let filament_area = disc_area(filament_diameter);
    let times = estimate(program, limits);
    let total = times.last().copied().unwrap_or(0.0);
    let first_moves = program.layers.first().map_or(0, |l| l.moves.len());
    let first_layer_seconds = if program.layers.len() > 1 {
        first_moves.checked_sub(1).and_then(|i| times.get(i)).copied().unwrap_or(0.0)
    } else {
        total
    };
    let pushed =
        |layer: &crate::Layer| layer.moves.iter().filter(|m| m.bead.is_some()).map(|m| m.e).sum::<f64>();
    Summary {
        layers: program.layers.len(),
        max_z: program.layers.iter().map(|l| l.z).fold(0.0, f64::max),
        filament_mm: program.layers.iter().map(pushed).sum(),
        volume_per_layer: program.layers.iter().map(|l| pushed(l) * filament_area).collect(),
        motion_seconds: total,
        first_layer_seconds,
        first_layer_bounds: first_layer_bounds(program),
    }
}

/// The machine's G-code around `program`: `templates` expanded with `scope`, the machine's
/// settings and the slicer's placeholders, and what the program itself tells.
pub fn frame(program: &Program, templates: &Templates, scope: &Scope) -> Result<Frame, MotionError> {
    let mut scope = scope.clone();
    let num = Value::Num;
    let pair = |a: f64, b: f64| Value::List(vec![num(a), num(b)]);
    let bounds = first_layer_bounds(program);
    let max_z = program.layers.iter().map(|l| l.z).fold(0.0, f64::max);
    scope.insert("first_layer_print_min".into(), pair(bounds[0][0], bounds[0][1]));
    scope.insert(
        "first_layer_print_size".into(),
        pair(bounds[1][0] - bounds[0][0], bounds[1][1] - bounds[0][1]),
    );
    scope.insert("max_print_z".into(), num(max_z));
    scope.insert("max_layer_z".into(), num(0.0));
    let t = templates;
    let mut pieces = Vec::new();
    if !t.limits.is_empty() {
        pieces.push(t.limits.clone());
    }
    pieces.push(format!("{}\n; MACHINE_START_GCODE_END\n", expand_text("start", &t.start, &scope)?));
    pieces.push(expand_text("filament start", &t.filament_start, &scope)?);
    if !t.prologue.is_empty() {
        pieces.push(expand_text("prologue", &t.prologue, &scope)?);
    }
    pieces.push("G90\nG21\nM83 ; use relative distances for extrusion\n".into());
    let start = pieces.iter().map(|p| lines(p)).collect();
    let mut layers = Vec::with_capacity(program.layers.len());
    for layer in &program.layers {
        scope.insert("layer_num".into(), num(layer.index as f64));
        scope.insert("layer_z".into(), num(layer.z));
        scope.insert("max_layer_z".into(), num(layer.z));
        layers.push(lines(&expand_text("layer change", &t.layer_change, &scope)?));
    }
    let end = lines(&format!(
        "; MACHINE_END_GCODE_START\n{}{}",
        expand_text("filament end", &t.filament_end, &scope)?,
        expand_text("end", &t.end, &scope)?
    ));
    Ok(Frame { start, layers, end })
}
