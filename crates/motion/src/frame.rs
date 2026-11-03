//! The machine's own G-code around the layers, and what a program does, read from its moves.
//!
//! The frame is the maker's templates expanded for the program as it stands: the start (the
//! machine's limits, its start sequence, the filament's, a prologue, absolute positions and
//! relative extrusion), a layer change before each layer's first pass, a filament change before
//! each pass of another material than the one before (the old filament's end, the change, the
//! new one's start), and the end. The first layer's extent, the highest layer and the filaments
//! come from the moves themselves, so a program merged from several, or edited by hand, gets the
//! frame it needs. A change flushes `flush[from][to]` mm of filament, as the caller says.

use crate::moves::chained;
use crate::time::{Limits, estimate};
use crate::{Layer, MotionError, Program, Summary, Templates, disc_area};
use layercraft_machine::{Scope, Value, expand};

/// The machine's G-code around a program's layers.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub start: String,
    /// One per pass, before it.
    pub layers: Vec<String>,
    pub end: String,
}

/// Item `k` of the list `key` in `scope`, or the value itself when it is no list.
fn item(scope: &Scope, key: &str, k: usize) -> Option<Value> {
    match scope.get(key)? {
        Value::List(items) => items.get(k).cloned(),
        other => Some(other.clone()),
    }
}

/// The first of the settings `(key, item)` that holds a number: a filament's own, then the
/// printer's; zero when none does.
fn first_number(scope: &Scope, keys: &[(&str, usize)]) -> f64 {
    keys.iter().find_map(|(key, k)| item(scope, key, *k).and_then(|v| v.number())).unwrap_or(0.0)
}

/// A filament's feed rate at its melt limit, in mm/min.
fn melt_feed(scope: &Scope, k: usize) -> f64 {
    let speed = item(scope, "filament_max_volumetric_speed", k).and_then(|v| v.number()).unwrap_or(0.0);
    let diameter = item(scope, "filament_diameter", k).and_then(|v| v.number()).unwrap_or(0.0);
    let area = disc_area(diameter);
    if area > 0.0 { speed / area * 60.0 } else { 0.0 }
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

/// Where the first layer lays plastic, all its passes: its beads' paths grown by their half widths.
fn first_layer_bounds(program: &Program) -> [[f64; 2]; 2] {
    let mut b = [[f64::MAX, f64::MAX], [f64::MIN, f64::MIN]];
    let first = program.layers.first().map(|l| l.index);
    for layer in program.layers.iter().take_while(|l| Some(l.index) == first) {
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
    let first_index = program.layers.first().map(|l| l.index);
    // The first layer is all the passes before the second begins.
    let first_moves: usize =
        program.layers.iter().take_while(|l| Some(l.index) == first_index).map(|l| l.moves.len()).sum();
    let first_layer_seconds = if program.layers.iter().any(|l| Some(l.index) != first_index) {
        first_moves.checked_sub(1).and_then(|i| times.get(i)).copied().unwrap_or(0.0)
    } else {
        total
    };
    let pushed =
        |layer: &crate::Layer| layer.moves.iter().filter(|m| m.bead.is_some()).map(|m| m.e).sum::<f64>();
    let mut by_material: Vec<f64> = Vec::new();
    let mut volume_per_layer: Vec<f64> = Vec::new();
    let mut index = None;
    for layer in &program.layers {
        let e = pushed(layer);
        if by_material.len() <= layer.material {
            by_material.resize(layer.material + 1, 0.0);
        }
        if let Some(slot) = by_material.get_mut(layer.material) {
            *slot += e;
        }
        match volume_per_layer.last_mut() {
            Some(last) if index == Some(layer.index) => *last += e * filament_area,
            _ => volume_per_layer.push(e * filament_area),
        }
        index = Some(layer.index);
    }
    Summary {
        layers: volume_per_layer.len(),
        max_z: program.layers.iter().map(Layer::highest).fold(0.0, f64::max),
        filament_mm: by_material.iter().sum(),
        filament_mm_by_material: by_material,
        volume_per_layer,
        motion_seconds: total,
        first_layer_seconds,
        first_layer_bounds: first_layer_bounds(program),
    }
}

/// The machine's G-code around `program`: `templates` expanded with `scope`, the machine's
/// settings and the slicer's placeholders, and what the program itself tells.
pub fn frame(
    program: &Program,
    templates: &Templates,
    scope: &Scope,
    flush: &[Vec<f64>],
) -> Result<Frame, MotionError> {
    let mut scope = scope.clone();
    let num = Value::Num;
    let pair = |a: f64, b: f64| Value::List(vec![num(a), num(b)]);
    let bounds = first_layer_bounds(program);
    let max_z = program.layers.iter().map(Layer::highest).fold(0.0, f64::max);
    scope.insert("first_layer_print_min".into(), pair(bounds[0][0], bounds[0][1]));
    scope.insert(
        "first_layer_print_size".into(),
        pair(bounds[1][0] - bounds[0][0], bounds[1][1] - bounds[0][1]),
    );
    scope.insert("max_print_z".into(), num(max_z));
    scope.insert("max_layer_z".into(), num(0.0));
    // The first pass's filament is loaded at the start.
    let first = program.layers.first().map_or(0, |l| l.material);
    for key in ["initial_no_support_filament_id", "initial_filament_id", "current_filament_id"] {
        scope.insert(key.into(), num(first as f64));
    }
    for key in ["first_filaments", "first_non_support_filaments"] {
        scope.insert(key.into(), pair(first as f64, -1.0));
    }
    let materials = program.layers.iter().map(|l| l.material).max().map_or(1, |m| m + 1);
    if materials > 1 && templates.change_filament.trim().is_empty() {
        return Err(MotionError::BadParams(
            "the machine has no filament change to print several materials".into(),
        ));
    }
    if materials > 1
        && (flush.len() < materials || flush.iter().take(materials).any(|row| row.len() < materials))
    {
        return Err(MotionError::BadParams(format!("no flush given between the {materials} materials")));
    }
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
    // The highest any layer so far reaches, a bent one above its plane.
    let mut highest = 0.0f64;
    let (mut index, mut material, mut changes) = (None, first, 0usize);
    for layer in &program.layers {
        let mut text = String::new();
        if index != Some(layer.index) {
            index = Some(layer.index);
            highest = highest.max(layer.highest());
            scope.insert("layer_num".into(), num(layer.index as f64));
            scope.insert("layer_z".into(), num(layer.highest()));
            scope.insert("max_layer_z".into(), num(highest));
            text += &lines(&expand_text("layer change", &t.layer_change, &scope)?);
        }
        if layer.material != material {
            let (from, to) = (material, layer.material);
            changes += 1;
            let length = flush.get(from).and_then(|row| row.get(to)).copied().unwrap_or(0.0);
            if !(length.is_finite() && length >= 0.0) {
                return Err(MotionError::BadParams(format!(
                    "flush of {length} mm from material {from} to {to}"
                )));
            }
            // Where the pass starts, which the machine's change may travel back to.
            let at = layer.moves.iter().find_map(|m| m.to).unwrap_or([0.0, 0.0, layer.z]);
            let temperature = |k: usize| item(&scope, "nozzle_temperature", k).unwrap_or(num(0.0));
            let change = [
                ("current_filament_id", num(from as f64)),
                ("next_filament_id", num(to as f64)),
                ("current_nozzle_id", num(0.0)),
                ("next_nozzle_id", num(0.0)),
                ("current_hotend", num(-1.0)),
                ("next_hotend", num(-1.0)),
                ("flush_length", num(length)),
                ("old_filament_temp", temperature(from)),
                ("new_filament_temp", temperature(to)),
                ("old_filament_e_feedrate", num(melt_feed(&scope, from))),
                ("new_filament_e_feedrate", num(melt_feed(&scope, to))),
                // The start's load counts as the first.
                ("toolchange_count", num((changes + 1) as f64)),
                ("x_after_toolchange", num(at[0])),
                ("y_after_toolchange", num(at[1])),
                ("z_after_toolchange", num(at[2])),
                ("is_prime_tower_interface", Value::Bool(false)),
                // How the old filament is pulled back when the change cuts it, and when it
                // leaves the extruder: the filament's own setting, else the printer's.
                (
                    "long_retraction_when_cut",
                    Value::Bool(
                        first_number(
                            &scope,
                            &[("filament_long_retractions_when_cut", from), ("long_retractions_when_cut", 0)],
                        ) > 0.0,
                    ),
                ),
                (
                    "retraction_distance_when_cut",
                    num(first_number(
                        &scope,
                        &[
                            ("filament_retraction_distances_when_cut", from),
                            ("retraction_distances_when_cut", 0),
                        ],
                    )),
                ),
                (
                    "long_retraction_when_ec",
                    Value::Bool(
                        first_number(
                            &scope,
                            &[
                                ("filament_long_retractions_when_ec", from),
                                ("long_retractions_when_ec", from),
                            ],
                        ) > 0.0,
                    ),
                ),
                (
                    "retraction_distance_when_ec",
                    num(first_number(
                        &scope,
                        &[
                            ("filament_retraction_distances_when_ec", from),
                            ("retraction_distances_when_ec", from),
                        ],
                    )),
                ),
                // The way the head takes to the change, which the X2D's template only names in a
                // comment: straight from the pass's start.
                ("travel_point_1_x", num(at[0])),
                ("travel_point_1_y", num(at[1])),
                ("travel_point_2_x", num(at[0])),
                ("travel_point_2_y", num(at[1])),
                ("travel_point_3_x", num(at[0])),
                ("travel_point_3_y", num(at[1])),
            ];
            scope.insert("current_filament_id".into(), num(from as f64));
            text += &lines(&expand_text("filament end", &t.filament_end, &scope)?);
            for (key, value) in change {
                scope.insert(key.into(), value);
            }
            text += &lines(&expand_text("filament change", &t.change_filament, &scope)?);
            scope.insert("current_filament_id".into(), num(to as f64));
            text += &lines(&expand_text("filament start", &t.filament_start, &scope)?);
            material = to;
        }
        layers.push(text);
    }
    let end = lines(&format!(
        "; MACHINE_END_GCODE_START\n{}{}",
        expand_text("filament end", &t.filament_end, &scope)?,
        expand_text("end", &t.end, &scope)?
    ));
    Ok(Frame { start, layers, end })
}
