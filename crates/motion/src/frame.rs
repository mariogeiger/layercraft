//! The machine's own G-code around the passes: the printer's templates expanded for the program as
//! it stands, the start (the machine's limits, its start sequence, a prologue, absolute positions
//! and relative extrusion), a layer change before each layer's first pass, a filament change before
//! each pass of another material than the one before, and the end. The first layer's extent, the
//! highest layer and the filaments come from the moves themselves, so a program merged from
//! several, or edited by hand, gets the frame it needs; the settings, each filament's among them,
//! come in the scope.
//!
//! Between the templates the frame holds the temperatures (`Heat`): the start leaves the first
//! filament and the bed at their first layer's, so a layer that needs another sets it on its way
//! up (`M104`, `M140`), the nozzle following the filament loaded; and a change leaves the nozzle
//! at whatever its cut cooled it to, so the pass after it waits for its own (`M109`).

use crate::summary::first_layer_bounds;
use crate::{MotionError, Program, Templates};
use layercraft_template::{Scope, Value, expand};
use serde::{Deserialize, Serialize};

/// The machine's G-code around a program's passes.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub start: String,
    /// One per pass, before it.
    pub passes: Vec<String>,
    pub end: String,
}

/// The temperatures a print holds, in °C, which the slicer sets between the machine's
/// templates: the start leaves the first layer's, and a filament change leaves the nozzle at no
/// set temperature.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Heat {
    /// Each material's nozzle temperature on the first layer, then on the others.
    pub nozzle: Vec<[f64; 2]>,
    /// The bed's temperature on the first layer, then on the others.
    pub bed: [f64; 2],
}

impl Heat {
    /// Material `k`'s nozzle temperature on layer `index`.
    pub fn nozzle_at(&self, k: usize, index: usize) -> Result<f64, MotionError> {
        let pair = self
            .nozzle
            .get(k)
            .ok_or_else(|| MotionError::BadParams(format!("no temperature for material {k}")))?;
        on_layer(*pair, index, "nozzle")
    }

    /// The bed's temperature on layer `index`.
    pub fn bed_at(&self, index: usize) -> Result<f64, MotionError> {
        on_layer(self.bed, index, "bed")
    }
}

/// The first layer's temperature or the others', checked: one a printer can be asked for,
/// finite and above zero.
fn on_layer([first, rest]: [f64; 2], index: usize, what: &str) -> Result<f64, MotionError> {
    let t = if index == 0 { first } else { rest };
    if t.is_finite() && t > 0.0 {
        Ok(t)
    } else {
        Err(MotionError::BadParams(format!("a {what} temperature of {t} °C")))
    }
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

/// The machine's G-code around `program`: `templates` expanded with `scope`, the settings, and
/// what the program itself tells, at the temperatures `heat` gives.
pub fn frame(
    program: &Program,
    templates: &Templates,
    heat: &Heat,
    scope: &Scope,
) -> Result<Frame, MotionError> {
    let mut scope = scope.clone();
    let num = Value::Num;
    let pair = |a: f64, b: f64| Value::List(vec![num(a), num(b)]);
    let bounds = first_layer_bounds(program);
    let max_z = program.highest();
    scope.insert("first_layer_print_min".into(), pair(bounds[0][0], bounds[0][1]));
    scope.insert(
        "first_layer_print_size".into(),
        pair(bounds[1][0] - bounds[0][0], bounds[1][1] - bounds[0][1]),
    );
    scope.insert("max_print_z".into(), num(max_z));
    scope.insert("max_layer_z".into(), num(0.0));
    // The first pass's filament is loaded at the start.
    let first = program.passes.first().map_or(0, |p| p.material);
    for key in ["initial_filament_id", "current_filament_id"] {
        scope.insert(key.into(), num(first as f64));
    }
    if program.materials() > 1 && templates.change_filament.trim().is_empty() {
        return Err(MotionError::BadParams(
            "the machine has no filament change to print several materials".into(),
        ));
    }
    let t = templates;
    let mut pieces = Vec::new();
    if !t.limits.is_empty() {
        pieces.push(t.limits.clone());
    }
    pieces.push(format!("{}\n; MACHINE_START_GCODE_END\n", expand_text("start", &t.start, &scope)?));
    if !t.prologue.is_empty() {
        pieces.push(expand_text("prologue", &t.prologue, &scope)?);
    }
    pieces.push("G90\nG21\nM83 ; use relative distances for extrusion\n".into());
    let start = pieces.iter().map(|p| lines(p)).collect();
    let mut passes = Vec::with_capacity(program.passes.len());
    // The highest any layer so far reaches, a bent one above its plane.
    let mut highest = 0.0f64;
    let (mut index, mut material) = (None, first);
    // What the nozzle and the bed are set to: the start brings both to the first layer's.
    let (mut nozzle, mut bed) = (heat.nozzle_at(first, 0)?, heat.bed_at(0)?);
    for pass in &program.passes {
        let mut text = String::new();
        if index != Some(pass.index) {
            index = Some(pass.index);
            highest = highest.max(pass.highest());
            scope.insert("layer_num".into(), num(pass.index as f64));
            scope.insert("layer_z".into(), num(pass.highest()));
            scope.insert("max_layer_z".into(), num(highest));
            text += &lines(&expand_text("layer change", &t.layer_change, &scope)?);
            let wanted = heat.bed_at(pass.index)?;
            if wanted != bed {
                bed = wanted;
                text += &format!("M140 S{bed} ; the bed for this layer\n");
            }
            // A pass of another filament sets its own after the change.
            let wanted = heat.nozzle_at(material, pass.index)?;
            if pass.material == material && wanted != nozzle {
                nozzle = wanted;
                text += &format!("M104 S{nozzle} ; the nozzle for this layer\n");
            }
        }
        if pass.material != material {
            let (from, to) = (material, pass.material);
            let (old_temp, new_temp) = (heat.nozzle_at(from, pass.index)?, heat.nozzle_at(to, pass.index)?);
            let change = [
                ("current_filament_id", num(from as f64)),
                ("next_filament_id", num(to as f64)),
                ("old_filament_temp", num(old_temp)),
                ("new_filament_temp", num(new_temp)),
            ];
            for (key, value) in change {
                scope.insert(key.into(), value);
            }
            text += &lines(&expand_text("filament change", &t.change_filament, &scope)?);
            scope.insert("current_filament_id".into(), num(to as f64));
            material = to;
            // Its first pass lies on plastic the change left to cool: as on the plate, at the
            // first layer's temperature, back to its own on the next layer.
            nozzle = heat.nozzle_at(to, 0)?;
            text += &format!("M109 S{nozzle} ; wait for the new filament, as hot as on the first layer\n");
        }
        passes.push(text);
    }
    let end = lines(&format!("; MACHINE_END_GCODE_START\n{}", expand_text("end", &t.end, &scope)?));
    Ok(Frame { start, passes, end })
}
