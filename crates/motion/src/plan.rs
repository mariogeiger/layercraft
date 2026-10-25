//! From ordered beads to the program: travels, retractions, layer changes and templates.

use crate::time::estimate;
use crate::{MotionError, MotionParams, Op, Program, RoleMotion, Summary, bead_area};
use layercraft_geom::Point;
use layercraft_machine::{Scope, Value, expand};
use layercraft_path::{Path, PathLayer, Paths, Role};

/// The variables a Bambu Lab printer's templates expect from the slicer for a single-filament
/// print on the first extruder: which filament and nozzle are first, the plate and its
/// first-layer temperature.
pub fn placeholders(bed_type: &str, bed_first_layer_temp: f64, nozzle_diameter: f64) -> Scope {
    let num = Value::Num;
    let list = |v: Vec<Value>| Value::List(v);
    [
        ("initial_no_support_filament_id", num(0.0)),
        ("initial_filament_id", num(0.0)),
        ("initial_nozzle_id", num(0.0)),
        ("initial_no_support_hotend", num(-1.0)),
        ("current_filament_id", num(0.0)),
        ("current_nozzle_id", num(0.0)),
        ("filament_map", list(vec![num(1.0)])),
        ("first_filaments", list(vec![num(0.0), num(-1.0)])),
        ("first_non_support_filaments", list(vec![num(0.0), num(-1.0)])),
        ("nozzle_diameter_at_nozzle_id", list(vec![num(nozzle_diameter)])),
        ("curr_bed_type", Value::Str(bed_type.to_string())),
        ("bed_temperature_initial_layer_single", num(bed_first_layer_temp)),
        ("overall_chamber_temperature", num(0.0)),
        ("hold_chamber_temp_for_flat_print", Value::Bool(false)),
        ("is_all_bbl_filament", Value::Bool(true)),
        ("toolchange_count", num(0.0)),
        ("spiral_mode", Value::Bool(false)),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect()
}

fn check(p: &MotionParams) -> Result<(), MotionError> {
    let positive = [
        p.filament_diameter,
        p.flow_ratio,
        p.max_volumetric_speed,
        p.outer_wall.speed,
        p.inner_wall.speed,
        p.top_surface.speed,
        p.solid.speed,
        p.sparse.speed,
        p.first_layer_wall_speed,
        p.first_layer_infill_speed,
        p.travel.speed,
        p.retract_speed,
        p.deretract_speed,
        p.slow_down_min_speed,
        p.limits.max_accel,
        p.limits.max_speed_xy,
        p.limits.max_speed_z,
    ];
    if !positive.iter().all(|v| v.is_finite() && *v > 0.0) {
        return Err(MotionError::BadParams("speeds, sizes and limits must be positive".into()));
    }
    let non_negative = [p.retract_length, p.retract_min_travel, p.z_hop, p.slow_down_layer_time, p.fan_speed];
    if !non_negative.iter().all(|v| v.is_finite() && *v >= 0.0) || p.fan_speed > 1.0 || p.z_hop > 10.0 {
        return Err(MotionError::BadParams("retraction, hop, cooling out of range".into()));
    }
    Ok(())
}

fn role_motion(p: &MotionParams, role: Role) -> &RoleMotion {
    match role {
        Role::OuterWall => &p.outer_wall,
        Role::InnerWall => &p.inner_wall,
        Role::TopSurface => &p.top_surface,
        Role::Solid => &p.solid,
        Role::Sparse => &p.sparse,
    }
}

fn path_points(path: &Path) -> Vec<Point> {
    let mut points = path.points.clone();
    if path.closed
        && let Some(&first) = points.first()
    {
        points.push(first);
    }
    points
}

fn length(points: &[Point]) -> f64 {
    layercraft_geom::polyline_length(points)
}

/// The speed of a bead: by role, the first layer's own, never past the melt limit.
fn bead_speed(p: &MotionParams, path: &Path, layer: &PathLayer) -> f64 {
    let base = if layer.index == 0 {
        match path.role {
            Role::OuterWall | Role::InnerWall => p.first_layer_wall_speed,
            _ => p.first_layer_infill_speed,
        }
    } else {
        role_motion(p, path.role).speed
    };
    let melt = p.max_volumetric_speed / bead_area(path.width, layer.height).max(1e-6);
    base.min(melt)
}

struct State {
    ops: Vec<Op>,
    at: [f64; 3],
    retracted: bool,
    feature: Option<(Role, f64)>,
    accel: Option<f64>,
    filament: f64,
}

impl State {
    fn accel(&mut self, value: f64) {
        if self.accel != Some(value) {
            self.ops.push(Op::Accel { value });
            self.accel = Some(value);
        }
    }

    fn retract(&mut self, p: &MotionParams) {
        if !self.retracted && p.retract_length > 0.0 {
            self.ops.push(Op::Retract { e: -p.retract_length, speed: p.retract_speed });
        }
        self.retracted = true;
    }

    fn deretract(&mut self, p: &MotionParams) {
        if self.retracted && p.retract_length > 0.0 {
            self.ops.push(Op::Retract { e: p.retract_length, speed: p.deretract_speed });
        }
        self.retracted = false;
    }

    fn travel(&mut self, to: [f64; 3], speed: f64) {
        if to != self.at {
            self.ops.push(Op::Travel { to, speed });
            self.at = to;
        }
    }

    /// Moves to `start` on the layer at `z`: short hops stay down, longer ones retract and lift.
    fn go_to(&mut self, p: &MotionParams, start: Point, z: f64, first_of_layer: bool) {
        let d = (start[0] - self.at[0]).hypot(start[1] - self.at[1]);
        if first_of_layer || d > p.retract_min_travel {
            self.retract(p);
            self.accel(p.travel.accel);
            // Straight up first: the nozzle never moves sideways below the hop height.
            let lift = z + p.z_hop;
            if self.at[2] < lift {
                self.ops.push(Op::Lift { z: lift, speed: p.limits.max_speed_z });
                self.at[2] = lift;
            }
            self.travel([start[0], start[1], self.at[2]], p.travel.speed);
            self.travel([start[0], start[1], z], p.travel.speed);
        } else {
            self.travel([start[0], start[1], z], p.travel.speed);
        }
        self.deretract(p);
    }
}

fn expand_named(which: &str, template: &str, scope: &Scope) -> Result<Op, MotionError> {
    Ok(Op::Raw { text: expand_text(which, template, scope)? })
}

fn expand_text(which: &str, template: &str, scope: &Scope) -> Result<String, MotionError> {
    expand(template, scope).map_err(|e| MotionError::Template(which.to_string(), e))
}

/// Where the first layer lays plastic: its beads' centre lines grown by their half widths.
fn first_layer_bounds(paths: &Paths) -> [[f64; 2]; 2] {
    let beads = paths.layers.first().into_iter().flat_map(|l| l.paths.iter());
    let mut b = [[f64::MAX, f64::MAX], [f64::MIN, f64::MIN]];
    for bead in beads {
        let r = bead.width / 2.0;
        for p in &bead.points {
            b = [
                [b[0][0].min(p[0] - r), b[0][1].min(p[1] - r)],
                [b[1][0].max(p[0] + r), b[1][1].max(p[1] + r)],
            ];
        }
    }
    if b[0][0] > b[1][0] { [[0.0, 0.0], [0.0, 0.0]] } else { b }
}

/// Turns the beads into the printer's program; `scope` holds the machine's settings and
/// the placeholders its templates read.
pub fn compile(paths: &Paths, p: &MotionParams, scope: &Scope) -> Result<Program, MotionError> {
    check(p)?;
    let filament_area = std::f64::consts::PI * (p.filament_diameter / 2.0).powi(2);
    let max_z = paths.layers.last().map(|l| l.top).unwrap_or(0.0);
    let bounds = first_layer_bounds(paths);
    let mut scope = scope.clone();
    let num = Value::Num;
    let pair = |a: f64, b: f64| Value::List(vec![num(a), num(b)]);
    scope.insert("first_layer_print_min".into(), pair(bounds[0][0], bounds[0][1]));
    scope.insert(
        "first_layer_print_size".into(),
        pair(bounds[1][0] - bounds[0][0], bounds[1][1] - bounds[0][1]),
    );
    scope.insert("max_print_z".into(), num(max_z));
    scope.insert("max_layer_z".into(), num(0.0));
    let mut s =
        State { ops: Vec::new(), at: [0.0; 3], retracted: false, feature: None, accel: None, filament: 0.0 };
    // The markers bound the machine's own sequences, as the printer's firmware expects.
    if !p.templates.limits.is_empty() {
        s.ops.push(Op::Raw { text: p.templates.limits.clone() });
    }
    let start = expand_text("start", &p.templates.start, &scope)?;
    s.ops.push(Op::Raw { text: format!("{start}\n; MACHINE_START_GCODE_END\n") });
    s.ops.push(expand_named("filament start", &p.templates.filament_start, &scope)?);
    if !p.templates.prologue.is_empty() {
        s.ops.push(expand_named("prologue", &p.templates.prologue, &scope)?);
    }
    s.ops.push(Op::Raw { text: "G90\nG21\nM83 ; use relative distances for extrusion\n".into() });
    let mut volume_per_layer = Vec::with_capacity(paths.layers.len());
    for layer in &paths.layers {
        s.ops.push(Op::Layer { index: layer.index, z: layer.top, height: layer.height });
        s.retract(p);
        scope.insert("layer_num".into(), num(layer.index as f64));
        scope.insert("layer_z".into(), num(layer.top));
        scope.insert("max_layer_z".into(), num(layer.top));
        s.ops.push(expand_named("layer change", &p.templates.layer_change, &scope)?);
        s.ops.push(Op::Progress { percent: 0, minutes_left: 0 });
        s.ops.push(Op::Fan { speed: if layer.index < p.fan_off_layers { 0.0 } else { p.fan_speed } });
        // Slow the whole layer down when it would print faster than it can cool.
        let busy: f64 =
            layer.paths.iter().map(|path| length(&path_points(path)) / bead_speed(p, path, layer)).sum();
        let slow =
            if busy > 0.0 && busy < p.slow_down_layer_time { busy / p.slow_down_layer_time } else { 1.0 };
        let mut volume = 0.0;
        for (k, path) in layer.paths.iter().enumerate() {
            let points = path_points(path);
            let Some(&start) = points.first() else { continue };
            s.go_to(p, start, layer.top, k == 0);
            if s.feature != Some((path.role, path.width)) {
                s.ops.push(Op::Feature { role: path.role, width: path.width });
                s.feature = Some((path.role, path.width));
            }
            let accel = if layer.index == 0 { p.first_layer_accel } else { role_motion(p, path.role).accel };
            s.accel(accel);
            let base = bead_speed(p, path, layer);
            let speed = (base * slow).max(p.slow_down_min_speed.min(base));
            let area = bead_area(path.width, layer.height);
            for w in points.windows(2) {
                let [a, b] = [w.first().copied().unwrap_or(start), w.get(1).copied().unwrap_or(start)];
                let l = (b[0] - a[0]).hypot(b[1] - a[1]);
                if l <= 0.0 {
                    continue;
                }
                let e = p.flow_ratio * area * l / filament_area;
                s.ops.push(Op::Extrude { to: [b[0], b[1], layer.top], e, speed });
                s.at = [b[0], b[1], layer.top];
                s.filament += e;
                volume += area * l;
            }
        }
        volume_per_layer.push(volume);
    }
    s.retract(p);
    let filament_end = expand_text("filament end", &p.templates.filament_end, &scope)?;
    let end = expand_text("end", &p.templates.end, &scope)?;
    s.ops.push(Op::Raw { text: format!("; MACHINE_END_GCODE_START\n{filament_end}{end}") });
    let times = estimate(&s.ops, &p.limits, [0.0; 3]);
    let total = times.last().copied().unwrap_or(0.0);
    for (i, op) in s.ops.iter_mut().enumerate() {
        if let Op::Progress { percent, minutes_left } = op {
            let t = times.get(i).copied().unwrap_or(0.0);
            *percent = if total > 0.0 { (100.0 * t / total).floor() as u32 } else { 0 };
            *minutes_left = ((total - t) / 60.0).ceil().max(0.0) as u32;
        }
    }
    let second_layer =
        s.ops.iter().enumerate().filter(|(_, op)| matches!(op, Op::Layer { .. })).nth(1).map(|(i, _)| i);
    let first_layer_seconds = second_layer.and_then(|i| times.get(i).copied()).unwrap_or(total);
    let summary = Summary {
        layers: paths.layers.len(),
        max_z,
        filament_mm: s.filament,
        volume_per_layer,
        motion_seconds: total,
        first_layer_seconds,
        first_layer_bounds: bounds,
    };
    Ok(Program { ops: s.ops, summary })
}
