//! From ordered beads to the program: travels, retractions, layer changes and templates.

use crate::time::{Limits, estimate};
use crate::{MotionError, MotionParams, Op, Program, RoleMotion, Summary, bead_area};
use layercraft_geom::Point;
use layercraft_machine::{Scope, Value, expand};
use layercraft_path::{Path, PathLayer, Paths, Role};
use layercraft_tune::{Field, centre};
use std::rc::Rc;

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
        p.outer_wall.flow,
        p.inner_wall.flow,
        p.top_surface.flow,
        p.solid.flow,
        p.sparse.flow,
        p.bridge.flow,
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
    let non_negative =
        [p.retract_length, p.retract_min_travel, p.z_hop, p.lift_radius, p.slow_down_layer_time, p.fan_speed];
    let bounded = p.fan_speed <= 1.0 && p.z_hop <= 10.0 && p.lift_radius <= 10.0;
    if !non_negative.iter().all(|v| v.is_finite() && *v >= 0.0) || !bounded {
        return Err(MotionError::BadParams("retraction, hop, lift, cooling out of range".into()));
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
        Role::Bridge => &p.bridge,
    }
}

/// The points a bead runs through: a loop back to its start, less `gap` at its end.
fn path_points(path: &Path, gap: f64) -> Vec<Point> {
    let mut points = path.points.clone();
    let Some(&first) = points.first() else { return points };
    if !path.closed {
        return points;
    }
    points.push(first);
    let mut left = gap;
    while left > 0.0 && points.len() > 2 {
        let (Some(&b), Some(&a)) = (points.last(), points.get(points.len() - 2)) else { break };
        let d = (b[0] - a[0]).hypot(b[1] - a[1]);
        if d > left {
            let t = (d - left) / d;
            if let Some(last) = points.last_mut() {
                *last = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            }
            break;
        }
        points.pop();
        left -= d;
    }
    points
}

/// The cross-section a bead lays: flattened on the layer below, or round when it bridges.
fn section_area(path: &Path, height: f64) -> f64 {
    match path.role {
        Role::Bridge => std::f64::consts::PI * path.width * path.width / 4.0,
        _ => bead_area(path.width, height),
    }
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
    let melt = p.max_volumetric_speed / pushed(p, path, layer.height).max(1e-6);
    base.min(melt)
}

/// The plastic a bead pushes per mm, in mm²: its section times the flow, which the hot end
/// must melt.
fn pushed(p: &MotionParams, path: &Path, height: f64) -> f64 {
    p.flow_ratio * role_motion(p, path.role).flow * section_area(path, height)
}

struct State {
    ops: Vec<Op>,
    at: [f64; 3],
    /// The bead being laid or just laid, with its speed: what a retraction wipes over.
    bead: Vec<[f64; 3]>,
    bead_speed: f64,
    /// The direction of the last move across the plate, a unit vector.
    heading: [f64; 2],
    /// Whether `at` is where the nozzle is: not after the machine's own G-code, which may move it.
    placed: bool,
    /// The filament pulled back, in mm, which the next bead pushes again.
    retracted: f64,
    /// The part cooling fan, when known: not after the machine's own G-code.
    fan: Option<f64>,
    feature: Option<(Role, f64)>,
    accel: Option<f64>,
}

impl State {
    /// The machine's own G-code, after which the nozzle may be anywhere.
    fn machine(&mut self, text: String) {
        self.ops.push(Op::Raw { text });
        self.placed = false;
        self.fan = None;
    }

    /// The direction across the plate from the nozzle to `to`; the last one when it is there.
    fn heading_to(&self, to: Point) -> [f64; 2] {
        let d = [to[0] - self.at[0], to[1] - self.at[1]];
        let l = d[0].hypot(d[1]);
        if l > 1e-9 { [d[0] / l, d[1] / l] } else { self.heading }
    }

    /// Records a move to `to`, and its direction across the plate.
    fn step(&mut self, to: [f64; 3]) {
        self.heading = self.heading_to([to[0], to[1]]);
        self.at = to;
        self.placed = true;
    }

    fn accel(&mut self, value: f64) {
        if self.accel != Some(value) {
            self.ops.push(Op::Accel { value });
            self.accel = Some(value);
        }
    }

    /// Pulls the filament back. After a bead, part of the pull happens while the nozzle moves
    /// back over it for the wipe distance, at the speed that keeps the pull at the retraction
    /// speed; what the wipe cannot take (a short bead) is pulled in place.
    fn retract(&mut self, p: &MotionParams) {
        if self.retracted > 0.0 || p.retract_length <= 0.0 {
            return;
        }
        let total = p.retract_length;
        let before = total * p.retract_before_wipe.clamp(0.0, 1.0);
        if before > 0.0 {
            self.ops.push(Op::Retract { e: -before, speed: p.retract_speed });
        }
        let mut left = total - before;
        let mut wiped = Vec::new();
        let mut budget = p.wipe_distance;
        let mut from = self.at;
        for &to in self.bead.iter().rev().skip(1) {
            // A nanometre of wipe left is rounding: stop rather than emit a move of nothing.
            if budget <= 1e-6 {
                break;
            }
            let d = (to[0] - from[0]).hypot(to[1] - from[1]);
            if d <= 1e-6 {
                continue;
            }
            let t = (budget / d).min(1.0);
            let end = [from[0] + t * (to[0] - from[0]), from[1] + t * (to[1] - from[1]), from[2]];
            wiped.push((end, d * t));
            budget -= d * t;
            from = end;
        }
        let length: f64 = wiped.iter().map(|(_, d)| d).sum();
        if length > 0.0 {
            let pull = left * (length / p.wipe_distance).min(1.0);
            let speed = self.bead_speed.min(p.retract_speed * length / pull);
            for (to, d) in wiped {
                self.ops.push(Op::Wipe { to, e: -pull * d / length, speed });
                self.step(to);
            }
            left -= pull;
        }
        // Below a nanometre of filament, what is left is rounding, not a pull.
        if left > 1e-6 {
            self.ops.push(Op::Retract { e: -left, speed: p.retract_speed });
        }
        self.bead.clear();
        self.retracted = total;
    }

    /// Pushes back exactly what was pulled, at this bead's speed.
    fn deretract(&mut self, p: &MotionParams) {
        if self.retracted > 0.0 {
            self.ops.push(Op::Retract { e: self.retracted, speed: p.deretract_speed });
        }
        self.retracted = 0.0;
    }

    fn fan(&mut self, speed: f64) {
        if self.fan != Some(speed) {
            self.ops.push(Op::Fan { speed });
            self.fan = Some(speed);
        }
    }

    fn travel(&mut self, to: [f64; 3], speed: f64) {
        if to != self.at {
            self.ops.push(Op::Travel { to, speed });
            self.step(to);
        }
    }

    /// Climbs to `z`, as fast as the travel speed and the Z axis allow. Where the nozzle is
    /// known, the climb turns a helix of the lift radius that sets off along `heading`, so it
    /// flows into a travel that way, and comes back over its start; elsewhere it goes straight up.
    fn lift(&mut self, p: &MotionParams, z: f64, heading: [f64; 2]) {
        let dz = z - self.at[2];
        if dz <= 1e-9 {
            return;
        }
        let r = if self.placed { p.lift_radius } else { 0.0 };
        let around = (r > 0.0).then(|| [self.at[0] - r * heading[1], self.at[1] + r * heading[0]]);
        let length = (std::f64::consts::TAU * r).hypot(dz);
        let speed = p.travel.speed.min(p.limits.max_speed_z * length / dz);
        self.ops.push(Op::Lift { z, around, speed });
        self.at[2] = z;
    }

    /// Leaves the bead just laid: pulls the filament back while wiping over the bead, then
    /// climbs the hop off it.
    fn leave(&mut self, p: &MotionParams) {
        self.retract(p);
        if self.placed {
            self.lift(p, self.at[2] + p.z_hop, self.heading);
        }
    }

    /// Moves to `start` on the layer at `z`: short hops stay down, longer ones retract and lift.
    /// Leaving takes the parameters of the bead left, `from`, whose string it breaks; arriving
    /// takes those of the bead reached, `to`.
    fn go_to(&mut self, from: &MotionParams, to: &MotionParams, start: Point, z: f64, first_of_layer: bool) {
        let d = (start[0] - self.at[0]).hypot(start[1] - self.at[1]);
        if first_of_layer || d > from.retract_min_travel {
            self.retract(from);
            self.accel(to.travel.accel);
            // Up first: the travel crosses nothing below the hop height.
            let heading = self.heading_to(start);
            self.lift(from, z + from.z_hop, heading);
            self.travel([start[0], start[1], self.at[2]], to.travel.speed);
            self.travel([start[0], start[1], z], to.travel.speed);
        } else {
            self.travel([start[0], start[1], z], to.travel.speed);
        }
        self.deretract(to);
        self.bead.clear();
        self.bead.push(self.at);
    }
}

fn expand_text(which: &str, template: &str, scope: &Scope) -> Result<String, MotionError> {
    expand(template, scope).map_err(|e| MotionError::Template(which.to_string(), e))
}

/// Grows the box `b` by a bead `r` around its centre point `p`.
fn grow(b: [[f64; 2]; 2], p: Point, r: f64) -> [[f64; 2]; 2] {
    [[b[0][0].min(p[0] - r), b[0][1].min(p[1] - r)], [b[1][0].max(p[0] + r), b[1][1].max(p[1] + r)]]
}

const NO_BOX: [[f64; 2]; 2] = [[f64::MAX, f64::MAX], [f64::MIN, f64::MIN]];

fn or_zero(b: [[f64; 2]; 2]) -> [[f64; 2]; 2] {
    if b[0][0] > b[1][0] { [[0.0, 0.0], [0.0, 0.0]] } else { b }
}

/// Where the first layer lays plastic: its beads' centre lines grown by their half widths.
fn first_layer_bounds(paths: &Paths) -> [[f64; 2]; 2] {
    let beads = paths.layers.first().into_iter().flat_map(|l| l.paths.iter());
    or_zero(beads.fold(NO_BOX, |b, bead| bead.points.iter().fold(b, |b, p| grow(b, *p, bead.width / 2.0))))
}

/// What a program does, read from its moves, and the printer's progress filled in along them:
/// the summary of the program as it stands, after any edit of its moves. Plastic is what the
/// extrusions push, `e` times the filament's section.
pub fn finish(ops: &mut [Op], limits: &Limits, filament_diameter: f64) -> Summary {
    let filament_area = std::f64::consts::PI * (filament_diameter / 2.0).powi(2);
    let times = estimate(ops, limits, [0.0; 3]);
    let total = times.last().copied().unwrap_or(0.0);
    let mut summary = Summary { motion_seconds: total, first_layer_seconds: total, ..Summary::default() };
    let (mut bounds, mut width, mut at) = (NO_BOX, 0.0, [0.0, 0.0]);
    for (i, op) in ops.iter_mut().enumerate() {
        let t = times.get(i).copied().unwrap_or(0.0);
        match op {
            Op::Layer { z, .. } => {
                if summary.layers == 1 {
                    summary.first_layer_seconds = t;
                }
                summary.layers += 1;
                summary.max_z = summary.max_z.max(*z);
                summary.volume_per_layer.push(0.0);
            }
            Op::Feature { width: w, .. } => width = *w,
            Op::Extrude { to, e, .. } => {
                summary.filament_mm += *e;
                if let Some(v) = summary.volume_per_layer.last_mut() {
                    *v += *e * filament_area;
                }
                if summary.layers == 1 {
                    bounds = grow(grow(bounds, at, width / 2.0), [to[0], to[1]], width / 2.0);
                }
            }
            Op::Progress { percent, minutes_left } => {
                *percent = if total > 0.0 { (100.0 * t / total).floor() as u32 } else { 0 };
                *minutes_left = ((total - t) / 60.0).ceil().max(0.0) as u32;
            }
            _ => {}
        }
        if let Op::Travel { to, .. } | Op::Extrude { to, .. } | Op::Wipe { to, .. } = op {
            at = [to[0], to[1]];
        }
    }
    summary.first_layer_bounds = or_zero(bounds);
    summary
}

/// Turns the beads into the printer's program; `scope` holds the machine's settings and
/// the placeholders its templates read. Each bead takes the parameters at its centre, a
/// layer's cooling those at the layer's centre; the program's frame (templates, limits,
/// filament) is the base's.
pub fn compile(paths: &Paths, field: &Field<MotionParams>, scope: &Scope) -> Result<Program, MotionError> {
    let base = field.base();
    check(base)?;
    let filament_area = std::f64::consts::PI * (base.filament_diameter / 2.0).powi(2);
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
    let mut s = State {
        ops: Vec::new(),
        at: [0.0; 3],
        bead: Vec::new(),
        bead_speed: base.travel.speed,
        heading: [1.0, 0.0],
        placed: false,
        retracted: 0.0,
        fan: None,
        feature: None,
        accel: None,
    };
    let t = &base.templates;
    // The markers bound the machine's own sequences, as the printer's firmware expects.
    if !t.limits.is_empty() {
        s.machine(t.limits.clone());
    }
    let start = expand_text("start", &t.start, &scope)?;
    s.machine(format!("{start}\n; MACHINE_START_GCODE_END\n"));
    s.machine(expand_text("filament start", &t.filament_start, &scope)?);
    if !t.prologue.is_empty() {
        s.machine(expand_text("prologue", &t.prologue, &scope)?);
    }
    s.machine("G90\nG21\nM83 ; use relative distances for extrusion\n".into());
    // The parameters of the bead last laid, which leaving it takes.
    let mut last = Rc::new(base.clone());
    for layer in &paths.layers {
        // Off the last bead while still on the layer below, where the nozzle is known.
        s.leave(&last);
        s.ops.push(Op::Layer { index: layer.index, z: layer.top, height: layer.height });
        scope.insert("layer_num".into(), num(layer.index as f64));
        scope.insert("layer_z".into(), num(layer.top));
        scope.insert("max_layer_z".into(), num(layer.top));
        s.machine(expand_text("layer change", &t.layer_change, &scope)?);
        s.ops.push(Op::Progress { percent: 0, minutes_left: 0 });
        let params = layer
            .paths
            .iter()
            .map(|path| field.at(centre(&path.points, layer.top), check))
            .collect::<Result<Vec<_>, _>>()?;
        // Slow the whole layer down when it would print faster than it can cool.
        let cooling = field.at(centre(layer.paths.iter().flat_map(|p| &p.points), layer.top), check)?;
        let busy: f64 = layer
            .paths
            .iter()
            .zip(&params)
            .map(|(path, p)| length(&path_points(path, p.seam_gap)) / bead_speed(p, path, layer))
            .sum();
        let goal = cooling.slow_down_layer_time;
        let slow = if busy > 0.0 && busy < goal { busy / goal } else { 1.0 };
        for (k, (path, p)) in layer.paths.iter().zip(params).enumerate() {
            let points = path_points(path, p.seam_gap);
            let Some(&start) = points.first() else { continue };
            s.fan(if layer.index < p.fan_off_layers { 0.0 } else { p.fan_speed });
            if path.travel_inside && k > 0 {
                // Inside the part from the last bead: straight there, nothing pulled back.
                s.deretract(&p);
                s.travel([start[0], start[1], layer.top], p.travel.speed);
                s.bead.clear();
                s.bead.push(s.at);
            } else {
                s.go_to(&last, &p, start, layer.top, k == 0);
            }
            if s.feature != Some((path.role, path.width)) {
                s.ops.push(Op::Feature { role: path.role, width: path.width });
                s.feature = Some((path.role, path.width));
            }
            let accel = if layer.index == 0 { p.first_layer_accel } else { role_motion(&p, path.role).accel };
            s.accel(accel);
            let full = bead_speed(&p, path, layer);
            let speed = (full * slow).max(p.slow_down_min_speed.min(full));
            let per_mm = pushed(&p, path, layer.height) / filament_area;
            for w in points.windows(2) {
                let [a, b] = [w.first().copied().unwrap_or(start), w.get(1).copied().unwrap_or(start)];
                let l = (b[0] - a[0]).hypot(b[1] - a[1]);
                if l <= 0.0 {
                    continue;
                }
                let e = per_mm * l;
                s.ops.push(Op::Extrude { to: [b[0], b[1], layer.top], e, speed });
                s.step([b[0], b[1], layer.top]);
                s.bead.push(s.at);
                s.bead_speed = speed;
            }
            last = p;
        }
    }
    // Off the part at once, before the machine's own slow end sequence.
    s.leave(&last);
    let heading = s.heading;
    s.lift(&last, max_z + last.z_hop, heading);
    let filament_end = expand_text("filament end", &t.filament_end, &scope)?;
    let end = expand_text("end", &t.end, &scope)?;
    s.machine(format!("; MACHINE_END_GCODE_START\n{filament_end}{end}"));
    let summary = finish(&mut s.ops, &base.limits, base.filament_diameter);
    Ok(Program { ops: s.ops, summary })
}
