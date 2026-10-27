//! From ordered beads to the program: travels, retractions, wipes and lifts, layer by layer.

use crate::{Arc, Bead, Layer, MotionError, MotionParams, Move, Program, RoleMotion, bead_area};
use layercraft_geom::Point;
use layercraft_machine::{Scope, Value};
use layercraft_path::{Kind, Path, PathLayer, Paths, Role};
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

pub(crate) fn check(p: &MotionParams) -> Result<(), MotionError> {
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
    if !p.overhang_speeds.iter().all(|v| v.is_finite() && *v >= 0.0) || p.overhang_speeds.len() > 100 {
        return Err(MotionError::BadParams("overhang speeds must be finite, not negative, and few".into()));
    }
    let non_negative =
        [p.retract_length, p.retract_min_travel, p.z_hop, p.lift_radius, p.slow_down_layer_time, p.fan_speed];
    let bounded = p.fan_speed <= 1.0 && p.z_hop <= 10.0 && p.lift_radius <= 10.0;
    if !non_negative.iter().all(|v| v.is_finite() && *v >= 0.0) || !bounded {
        return Err(MotionError::BadParams("retraction, hop, lift, cooling out of range".into()));
    }
    Ok(())
}

/// The speed, acceleration and flow a role takes: a strand bridges; otherwise a wall shows
/// outside or not, a dense line on top or not.
fn role_motion(p: &MotionParams, role: Role) -> &RoleMotion {
    use layercraft_path::Face;
    match role.kind {
        _ if role.strand() => &p.bridge,
        Kind::Wall if role.face == Face::Inside => &p.inner_wall,
        Kind::Wall => &p.outer_wall,
        Kind::Solid if role.face == Face::Top => &p.top_surface,
        Kind::Solid => &p.solid,
        Kind::Sparse => &p.sparse,
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

/// The cross-section a bead lays: flattened on the layer below, or round for a strand.
fn section_area(path: &Path, height: f64) -> f64 {
    if path.role.strand() {
        std::f64::consts::PI * path.width * path.width / 4.0
    } else {
        bead_area(path.width, height)
    }
}

/// The speed of a bead: by role, the first layer's own, never past the melt limit.
fn bead_speed(p: &MotionParams, path: &Path, layer: &PathLayer) -> f64 {
    let base = if layer.index == 0 {
        if path.role.kind == Kind::Wall { p.first_layer_wall_speed } else { p.first_layer_infill_speed }
    } else {
        role_motion(p, path.role).speed
    };
    let melt = p.max_volumetric_speed / pushed(p, path, layer.height).max(1e-6);
    base.min(melt)
}

/// The speed of a wall bead hanging the fraction `u` of its width over air, `speed` on solid.
/// Each overhang speed holds at the middle of its band, the last at a bead wholly over air,
/// the wall's own at none; between, the speed runs linearly, so a wall whose overhang wavers
/// about a band's edge from layer to layer keeps nearly one speed. Never faster than `speed`.
fn overhang_speed(p: &MotionParams, u: f64, speed: f64) -> f64 {
    let n = p.overhang_speeds.len().saturating_sub(1);
    if u <= 0.0 || n == 0 {
        return speed;
    }
    let own = |s: f64| if s > 0.0 { s.min(speed) } else { speed };
    let bands =
        p.overhang_speeds.iter().take(n).enumerate().map(|(k, &s)| ((k as f64 + 0.5) / n as f64, own(s)));
    let full = p.overhang_speeds.get(n).map(|&s| (1.0, own(s)));
    let mut below = (0.0, speed);
    for (at, s) in bands.chain(full) {
        if u <= at {
            let t = (u - below.0) / (at - below.0);
            return below.1 + t * (s - below.1);
        }
        below = (at, s);
    }
    below.1
}

/// The plastic a bead pushes per mm, in mm²: its section times the flow, which the hot end
/// must melt.
fn pushed(p: &MotionParams, path: &Path, height: f64) -> f64 {
    p.flow_ratio * role_motion(p, path.role).flow * section_area(path, height)
}

/// The nozzle as the program is written: where it is, what it does, and the moves so far.
struct State {
    moves: Vec<Move>,
    at: [f64; 3],
    /// Whether `at` is where the nozzle is: not at a layer's start, after the machine's own
    /// G-code, which may move it.
    placed: bool,
    /// The bead being laid or just laid, with its speed: what a retraction wipes over.
    bead: Vec<[f64; 3]>,
    bead_speed: f64,
    /// The direction of the last move across the plate, a unit vector.
    heading: [f64; 2],
    /// The filament pulled back, in mm, which the next bead pushes again.
    retracted: f64,
    accel: f64,
    fan: f64,
}

impl State {
    /// The direction across the plate from the nozzle to `to`; the last one when it is there.
    fn heading_to(&self, to: Point) -> [f64; 2] {
        let d = [to[0] - self.at[0], to[1] - self.at[1]];
        let l = d[0].hypot(d[1]);
        if l > 1e-9 { [d[0] / l, d[1] / l] } else { self.heading }
    }

    /// A move to `to` (or in place), with the nozzle's state.
    fn push(&mut self, to: Option<[f64; 3]>, arc: Option<Arc>, e: f64, speed: f64, bead: Option<Bead>) {
        self.moves.push(Move { to, arc, e, speed, accel: self.accel, fan: self.fan, bead });
        if let Some(to) = to {
            if arc.is_none() {
                self.heading = self.heading_to([to[0], to[1]]);
            }
            self.at = to;
            self.placed = true;
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
            self.push(None, None, -before, p.retract_speed, None);
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
                self.push(Some(to), None, -pull * d / length, speed, None);
            }
            left -= pull;
        }
        // Below a nanometre of filament, what is left is rounding, not a pull.
        if left > 1e-6 {
            self.push(None, None, -left, p.retract_speed, None);
        }
        self.bead.clear();
        self.retracted = total;
    }

    /// Pushes back exactly what was pulled, at this bead's speed.
    fn deretract(&mut self, p: &MotionParams) {
        if self.retracted > 0.0 {
            let e = self.retracted;
            self.push(None, None, e, p.deretract_speed, None);
        }
        self.retracted = 0.0;
    }

    fn travel(&mut self, to: [f64; 3], speed: f64) {
        if !self.placed || to != self.at {
            self.push(Some(to), None, 0.0, speed, None);
        }
    }

    /// Climbs to `z`, as fast as the travel speed and the Z axis allow: one turn of a helix of
    /// the lift radius that sets off along `heading`, so it flows into a travel that way, and
    /// comes back over its start; straight up with no radius.
    fn lift(&mut self, p: &MotionParams, z: f64, heading: [f64; 2]) {
        let dz = z - self.at[2];
        if !self.placed || dz <= 1e-9 {
            return;
        }
        let r = p.lift_radius;
        let arc = (r > 0.0).then(|| Arc {
            centre: [self.at[0] - r * heading[1], self.at[1] + r * heading[0]],
            turn: std::f64::consts::TAU,
        });
        let length = (std::f64::consts::TAU * r).hypot(dz);
        let speed = p.travel.speed.min(p.limits.max_speed_z * length / dz);
        self.push(Some([self.at[0], self.at[1], z]), arc, 0.0, speed, None);
    }

    /// Leaves the bead just laid: pulls the filament back while wiping over the bead, then
    /// climbs the hop off it. Nothing to leave before the first bead.
    fn leave(&mut self, p: &MotionParams) {
        if !self.placed {
            return;
        }
        self.retract(p);
        self.lift(p, self.at[2] + p.z_hop, self.heading);
    }

    /// Moves to `start` on the layer at `z`: short hops stay down, longer ones retract and lift.
    /// Leaving takes the parameters of the bead left, `from`, whose string it breaks; arriving
    /// takes those of the bead reached, `to`. From where a layer starts, which is unknown, the
    /// nozzle goes over at the hop height.
    fn go_to(&mut self, from: &MotionParams, to: &MotionParams, start: Point, z: f64, first_of_layer: bool) {
        let d = (start[0] - self.at[0]).hypot(start[1] - self.at[1]);
        if first_of_layer || !self.placed || d > from.retract_min_travel {
            self.retract(from);
            self.accel = to.travel.accel;
            // Up first: the travel crosses nothing below the hop height.
            let hop = z + from.z_hop;
            let heading = self.heading_to(start);
            self.lift(from, hop, heading);
            let height = if self.placed { self.at[2].max(hop) } else { hop };
            self.travel([start[0], start[1], height], to.travel.speed);
            self.travel([start[0], start[1], z], to.travel.speed);
        } else {
            self.travel([start[0], start[1], z], to.travel.speed);
        }
        self.deretract(to);
        self.bead.clear();
        self.bead.push(self.at);
    }
}

/// Turns the beads into the program's moves, layer by layer. Each bead takes the parameters
/// at its centre. Beads run at their full speeds: cooling is a pass of its own (`cool`), and
/// the machine's own G-code goes around the layers (`frame`).
pub fn compile(paths: &Paths, field: &Field<MotionParams>) -> Result<Program, MotionError> {
    let base = field.base();
    check(base)?;
    let filament_area = std::f64::consts::PI * (base.filament_diameter / 2.0).powi(2);
    let mut s = State {
        moves: Vec::new(),
        at: [0.0; 3],
        placed: false,
        bead: Vec::new(),
        bead_speed: base.travel.speed,
        heading: [1.0, 0.0],
        retracted: 0.0,
        accel: base.travel.accel,
        fan: 0.0,
    };
    // The parameters of the bead last laid, which leaving it takes.
    let mut last = Rc::new(base.clone());
    let mut layers: Vec<Layer> = Vec::with_capacity(paths.layers.len());
    for layer in &paths.layers {
        if let Some(previous) = layers.last_mut() {
            // Off the last bead while still on the layer below, where the nozzle is known.
            s.leave(&last);
            previous.moves = std::mem::take(&mut s.moves);
        }
        layers.push(Layer { index: layer.index, z: layer.top, height: layer.height, moves: Vec::new() });
        // After the machine's layer change, the nozzle may be anywhere.
        s.placed = false;
        for (k, path) in layer.paths.iter().enumerate() {
            let p = field.at(centre(&path.points, layer.top), check)?;
            let points = path_points(path, p.seam_gap);
            let Some(&start) = points.first() else { continue };
            s.fan = if layer.index < p.fan_off_layers { 0.0 } else { p.fan_speed };
            if path.travel_inside && k > 0 && s.placed {
                // Inside the part from the last bead: straight there, nothing pulled back.
                s.deretract(&p);
                s.travel([start[0], start[1], layer.top], p.travel.speed);
                s.bead.clear();
                s.bead.push(s.at);
            } else {
                s.go_to(&last, &p, start, layer.top, k == 0);
            }
            s.accel = if layer.index == 0 { p.first_layer_accel } else { role_motion(&p, path.role).accel };
            let speed = bead_speed(&p, path, layer);
            let per_mm = pushed(&p, path, layer.height) / filament_area;
            let bead = Bead { role: path.role, width: path.width };
            for (i, w) in points.windows(2).enumerate() {
                let [a, b] = [w.first().copied().unwrap_or(start), w.get(1).copied().unwrap_or(start)];
                let l = (b[0] - a[0]).hypot(b[1] - a[1]);
                if l <= 0.0 {
                    continue;
                }
                let speed = overhang_speed(&p, path.overhang.get(i).copied().unwrap_or(0.0), speed);
                s.push(Some([b[0], b[1], layer.top]), None, per_mm * l, speed, Some(bead));
                s.bead.push(s.at);
                s.bead_speed = speed;
            }
            last = p;
        }
    }
    // Off the part at once, before the machine's own slow end sequence.
    s.leave(&last);
    let max_z = paths.layers.last().map(|l| l.top).unwrap_or(0.0);
    let heading = s.heading;
    s.lift(&last, max_z + last.z_hop, heading);
    if let Some(previous) = layers.last_mut() {
        previous.moves = std::mem::take(&mut s.moves);
    }
    Ok(Program { layers })
}
