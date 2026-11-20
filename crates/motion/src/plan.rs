//! From ordered beads to the program: travels, retractions, wipes and lifts, layer by layer.
//!
//! On a bent layer every bead is cut where the layer bends (`leaf`), each piece laying its exact
//! mean section. A travel longer than the retraction's minimum pulls the filament back and climbs
//! straight up over the highest the layer reaches on its way: left under pressure, the nozzle
//! would draw a string behind it, wherever it goes. A shorter one slides along the layer, unless
//! it crosses a bead laid at this height: that one lifts too, nothing pulled back. The helix is
//! kept for where the layer is level around the nozzle. A pass starts over its layer's highest
//! point, the pass before left at least over its own; done, its beads keep the room the nozzle's
//! face leaves them over the cuts of the layer below (`room`).

use crate::leaf::Leaf;
use crate::room::{self, Cuts};
use crate::{Arc, Bead, MotionError, MotionParams, Move, Pass, Program, RoleMotion, bead_area, disc_area};
use layercraft_geom::{GeomError, Point, Relief, bead_spacing};
use layercraft_path::{Kind, Path, PathLayer, Paths, Role, Support};
use layercraft_tune::{Field, centre};
use std::rc::Rc;

/// The sine of the angle from one heading to the next below which they are the same, or
/// opposite, line: rounding, not a turn.
const STRAIGHT_ON: f64 = 1e-9;

/// The speed, acceleration and flow a role takes: a strand bridges; otherwise a wall shows
/// outside or not, a dense line on top or not, a support's line is its top or not.
fn role_motion(p: &MotionParams, role: Role) -> &RoleMotion {
    use layercraft_path::Face;
    match role.kind {
        _ if role.strand() => &p.bridge,
        Kind::Wall if role.face == Face::Inside => &p.inner_wall,
        Kind::Wall => &p.outer_wall,
        Kind::Solid if role.face == Face::Top => &p.top_surface,
        Kind::Solid => &p.solid,
        Kind::Sparse => &p.sparse,
        Kind::Support if role.face == Face::Top => &p.support_interface,
        Kind::Support => &p.support,
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
    if path.role.strand() { disc_area(path.width) } else { bead_area(path.width, height) }
}

/// The mean cross-section along a straight piece of bead on a layer `height` high, over which
/// the slab's thickness runs from `from` to `to`: the strip the bead fills, as wide as beads lie
/// apart, through the slab; on a flat slab, its section. A strand, round, keeps its disc.
fn mean_section(path: &Path, height: f64, from: f64, to: f64) -> f64 {
    if path.role.strand() {
        disc_area(path.width)
    } else {
        bead_spacing(path.width, height) * (from + to) / 2.0
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

/// The fraction of segment `i` of a bead's width hanging over air: a strand on air hangs whole,
/// a wall by what the path stage measured under it.
fn hanging(path: &Path, i: usize) -> f64 {
    if path.role.support == Support::Air { 1.0 } else { path.overhang.get(i).copied().unwrap_or(0.0) }
}

/// The speed of a bead hanging the fraction `u` of its width over air, `speed` on solid: the
/// speed of the overhang band `u` falls in, never faster than `speed`.
fn overhang_speed(p: &MotionParams, u: f64, speed: f64) -> f64 {
    let n = p.overhang_speeds.len().saturating_sub(1);
    if u <= 0.0 || n == 0 {
        return speed;
    }
    let band = if u >= 1.0 { n } else { ((u * n as f64).ceil() as usize).clamp(1, n) - 1 };
    match p.overhang_speeds.get(band) {
        Some(&s) if s > 0.0 => s.min(speed),
        _ => speed,
    }
}

/// The fan over a bead hanging the fraction `u` of its width over air, `fan` on solid: the
/// overhang fan from `overhang_fan_share` on, and over every strand on air.
fn overhang_fan(p: &MotionParams, role: Role, u: f64, fan: f64) -> f64 {
    let hangs = role.support == Support::Air || (p.overhang_fan_share > 0.0 && u >= p.overhang_fan_share);
    if hangs { fan.max(p.overhang_fan_speed) } else { fan }
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
    /// The direction of the last move across the plate, a unit vector; none before the nozzle
    /// has gone anywhere known.
    heading: Option<[f64; 2]>,
    /// The filament pulled back, in mm, which the next bead pushes again.
    retracted: f64,
    accel: f64,
    fan: f64,
}

impl State {
    /// The direction across the plate from the nozzle to `to`; the last one when it is there.
    fn heading_to(&self, to: Point) -> Option<[f64; 2]> {
        let d = [to[0] - self.at[0], to[1] - self.at[1]];
        let l = d[0].hypot(d[1]);
        if l > 1e-9 { Some([d[0] / l, d[1] / l]) } else { self.heading }
    }

    /// A move to `to` (or in place), with the nozzle's state, at the speed asked: the machine's
    /// limits hold it later (`limit`).
    fn push(&mut self, to: Option<[f64; 3]>, arc: Option<Arc>, e: f64, speed: f64, bead: Option<Bead>) {
        self.moves.push(Move { to, arc, e, speed, accel: self.accel, fan: self.fan, bead, wait: 0.0 });
        if let Some(to) = to {
            // The first move leaves from nowhere known: it gives no direction.
            if arc.is_none() && (self.placed || self.heading.is_some()) {
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
            let end = [
                from[0] + t * (to[0] - from[0]),
                from[1] + t * (to[1] - from[1]),
                from[2] + t * (to[2] - from[2]),
            ];
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

    /// Pushes back exactly what was pulled, at the deretraction speed of `p`, the bead's to come.
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

    /// Climbs to `z` at the travel speed, which the Z axis holds (`limit`): one turn of a helix of
    /// the lift radius that sets off along `heading`, so it flows into a travel that way, and
    /// comes back over its start; the helix turns the way the nozzle turns from the last move
    /// to `heading`, and to the left where it goes straight on or back. Straight up with no
    /// radius or no heading, or where the layer `leaf` is not level under the helix, which
    /// would climb slower than the layer.
    fn lift(&mut self, p: &MotionParams, z: f64, heading: Option<[f64; 2]>, leaf: &Leaf) {
        let dz = z - self.at[2];
        if !self.placed || dz <= 1e-9 {
            return;
        }
        let r = if leaf.level_within([self.at[0], self.at[1]], 2.0 * p.lift_radius) {
            p.lift_radius
        } else {
            0.0
        };
        let arc = heading.filter(|_| r > 0.0).map(|h| {
            let turn = self.heading.map_or(0.0, |last| last[0] * h[1] - last[1] * h[0]);
            let side = if turn < -STRAIGHT_ON { -1.0 } else { 1.0 };
            Arc {
                centre: [self.at[0] - side * r * h[1], self.at[1] + side * r * h[0]],
                turn: side * std::f64::consts::TAU,
            }
        });
        self.push(Some([self.at[0], self.at[1], z]), arc, 0.0, p.travel.speed, None);
    }

    /// Leaves the bead just laid on `leaf`: pulls the filament back while wiping over the bead,
    /// then climbs the hop off it. Nothing to leave before the first bead.
    fn leave(&mut self, p: &MotionParams, leaf: &Leaf) {
        if !self.placed {
            return;
        }
        self.retract(p);
        self.lift(p, self.at[2] + p.z_hop, self.heading, leaf);
    }

    /// Leaves the layer `leaf`, done: off its last bead, laid with `p`, while still on it, where
    /// the nozzle is known, and over its highest point, where the next layer starts from; the
    /// layer's moves.
    fn close(&mut self, p: &MotionParams, leaf: &Leaf) -> Vec<Move> {
        self.leave(p, leaf);
        let heading = self.heading;
        self.lift(p, leaf.highest() + p.z_hop, heading, leaf);
        std::mem::take(&mut self.moves)
    }

    /// Slides along the layer `leaf` to `to`, through every point where it bends.
    fn glide(&mut self, leaf: &Leaf, to: Point, speed: f64) {
        let from = [self.at[0], self.at[1]];
        for q in leaf.pieces(from, to) {
            self.travel([q[0], q[1], leaf.z(q)], speed);
        }
    }

    /// Lifts over the layer `leaf` on the way to `start`, travels there and comes down on it:
    /// leaving with the parameters of the bead left, `leaving`, arriving with `arriving`'s. From
    /// where the nozzle is not known, it goes over the layer's highest point.
    fn hop(&mut self, leaving: &MotionParams, arriving: &MotionParams, start: Point, leaf: &Leaf) {
        self.accel = arriving.travel.accel;
        // Up first: the travel crosses nothing below the hop over the layer on its way.
        let over =
            if self.placed { leaf.highest_along([self.at[0], self.at[1]], start) } else { leaf.highest() };
        let hop = over + leaving.z_hop;
        let heading = self.heading_to(start);
        self.lift(leaving, hop, heading, leaf);
        let height = if self.placed { self.at[2].max(hop) } else { hop };
        self.travel([start[0], start[1], height], arriving.travel.speed);
        self.travel([start[0], start[1], leaf.z(start)], arriving.travel.speed);
    }

    /// Moves to `start` on the layer `leaf`: short hops slide along it, or lift over the laid bead
    /// they `cross`, and longer ones retract and lift. Leaving takes the parameters of the bead
    /// left, `leaving`, whose string it breaks; arriving takes those of the bead reached,
    /// `arriving`. From where a pass starts, which is unknown, the nozzle goes over the layer's
    /// highest point by the hop.
    fn go_to(
        &mut self,
        leaving: &MotionParams,
        arriving: &MotionParams,
        start: Point,
        leaf: &Leaf,
        cross: bool,
    ) {
        let d = (start[0] - self.at[0]).hypot(start[1] - self.at[1]);
        if !self.placed || d > leaving.retract_min_travel {
            self.retract(leaving);
            self.hop(leaving, arriving, start, leaf);
        } else if cross {
            self.hop(leaving, arriving, start, leaf);
        } else {
            self.glide(leaf, start, arriving.travel.speed);
        }
        self.deretract(arriving);
        self.bead.clear();
        self.bead.push(self.at);
    }
}

/// Turns the beads into the program's moves, pass by pass. Each bead takes the parameters of its
/// material (`fields`, the last given standing for those past them) at its centre. Beads run at
/// their full speeds: cooling is a pass of its own (`cool`), and the machine's own G-code goes
/// around the passes (`frame`).
pub fn compile(paths: &Paths, fields: &[Field<MotionParams>]) -> Result<Program, MotionError> {
    let (Some(first), Some(last_field)) = (fields.first(), fields.last()) else {
        return Err(MotionError::BadParams("no material's parameters".into()));
    };
    for field in fields {
        field.base().check()?;
    }
    let base = first.base();
    let mut s = State {
        moves: Vec::new(),
        at: [0.0; 3],
        placed: false,
        bead: Vec::new(),
        bead_speed: base.travel.speed,
        heading: None,
        retracted: 0.0,
        accel: base.travel.accel,
        fan: 0.0,
    };
    // The parameters of the bead last laid, which leaving it takes.
    let mut last = Rc::new(base.clone());
    let mut passes: Vec<Pass> = Vec::with_capacity(paths.layers.len());
    // Each layer's top over its plane, its warp on the paths' carrier.
    let reliefs = paths
        .layers
        .iter()
        .map(|l| paths.carrier.relief(&l.warp))
        .collect::<Result<Vec<Relief>, GeomError>>()?;
    let bed = Relief::default();
    let mut below = &bed;
    // The layer the passes so far belong to, and its top: a pass stands on the layer before its
    // own, not on the pass before it.
    let mut current: Option<(usize, &Relief)> = None;
    let mut previous_leaf: Option<Leaf> = None;
    // How deep the nozzle's face, one for the whole print, cut into the top of the layer below the
    // current one, and into the current one's so far, pass by pass.
    let (mut cuts_below, mut cuts_here) = (Cuts::default(), Cuts::default());
    // The material of the pass before.
    let mut material: Option<usize> = None;
    for (layer, relief) in paths.layers.iter().zip(&reliefs) {
        // A pass after a filament change lays its beads on plastic the change left to cool: as
        // on the first layers, without the fan, so that they melt into it.
        let changed = material.is_some_and(|m| m != layer.material);
        material = Some(layer.material);
        if let (Some(previous), Some(left)) = (passes.last_mut(), &previous_leaf) {
            previous.moves = s.close(&last, left);
            cuts_here.extend(room::keep(&mut previous.moves, left, &cuts_below, base.face_radius));
        }
        if let Some((index, warp)) = current
            && index != layer.index
        {
            below = warp;
            cuts_below = std::mem::take(&mut cuts_here);
        }
        let leaf = Leaf { top: layer.top, height: layer.height, warp: relief, below };
        passes.push(Pass {
            index: layer.index,
            z: layer.top,
            height: layer.height,
            material: layer.material,
            warp: layer.warp.clone(),
            free: false,
            moves: Vec::new(),
        });
        // After the machine's layer change, the nozzle may be anywhere.
        s.placed = false;
        let field = fields.get(layer.material).unwrap_or(last_field);
        for path in &layer.paths {
            let p = field.at(centre(&path.points, layer.top), MotionParams::check)?;
            let points = path_points(path, p.seam_gap);
            let Some(&start) = points.first() else { continue };
            let fan_off = layer.index < p.fan_off_layers || changed;
            s.fan = if fan_off { 0.0 } else { p.fan_speed };
            s.go_to(&last, &p, start, &leaf, path.travel_crosses);
            s.accel = if layer.index == 0 { p.first_layer_accel } else { role_motion(&p, path.role).accel };
            let speed = bead_speed(&p, path, layer);
            let per_mm3 = p.flow_ratio * role_motion(&p, path.role).flow / disc_area(p.filament_diameter);
            let bead = Bead { role: path.role, width: path.width };
            for (i, w) in points.windows(2).enumerate() {
                let &[a, b] = w else { continue };
                if (b[0] - a[0]).hypot(b[1] - a[1]) <= 0.0 {
                    continue;
                }
                let u = hanging(path, i);
                let speed = overhang_speed(&p, u, speed);
                s.fan = if fan_off { 0.0 } else { overhang_fan(&p, path.role, u, p.fan_speed) };
                let mut from = a;
                for q in leaf.pieces(a, b) {
                    let l = (q[0] - from[0]).hypot(q[1] - from[1]);
                    let section = mean_section(path, layer.height, leaf.thickness(from), leaf.thickness(q));
                    s.push(Some([q[0], q[1], leaf.z(q)]), None, per_mm3 * section * l, speed, Some(bead));
                    s.bead.push(s.at);
                    from = q;
                }
                s.bead_speed = speed;
            }
            last = p;
        }
        current = Some((layer.index, relief));
        previous_leaf = Some(leaf);
    }
    // Off the part at once, before the machine's own slow end sequence.
    if let (Some(previous), Some(left)) = (passes.last_mut(), &previous_leaf) {
        previous.moves = s.close(&last, left);
        room::keep(&mut previous.moves, left, &cuts_below, base.face_radius);
    }
    Ok(Program { carrier: paths.carrier.clone(), passes })
}
