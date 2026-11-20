//! Stage 4: the program the printer runs, pass by pass (a layer prints one pass per material), as
//! moves that each carry their whole state: where the nozzle goes and along what (a line, or an arc
//! about a vertical axis, climbing as it turns), the filament it pushes or pulls, its speed and
//! acceleration, the fan, and the bead it lays. Moves can be concatenated, reordered or edited
//! without any state leaking from one to the next; the machine's own G-code goes around (`frame`).
//!
//! A bead `w` wide on a layer `h` high has the cross-section of a rectangle with half-discs at its
//! sides, `(w − h)·h + π·h²/4`, and a strand, round, the disc `π·w²/4`; a move of length `L` fills
//! that times `L`, and pushes `flow · area · L / filament_area` of filament; over a bent slab `t`
//! thick, it fills its strip, as wide as beads lie apart, `t` deep, less what the nozzle's face,
//! passing later and lower, planes off (`room`). A wall loop stops `seam_gap` short of its start.
//! Speeds come by role, slowed by the share of the bead over air (`overhang_speeds`, by band, with
//! `overhang_fan_speed` past a share): a wall's as measured, a bridge's strand wholly, as slow as a
//! wall over air; all capped by the hot end's volumetric limit. A layer too short to cool is slowed
//! by a pass of its own (`cool`), and waits out the rest off the part when even its slowest is too
//! short; the beads are written as the fewest lines and arcs within the resolution (`fit`), and
//! every move is held to its filament's melt limit and the Z axis's speed (`limit`). Leaving a
//! bead, the nozzle wipes back over it while it pulls the filament, then climbs one turn of a helix:
//! moving sideways as it rises, it breaks the string instead of stretching it. Each bead takes the
//! parameters at its centre; `summarize` reads a program's summary from its moves.

mod cool;
mod fit;
mod frame;
mod leaf;
mod limit;
mod merge;
mod moves;
mod park;
mod plan;
mod plane;
mod room;
mod summary;
mod time;

pub use cool::cool;
pub use fit::fit;
pub use frame::{Frame, Heat, frame};
pub use limit::limit;
pub use merge::merge;
pub use plan::compile;
pub use summary::{Summary, summarize};
pub use time::{Limits, estimate};

use layercraft_geom::{Carrier, GeomError, Warp};
pub use layercraft_geom::{bead_area, disc_area};
use layercraft_path::Role;
use layercraft_template::TemplateError;
use layercraft_tune::TuneError;
use serde::{Deserialize, Serialize};

/// The decimals the program's positions are written to in G-code, in mm, and the unit of the last:
/// a micrometre.
pub const POSITION_DECIMALS: usize = 3;
pub const POSITION_UNIT: f64 = 1e-3;

/// The way a move goes about a vertical axis through `centre`, turning `turn` radians
/// (counter-clockwise positive, at most a full turn either way) and climbing evenly.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Arc {
    pub centre: [f64; 2],
    pub turn: f64,
}

/// The plastic a move lays: what it is for, and how wide.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Bead {
    pub role: Role,
    pub width: f64,
}

/// One move of the nozzle, with all it does.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Move {
    /// Where the nozzle goes; nowhere, for filament pushed or pulled in place.
    pub to: Option<[f64; 3]>,
    /// Along an arc; a straight line when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arc: Option<Arc>,
    /// Filament pushed (positive) or pulled back (negative), in mm.
    pub e: f64,
    /// Along the path, in mm/s; the filament's own speed for a move in place.
    pub speed: f64,
    /// In mm/s².
    pub accel: f64,
    /// The part cooling fan, from 0 to 1.
    pub fan: f64,
    /// The bead laid, for a move that lays one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bead: Option<Bead>,
    /// The seconds the nozzle then stays where the move leaves it, the fan running.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub wait: f64,
}

fn is_zero(v: &f64) -> bool {
    *v == 0.0
}

/// One pass of a layer: the moves printed in one material at `z`, or on the layer's top bent from
/// it, after the machine's own layer or filament change, from wherever that leaves the nozzle. The
/// passes of a layer follow one another and share its index, height and warp.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pass {
    pub index: usize,
    pub z: f64,
    pub height: f64,
    /// The material the pass prints: a layer printed in several materials is several passes, one
    /// after the other, with the filament changed between them.
    #[serde(default, skip_serializing_if = "is_first")]
    pub material: usize,
    /// How far the layer's top, where its beads lie, rises above `z` at each corner of the
    /// program's carrier: zero where it is flat.
    #[serde(default, skip_serializing_if = "Warp::is_zero")]
    pub warp: Warp,
    /// Whether the pass's beads follow no top: free strands, each resting on the plate or on the
    /// beads of the layers before (`verify` holds it to that), `z` the highest the nozzle goes.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub free: bool,
    pub moves: Vec<Move>,
}

impl Pass {
    /// The highest the layer's top reaches.
    pub fn highest(&self) -> f64 {
        self.z + self.warp.range()[1]
    }
}

fn is_first(n: &usize) -> bool {
    *n == 0
}

/// The output of the motion stage: what it does is read from it (`summarize`), never stored.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Program {
    /// The triangles the bent layers' warps are linear on, with their corners; none when every
    /// layer is flat.
    #[serde(default, skip_serializing_if = "Carrier::is_empty")]
    pub carrier: Carrier,
    /// The passes in printing order, layer after layer.
    #[serde(alias = "layers")]
    pub passes: Vec<Pass>,
}

/// Whether two passes, one after the other, print one layer.
fn same_layer(a: &Pass, b: &Pass) -> bool {
    a.index == b.index
}

impl Program {
    /// The layers, each as its passes.
    pub fn layers(&self) -> impl Iterator<Item = &[Pass]> {
        self.passes.chunk_by(same_layer)
    }

    /// The layers, each as its passes, to change.
    pub fn layers_mut(&mut self) -> impl Iterator<Item = &mut [Pass]> {
        self.passes.chunk_by_mut(same_layer)
    }

    /// The highest any layer's top reaches; 0 with no layer.
    pub fn highest(&self) -> f64 {
        self.passes.iter().map(Pass::highest).fold(0.0, f64::max)
    }

    /// How many materials the program prints, numbered from 0: one at least.
    pub fn materials(&self) -> usize {
        self.passes.iter().map(|p| p.material).max().map_or(1, |m| m + 1)
    }
}

/// A material's filament as its moves see it: its diameter, which turns the millimetres pushed
/// into plastic, and how much of it the hot end melts.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Filament {
    pub diameter: f64,
    /// The hot end's melt limit for it, in mm³/s.
    pub max_volumetric_speed: f64,
}

/// Speed, acceleration and flow for one role.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoleMotion {
    pub speed: f64,
    pub accel: f64,
    /// The role's share of the filament's flow ratio: a bead pushes `flow_ratio · flow` of its
    /// volume.
    #[serde(default = "one")]
    pub flow: f64,
}

fn one() -> f64 {
    1.0
}

/// The machine's G-code templates around the moves.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Templates {
    /// The machine's motion limits, set before anything else.
    pub limits: String,
    pub start: String,
    pub end: String,
    pub layer_change: String,
    /// Printed after the start, before the first layer.
    pub prologue: String,
    /// The change from one filament to another, between two passes of different materials.
    #[serde(default)]
    pub change_filament: String,
}

/// How beads become moves.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotionParams {
    pub filament_diameter: f64,
    pub flow_ratio: f64,
    /// The hot end's melt limit, in mm³/s.
    pub max_volumetric_speed: f64,
    pub outer_wall: RoleMotion,
    pub inner_wall: RoleMotion,
    pub top_surface: RoleMotion,
    pub solid: RoleMotion,
    pub sparse: RoleMotion,
    pub bridge: RoleMotion,
    /// The supports' lines, and their top, the interface the part rests on.
    pub support: RoleMotion,
    pub support_interface: RoleMotion,
    /// The speeds of a wall bead hanging over air, by the fraction of its width that hangs:
    /// with `n + 1` speeds, speed `k` for the band `(k/n, (k+1)/n]`, the last for a bead wholly
    /// over air. A speed of 0 keeps the wall's own; none, no slowing.
    #[serde(default)]
    pub overhang_speeds: Vec<f64>,
    pub first_layer_wall_speed: f64,
    pub first_layer_infill_speed: f64,
    pub first_layer_accel: f64,
    pub travel: RoleMotion,
    pub retract_length: f64,
    pub retract_speed: f64,
    pub deretract_speed: f64,
    /// Travels shorter than this are made without retracting.
    pub retract_min_travel: f64,
    /// How far short of its start a loop stops, in mm.
    pub seam_gap: f64,
    /// How far back over the last bead the nozzle wipes while retracting, in mm (0: no wipe).
    pub wipe_distance: f64,
    /// The fraction of the retraction done in place before the wipe, from 0 to 1.
    pub retract_before_wipe: f64,
    pub z_hop: f64,
    /// The radius of the helix a lift turns, in mm (0: straight up).
    pub lift_radius: f64,
    /// The radius of the nozzle's flat face, in mm, which levels the beads it passes over later
    /// and lower on a bent layer: each bead pushes only the plastic that stays (0: none planed).
    #[serde(default)]
    pub face_radius: f64,
    /// The least time a layer takes, to cool before the next lands on it, in seconds: slowed
    /// down to `slow_down_min_speed`, then waiting off the part.
    pub slow_down_layer_time: f64,
    pub slow_down_min_speed: f64,
    pub fan_speed: f64,
    /// The fan, 0 to 1, over a strand on air and over a wall hanging at least
    /// `overhang_fan_share` of its width; a share of 0 leaves walls to the layer's fan.
    #[serde(default)]
    pub overhang_fan_speed: f64,
    #[serde(default)]
    pub overhang_fan_share: f64,
    /// Layers printed with the fan off, from the first.
    pub fan_off_layers: usize,
    /// How far, in mm, the moves written may stray from the beads traced (0: as traced).
    #[serde(default)]
    pub resolution: f64,
    /// Whether the moves written may be arcs.
    #[serde(default)]
    pub arc_fitting: bool,
    pub limits: Limits,
    pub templates: Templates,
}

/// The most overhang speeds a material may give.
const MAX_OVERHANG_SPEEDS: usize = 100;

/// The most a hop, a lift's radius or the nozzle's face may measure, in mm.
const MAX_HEAD_MM: f64 = 10.0;

impl MotionParams {
    /// Every role's motion, by name, travel last.
    fn roles(&self) -> [(&'static str, &RoleMotion); 9] {
        [
            ("outer wall", &self.outer_wall),
            ("inner wall", &self.inner_wall),
            ("top surface", &self.top_surface),
            ("solid", &self.solid),
            ("sparse", &self.sparse),
            ("bridge", &self.bridge),
            ("support", &self.support),
            ("support interface", &self.support_interface),
            ("travel", &self.travel),
        ]
    }

    /// The filament its moves are held to (`limit`, `verify`).
    pub fn filament(&self) -> Filament {
        Filament { diameter: self.filament_diameter, max_volumetric_speed: self.max_volumetric_speed }
    }

    /// Whether every parameter is one the stage can work with: the failing one named.
    pub(crate) fn check(&self) -> Result<(), MotionError> {
        let bad = |what: String| Err(MotionError::BadParams(what));
        let positive = |v: f64| v.is_finite() && v > 0.0;
        for (name, r) in self.roles() {
            // A travel lays nothing: its flow is never read.
            if !(positive(r.speed) && positive(r.accel) && (name == "travel" || positive(r.flow))) {
                return bad(format!("the {name}'s speed, acceleration and flow must be positive"));
            }
        }
        let positives = [
            ("filament diameter", self.filament_diameter),
            ("flow ratio", self.flow_ratio),
            ("melt limit", self.max_volumetric_speed),
            ("first layer's wall speed", self.first_layer_wall_speed),
            ("first layer's infill speed", self.first_layer_infill_speed),
            ("first layer's acceleration", self.first_layer_accel),
            ("retraction speed", self.retract_speed),
            ("deretraction speed", self.deretract_speed),
            ("slowest cooling speed", self.slow_down_min_speed),
            ("machine's acceleration", self.limits.max_accel),
            ("machine's speed", self.limits.max_speed_xy),
            ("machine's Z speed", self.limits.max_speed_z),
        ];
        if let Some((name, v)) = positives.into_iter().find(|&(_, v)| !positive(v)) {
            return bad(format!("the {name} must be positive, not {v}"));
        }
        let unbounded = f64::INFINITY;
        let ranges = [
            ("jerk", self.limits.jerk, unbounded),
            ("retraction", self.retract_length, unbounded),
            ("least travel retracted", self.retract_min_travel, unbounded),
            ("seam gap", self.seam_gap, unbounded),
            ("wipe", self.wipe_distance, unbounded),
            ("share retracted before the wipe", self.retract_before_wipe, 1.0),
            ("hop", self.z_hop, MAX_HEAD_MM),
            ("lift's radius", self.lift_radius, MAX_HEAD_MM),
            ("face's radius", self.face_radius, MAX_HEAD_MM),
            ("cooling time", self.slow_down_layer_time, unbounded),
            ("fan", self.fan_speed, 1.0),
            ("overhang fan", self.overhang_fan_speed, 1.0),
            ("overhang fan's share", self.overhang_fan_share, 1.0),
            ("resolution", self.resolution, unbounded),
        ];
        if let Some((name, v, most)) =
            ranges.into_iter().find(|&(_, v, most)| !(v.is_finite() && (0.0..=most).contains(&v)))
        {
            return bad(format!("the {name} must lie from 0 to {most}, not {v}"));
        }
        let speeds = &self.overhang_speeds;
        if speeds.len() > MAX_OVERHANG_SPEEDS || !speeds.iter().all(|v| v.is_finite() && *v >= 0.0) {
            return bad(format!("at most {MAX_OVERHANG_SPEEDS} overhang speeds, finite, not negative"));
        }
        Ok(())
    }
}

/// Why the motion stage failed.
#[derive(Debug, thiserror::Error)]
pub enum MotionError {
    #[error("bad motion parameters: {0}")]
    BadParams(String),
    #[error("bad motion parameters: {0}")]
    Tune(#[from] TuneError),
    #[error("the machine's {0} G-code: {1}")]
    Template(String, TemplateError),
    #[error("motion geometry: {0}")]
    Geometry(#[from] GeomError),
}

#[cfg(test)]
mod tests;
