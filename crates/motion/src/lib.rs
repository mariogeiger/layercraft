//! Stage 4: the program the printer runs, layer by layer, as moves that each carry their
//! whole state: where the nozzle goes and along what (a line, or an arc about a vertical axis,
//! climbing as it turns), the filament it pushes or pulls, its speed and acceleration, the
//! fan, and the bead it lays. Moves can be concatenated, reordered or edited without any state
//! leaking from one to the next; the machine's own G-code goes around the layers (`frame`).
//!
//! A bead `w` wide on a layer `h` high has the cross-section of a rectangle with half-discs at
//! its sides, `(w − h)·h + π·h²/4`, and a strand, round, the disc `π·w²/4`; a move of length
//! `L` fills that times `L`, and pushes `flow · area · L / filament_area` of filament. A wall
//! loop stops `seam_gap` short of its start. Speeds come by role, capped by the hot end's
//! volumetric limit; a layer too short to cool is slowed by a pass of its own (`cool`). Leaving
//! a bead, the nozzle wipes back over it while it pulls the filament, then climbs one turn of a
//! helix: moving sideways as it rises, it breaks the string instead of stretching it. Each bead
//! takes the parameters at its centre; `finish` reads a program's summary from its moves.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

mod cool;
mod frame;
mod melt;
mod merge;
mod moves;
mod plan;
mod time;

pub use cool::cool;
pub use frame::{Frame, finish, frame};
pub use melt::melt;
pub use merge::merge;
pub use plan::{compile, placeholders};
pub use time::{Limits, estimate};

pub use layercraft_geom::bead_area;
use layercraft_machine::TemplateError;
use layercraft_path::Role;
use layercraft_tune::TuneError;
use serde::{Deserialize, Serialize};

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
}

/// One layer: the moves printed at `z`, after the machine's own layer change, from wherever
/// that leaves the nozzle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Layer {
    pub index: usize,
    pub z: f64,
    pub height: f64,
    pub moves: Vec<Move>,
}

/// The output of the motion stage: what it does is read from it (`finish`), never stored.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Program {
    pub layers: Vec<Layer>,
}

/// What the program does, for people and for comparisons.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Summary {
    pub layers: usize,
    pub max_z: f64,
    /// Filament pushed, in mm.
    pub filament_mm: f64,
    /// Plastic pushed per layer, in mm³.
    pub volume_per_layer: Vec<f64>,
    /// Time of the moves, in seconds (the machine's own start and end sequences not counted).
    pub motion_seconds: f64,
    /// Time of the moves up to the end of the first layer, in seconds.
    pub first_layer_seconds: f64,
    /// The first layer's extent `[min, max]`, for bed levelling.
    pub first_layer_bounds: [[f64; 2]; 2],
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
    pub filament_start: String,
    /// Printed after the filament's start, before the first layer.
    pub prologue: String,
    pub filament_end: String,
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
    /// A layer printing faster than this is slowed down, in seconds.
    pub slow_down_layer_time: f64,
    pub slow_down_min_speed: f64,
    pub fan_speed: f64,
    /// Layers printed with the fan off, from the first.
    pub fan_off_layers: usize,
    pub limits: Limits,
    pub templates: Templates,
}

/// Why the motion stage failed.
#[derive(Debug)]
pub enum MotionError {
    BadParams(String),
    Template(String, TemplateError),
}

impl std::fmt::Display for MotionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MotionError::BadParams(e) => write!(f, "bad motion parameters: {e}"),
            MotionError::Template(which, e) => write!(f, "the machine's {which} G-code: {e}"),
        }
    }
}

impl std::error::Error for MotionError {}

impl From<TuneError> for MotionError {
    fn from(e: TuneError) -> Self {
        MotionError::BadParams(e.to_string())
    }
}

#[cfg(test)]
mod tests;
