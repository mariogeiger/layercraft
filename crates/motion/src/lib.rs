//! Stage 4: the program the printer runs — moves with their filament, speeds and the machine's
//! own G-code around them.
//!
//! A bead `w` wide on a layer `h` high has the cross-section of a rectangle with half-discs at
//! its sides, `(w − h)·h + π·h²/4`, and a bridge strand, hanging free, the disc `π·w²/4`; a
//! move of length `L` fills that times `L`, and pushes `flow · area · L / filament_area` of
//! filament. A wall loop stops `seam_gap` short of its start, so the seam does not pile up. Speeds come by role, capped by the hot end's
//! volumetric limit, and slowed down on layers too short to cool. Print time comes from a
//! look-ahead planner of the machine's accelerations and corner speeds.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

mod plan;
mod time;

pub use plan::{compile, placeholders};
pub use time::{Limits, estimate};

pub use layercraft_geom::bead_area;
use layercraft_machine::{Scope, TemplateError};
use layercraft_path::Role;
use serde::{Deserialize, Serialize};

/// One instruction of the program.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Op {
    /// The machine's own G-code, copied as is.
    Raw { text: String },
    /// The start of a layer printed at `z`.
    Layer { index: usize, z: f64, height: f64 },
    /// What the following beads are.
    Feature { role: Role, width: f64 },
    /// A move without extrusion, at `speed` mm/s.
    Travel { to: [f64; 3], speed: f64 },
    /// A move back over the bead just laid, pulling `e` (negative) mm of filament on the way, at
    /// `speed` mm/s: the nozzle wipes its tip on the part instead of leaving a string.
    Wipe { to: [f64; 3], e: f64, speed: f64 },
    /// A vertical move to `z` from wherever the nozzle is, at `speed` mm/s.
    Lift { z: f64, speed: f64 },
    /// A move pushing `e` mm of filament, at `speed` mm/s.
    Extrude { to: [f64; 3], e: f64, speed: f64 },
    /// Filament pulled back (negative) or pushed again (positive), at `speed` mm/s.
    Retract { e: f64, speed: f64 },
    /// Part cooling fan, from 0 to 1.
    Fan { speed: f64 },
    /// Acceleration for the following moves, in mm/s².
    Accel { value: f64 },
    /// Progress shown by the printer: percent done and minutes left.
    Progress { percent: u32, minutes_left: u32 },
}

/// The output of the motion stage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Program {
    pub ops: Vec<Op>,
    pub summary: Summary,
}

/// What the program does, for people and for comparisons.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Summary {
    pub layers: usize,
    pub max_z: f64,
    /// Filament pushed, in mm.
    pub filament_mm: f64,
    /// Plastic laid per layer, in mm³.
    pub volume_per_layer: Vec<f64>,
    /// Time of the moves, in seconds (the machine's own start and end sequences not counted).
    pub motion_seconds: f64,
    /// Time of the moves up to the end of the first layer, in seconds.
    pub first_layer_seconds: f64,
    /// The first layer's extent `[min, max]`, for bed levelling.
    pub first_layer_bounds: [[f64; 2]; 2],
}

/// Speed and acceleration for one role.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoleMotion {
    pub speed: f64,
    pub accel: f64,
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

/// The variables of the machine's templates that the slicer computes, added to its settings.
pub type Placeholders = Scope;

#[cfg(test)]
mod tests;
