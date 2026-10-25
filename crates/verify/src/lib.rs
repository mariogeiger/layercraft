//! Stage 5: the guard between the compiler and the machine. A program leaves only if every
//! move it makes is safe.
//!
//! Checked for every move layercraft generates: finite numbers; positions inside the plate's
//! printable area and below its printable height; the nozzle never below the layer it prints
//! (extrusions at the layer's height, travels at or above it); layers rising; speeds within the
//! machine's limits; filament flowing no faster than the hot end melts it; fan between 0 and 1.
//! The machine's own G-code (start, end, layer change) is the maker's and is not interpreted.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use layercraft_motion::{Op, Program};
use serde::{Deserialize, Serialize};

/// What the machine can safely do.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Envelope {
    /// The printable area `[min, max]` of the plate, in mm.
    pub area: [[f64; 2]; 2],
    pub max_z: f64,
    pub max_speed: f64,
    pub filament_diameter: f64,
    /// The hot end's melt limit, in mm³/s.
    pub max_volumetric_speed: f64,
}

/// One unsafe instruction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Violation {
    /// The position of the instruction in the program.
    pub op: usize,
    pub reason: String,
}

/// Room for rounding in comparisons, in mm (or mm³/s for flow).
const SLACK: f64 = 1e-6;
/// The most violations listed.
const MAX_LISTED: usize = 50;

/// Every unsafe instruction of the program; empty when it may be sent to the machine.
pub fn verify(program: &Program, envelope: &Envelope) -> Vec<Violation> {
    let mut found = Vec::new();
    let mut note = |op: usize, reason: String| {
        if found.len() < MAX_LISTED {
            found.push(Violation { op, reason });
        }
    };
    let filament_area = std::f64::consts::PI * (envelope.filament_diameter / 2.0).powi(2);
    let mut layer_z: Option<f64> = None;
    let mut at: Option<[f64; 3]> = None;
    let inside = |p: &[f64; 3]| {
        p[0] >= envelope.area[0][0] - SLACK
            && p[0] <= envelope.area[1][0] + SLACK
            && p[1] >= envelope.area[0][1] - SLACK
            && p[1] <= envelope.area[1][1] + SLACK
    };
    for (i, op) in program.ops.iter().enumerate() {
        match op {
            Op::Layer { z, height, .. } => {
                if !(z.is_finite() && height.is_finite() && *height > 0.0 && *z > 0.0) {
                    note(i, format!("layer at z = {z} with height {height}"));
                }
                if let Some(previous) = layer_z
                    && *z <= previous
                {
                    note(i, format!("layer at z = {z} does not rise above {previous}"));
                }
                if *z > envelope.max_z {
                    note(i, format!("layer at z = {z} is above the printable height {}", envelope.max_z));
                }
                layer_z = Some(*z);
            }
            Op::Travel { to, speed } | Op::Extrude { to, speed, .. } => {
                if !to.iter().all(|v| v.is_finite()) || !speed.is_finite() || *speed <= 0.0 {
                    note(i, format!("move to {to:?} at {speed} mm/s is not finite"));
                    continue;
                }
                if !inside(to) {
                    note(i, format!("move to ({:.3}, {:.3}) leaves the printable area", to[0], to[1]));
                }
                if to[2] > envelope.max_z {
                    note(i, format!("move to z = {} is above the printable height", to[2]));
                }
                if *speed > envelope.max_speed + SLACK {
                    note(i, format!("{speed} mm/s is faster than the machine's {}", envelope.max_speed));
                }
                match (op, layer_z) {
                    (_, None) => note(i, "a move before the first layer".into()),
                    (Op::Extrude { .. }, Some(z)) if (to[2] - z).abs() > SLACK => {
                        note(i, format!("extrusion at z = {} on the layer at {z}", to[2]));
                    }
                    (Op::Travel { .. }, Some(z)) if to[2] < z - SLACK => {
                        note(i, format!("travel at z = {} below the layer at {z}", to[2]));
                    }
                    _ => {}
                }
                if let (Op::Extrude { e, .. }, Some(from)) = (op, at) {
                    let length = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2)).sqrt();
                    if !e.is_finite() || *e < 0.0 {
                        note(i, format!("extrusion of {e} mm of filament"));
                    } else if length > 0.0 {
                        let flow = e * filament_area * speed / length;
                        if flow > envelope.max_volumetric_speed + SLACK {
                            note(
                                i,
                                format!(
                                    "{flow:.2} mm³/s is past the melt limit {}",
                                    envelope.max_volumetric_speed
                                ),
                            );
                        }
                    }
                }
                at = Some(*to);
            }
            Op::Lift { z, speed } => {
                if !(z.is_finite() && speed.is_finite() && *speed > 0.0) || *z > envelope.max_z {
                    note(i, format!("lift to z = {z} at {speed} mm/s"));
                }
                if let (Some(layer), true) = (layer_z, z.is_finite())
                    && *z < layer - SLACK
                {
                    note(i, format!("lift down to z = {z} below the layer at {layer}"));
                }
                at = at.map(|p| [p[0], p[1], *z]);
            }
            Op::Retract { e, speed } => {
                if !(e.is_finite() && speed.is_finite() && *speed > 0.0) || e.abs() > 20.0 {
                    note(i, format!("retraction of {e} mm at {speed} mm/s"));
                }
            }
            Op::Fan { speed } => {
                if !(speed.is_finite() && (0.0..=1.0).contains(speed)) {
                    note(i, format!("fan at {speed}"));
                }
            }
            Op::Accel { value } => {
                if !(value.is_finite() && *value > 0.0) {
                    note(i, format!("acceleration {value}"));
                }
            }
            Op::Raw { text } => {
                if text.contains('\0') {
                    note(i, "machine G-code holds a NUL byte".into());
                }
            }
            Op::Feature { .. } | Op::Progress { .. } => {}
        }
    }
    found
}

#[cfg(test)]
mod tests;
