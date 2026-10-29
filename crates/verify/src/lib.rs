//! Stage 5: the guard between the compiler and the machine. A program leaves only if every
//! move it makes is safe.
//!
//! Checked for every move, compiled or read from another slicer: finite numbers; the whole
//! path inside the plate's printable area, an arc's sweep included, and below its printable
//! height; the head never down into what is printed before (`clearance`), the nozzle never
//! below the lowest layer, and extrusions on their layer's top, at both ends and halfway: at its
//! height on a flat layer, within a quarter of its thickness on a bent one, whose beads are
//! written within the printer's resolution; speeds within the machine's
//! limits, the Z axis's too, accelerations positive; filament flowing no faster than the hot end melts it; wipes
//! that move; arcs that end on their circle, at most a turn; fan between 0 and 1; waits finite
//! and not negative. The machine's own G-code between layers is the maker's and is not
//! interpreted: after it the nozzle may be anywhere, so a layer's first move must be a straight
//! travel to a named point, checked against the print from where the layer before left it.

mod clearance;

use clearance::{Heights, intrusion, lay};
use layercraft_motion::{Layer, Move, Program, disc_area};
use serde::{Deserialize, Serialize};

/// The toolhead's underside, about the nozzle's axis, in mm (`clearance`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Toolhead {
    /// The radius of the nozzle's flat tip.
    pub tip_radius: f64,
    /// How much wider the nozzle's cone grows per mm up: the tangent of its half-angle.
    pub flare: f64,
    /// How far the nozzle reaches below the rest of the head.
    pub nozzle_height: f64,
    /// How far around the nozzle the rest of the head reaches.
    pub head_radius: f64,
    /// How high the gantry stands above the tip, past the head.
    pub gantry_height: f64,
    /// The room kept under the head, past its tip, for beads that curl up.
    pub margin: f64,
}

/// What the machine can safely do.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Envelope {
    /// The printable area `[min, max]` of the plate, in mm.
    pub area: [[f64; 2]; 2],
    pub max_z: f64,
    pub max_speed: f64,
    /// The fastest the Z axis moves, in mm/s.
    pub max_speed_z: f64,
    pub filament_diameter: f64,
    /// The hot end's melt limit, in mm³/s.
    pub max_volumetric_speed: f64,
    pub toolhead: Toolhead,
}

/// One unsafe move.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Violation {
    pub layer: usize,
    /// The move's place in its layer.
    pub index: usize,
    pub reason: String,
}

/// Room for rounding in comparisons, in mm (or mm³/s for flow).
const SLACK: f64 = 1e-6;
/// Room for the rounding of positions in G-code, three decimals, in mm.
const WRITTEN: f64 = 2e-3;
/// The most violations listed.
const MAX_LISTED: usize = 50;
/// The most filament pulled or pushed in place, in mm.
const MAX_IN_PLACE: f64 = 20.0;

/// What is wrong with move `m` from `from` on `layer`, over the lowest layer at `bed`, if
/// anything.
fn check(m: &Move, from: Option<[f64; 3]>, layer: &Layer, bed: f64, e: &Envelope) -> Option<String> {
    let finite = m.e.is_finite() && m.speed.is_finite() && m.accel.is_finite() && m.fan.is_finite();
    if !finite || m.speed <= 0.0 || m.accel <= 0.0 {
        return Some(format!("move pushing {} mm at {} mm/s, {} mm/s²", m.e, m.speed, m.accel));
    }
    if !(m.wait.is_finite() && m.wait >= 0.0) {
        return Some(format!("a wait of {} s", m.wait));
    }
    if !(0.0..=1.0).contains(&m.fan) {
        return Some(format!("fan at {}", m.fan));
    }
    let Some(to) = m.to else {
        return (m.e.abs() > MAX_IN_PLACE).then(|| format!("{} mm of filament in place", m.e));
    };
    if !to.iter().all(|v| v.is_finite()) {
        return Some(format!("move to {to:?}"));
    }
    if m.speed > e.max_speed + SLACK {
        return Some(format!("{} mm/s is faster than the machine's {}", m.speed, e.max_speed));
    }
    if to[2] > e.max_z {
        return Some(format!("move to z = {} is above the printable height", to[2]));
    }
    if to[2] < bed - SLACK {
        return Some(format!("move to z = {} below the lowest layer at {bed}", to[2]));
    }
    let Some(from) = from else {
        // After the machine's own G-code: only a travel to a named point is safe.
        return (m.arc.is_some() || m.e != 0.0).then(|| "a move from where the nozzle is not known".into());
    };
    if let Some(arc) = m.arc {
        let end = m.point(from, 1.0);
        let off = (end[0] - to[0]).hypot(end[1] - to[1]);
        if !(arc.turn.is_finite() && arc.turn != 0.0 && arc.turn.abs() <= std::f64::consts::TAU + SLACK) {
            return Some(format!("an arc turning {} radians", arc.turn));
        }
        if !(off <= WRITTEN && arc.centre.iter().all(|v| v.is_finite())) {
            return Some(format!("an arc ending {off:.4} mm off its circle"));
        }
    }
    let [lo, hi] = m.bounds(from);
    if lo[0] < e.area[0][0] - SLACK
        || lo[1] < e.area[0][1] - SLACK
        || hi[0] > e.area[1][0] + SLACK
        || hi[1] > e.area[1][1] + SLACK
    {
        return Some(format!("move to ({:.3}, {:.3}) leaves the printable area", to[0], to[1]));
    }
    let length = m.length(from);
    let dz = (to[2] - from[2]).abs();
    if m.speed * dz > (e.max_speed_z + SLACK) * length {
        let vz = m.speed * dz / length;
        return Some(format!("{vz:.2} mm/s up or down is faster than the Z axis's {}", e.max_speed_z));
    }
    if m.e > 0.0 {
        let on = |p: [f64; 3]| layer.z + layer.warp.at([p[0], p[1]]);
        let half = m.point(from, 0.5);
        // On a flat layer, at its height; on a bent one, within a quarter of the layer's
        // thickness of its top, short of the layer below by far.
        let stray = if layer.warp.is_zero() { SLACK } else { layer.height / 4.0 };
        if (to[2] - on(to)).abs() > stray || (half[2] - on(half)).abs() > stray {
            return Some(format!("extrusion at z = {} on the layer whose top is at {}", to[2], on(to)));
        }
        let filament_area = disc_area(e.filament_diameter);
        let flow = if length > 0.0 { m.e * filament_area * m.speed / length } else { f64::INFINITY };
        if flow > e.max_volumetric_speed + SLACK {
            return Some(format!("{flow:.2} mm³/s is past the melt limit {}", e.max_volumetric_speed));
        }
    }
    if m.e < 0.0 && length <= SLACK {
        return Some("a wipe that does not move pulls filament in place".into());
    }
    None
}

/// Every unsafe move of the program; empty when it may be sent to the machine.
pub fn verify(program: &Program, envelope: &Envelope) -> Vec<Violation> {
    let mut found = Vec::new();
    let mut note = |layer: usize, index: usize, reason: String| {
        if found.len() < MAX_LISTED {
            found.push(Violation { layer, index, reason });
        }
    };
    let bed = program.layers.iter().map(|l| l.z).fold(f64::INFINITY, f64::min);
    let mut heights = Heights::new(envelope.area);
    // Where the layer before left the nozzle, which the machine's layer change keeps.
    let mut left: Option<[f64; 3]> = None;
    for (k, layer) in program.layers.iter().enumerate() {
        let z = layer.z;
        if !(z.is_finite() && layer.height.is_finite() && layer.height > 0.0 && z > 0.0) {
            note(k, 0, format!("layer at z = {z} with height {}", layer.height));
        }
        if z > envelope.max_z {
            note(k, 0, format!("layer at z = {z} is above the printable height {}", envelope.max_z));
        }
        let steepest = layer.warp.steepest();
        // After the machine's layer change the nozzle may be anywhere.
        let mut at: Option<[f64; 3]> = None;
        for (i, m) in layer.moves.iter().enumerate() {
            if let Some(reason) = check(m, at, layer, bed, envelope) {
                note(k, i, reason);
            }
            if let Some(from) = at.or(left)
                && m.to.is_some()
                && m.to.iter().flatten().all(|v| v.is_finite())
            {
                if let Some(depth) = intrusion(&heights, &envelope.toolhead, m, from, layer, steepest) {
                    let to = m.to.unwrap_or(from);
                    note(
                        k,
                        i,
                        format!(
                            "the head comes {depth:.3} mm down into what is printed, moving to ({:.3}, {:.3}, {:.3})",
                            to[0], to[1], to[2]
                        ),
                    );
                }
                lay(&mut heights, m, from);
            }
            if m.to.is_some() {
                at = m.to;
            }
        }
        left = at.or(left);
    }
    found
}

#[cfg(test)]
mod tests;
