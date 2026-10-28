//! Stage 5: the guard between the compiler and the machine. A program leaves only if every
//! move it makes is safe.
//!
//! Checked for every move, compiled or read from another slicer: finite numbers; the whole
//! path inside the plate's printable area, an arc's sweep included, and below its printable
//! height; the nozzle never below the layer it prints (extrusions at the layer's height, other
//! moves at or above it); layers rising; speeds within the machine's limits, accelerations
//! positive; filament flowing no faster than the hot end melts it; wipes that move; arcs that
//! end on their circle, at most a turn; fan between 0 and 1. The machine's own G-code between
//! layers is the maker's and is not interpreted: after it the nozzle may be anywhere, so a
//! layer's first move must be a straight travel to a named point.

use layercraft_motion::{Move, Program, disc_area};
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

/// What is wrong with move `m` from `from` on the layer at `z`, if anything.
fn check(m: &Move, from: Option<[f64; 3]>, z: f64, e: &Envelope) -> Option<String> {
    let finite = m.e.is_finite() && m.speed.is_finite() && m.accel.is_finite() && m.fan.is_finite();
    if !finite || m.speed <= 0.0 || m.accel <= 0.0 {
        return Some(format!("move pushing {} mm at {} mm/s, {} mm/s²", m.e, m.speed, m.accel));
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
    if to[2] < z - SLACK {
        return Some(format!("move to z = {} below the layer at {z}", to[2]));
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
    if m.e > 0.0 {
        if (to[2] - z).abs() > SLACK {
            return Some(format!("extrusion at z = {} on the layer at {z}", to[2]));
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
    let mut below: Option<f64> = None;
    for (k, layer) in program.layers.iter().enumerate() {
        let z = layer.z;
        if !(z.is_finite() && layer.height.is_finite() && layer.height > 0.0 && z > 0.0) {
            note(k, 0, format!("layer at z = {z} with height {}", layer.height));
        }
        if below.is_some_and(|b| z <= b) {
            note(k, 0, format!("layer at z = {z} does not rise above {}", below.unwrap_or(0.0)));
        }
        if z > envelope.max_z {
            note(k, 0, format!("layer at z = {z} is above the printable height {}", envelope.max_z));
        }
        below = Some(z);
        // After the machine's layer change the nozzle may be anywhere.
        let mut at: Option<[f64; 3]> = None;
        for (i, m) in layer.moves.iter().enumerate() {
            if let Some(reason) = check(m, at, z, envelope) {
                note(k, i, reason);
            }
            if m.to.is_some() {
                at = m.to;
            }
        }
    }
    found
}

#[cfg(test)]
mod tests;
