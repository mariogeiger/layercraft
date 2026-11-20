//! Stage 5: the guard between the compiler and the machine. A program leaves only if every
//! move it makes is safe.
//!
//! Checked for every move, compiled or read from another slicer: finite numbers; the whole
//! path inside the plate's printable area, an arc's sweep included, and below its printable
//! height; the head never down into what is printed before (`clearance`), the nozzle never
//! below the lowest layer, and extrusions on their layer's top, at both ends and halfway: at its
//! height on a flat layer, within a quarter of its thickness on a bent one, whose beads are
//! written within the printer's resolution; on a free layer, each bead resting on the plate or on
//! a bead of a layer before, one to two of its heights below its tip; speeds within the machine's
//! limits, the Z axis's too, accelerations positive; filament flowing no faster than the hot end
//! melts the pass's own (`Filament`); wipes that move; arcs that end on their circle, at most a
//! turn; fan between 0 and 1; waits finite and not negative. The machine's own G-code between
//! passes is the maker's and is not interpreted: after it the nozzle may be anywhere, so a pass's
//! first move must be a straight travel to a named point, checked against the print from where
//! the pass before left it.

mod beads;
mod clearance;

use beads::{Allowed, Beads};
use clearance::intrusion;
use layercraft_geom::Relief;
use layercraft_motion::{Filament, Move, POSITION_UNIT, Pass, Program, disc_area};
use serde::{Deserialize, Serialize};

/// The toolhead's underside, about the nozzle's axis, in mm (`clearance`): the nozzle, a flat
/// face whose flank widens up to a column, reaching below the rest of the head, whose underside
/// is taken, with all that lies past it, as one plane.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Toolhead {
    /// The radius of the nozzle's flat face.
    pub face_radius: f64,
    /// How steeply the nozzle's flank rises from its face to its column: the tangent of its
    /// angle above the plate.
    pub flank_slope: f64,
    /// The radius of the nozzle's column.
    pub column_radius: f64,
    /// How far the nozzle reaches below the plane of the rest of the head.
    pub protrusion: f64,
    /// The room kept under that plane, for beads that curl up and a plate not quite flat.
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
    pub toolhead: Toolhead,
    /// How deep the nozzle's face may iron the beads of the layer it prints, in mm: on a bent
    /// layer it passes over those it laid just before, up the slope.
    pub ironing: f64,
    /// How far from its layer's top a move is written, in space, in mm: the motion stage's fit.
    pub resolution: f64,
}

/// One unsafe move.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Violation {
    /// The layer's index.
    pub layer: usize,
    /// The pass's place in the program, and the move's in its pass.
    pub pass: usize,
    pub index: usize,
    pub reason: String,
}

/// Room for rounding in comparisons, in mm (or mm³/s for flow).
const SLACK: f64 = 1e-6;
/// Room for the rounding of positions in G-code, in mm: two units of their last decimal.
const WRITTEN: f64 = 2.0 * POSITION_UNIT;
/// The most violations listed.
const MAX_LISTED: usize = 50;
/// The most filament pulled or pushed in place, in mm.
const MAX_IN_PLACE: f64 = 20.0;

/// The pass a move is checked on: its top lies `relief` over its plane, the lowest layer at
/// `bed`, its filament `filament`, within the machine's `envelope`.
struct On<'a> {
    pass: &'a Pass,
    relief: &'a Relief,
    bed: f64,
    filament: &'a Filament,
    envelope: &'a Envelope,
}

/// What is wrong with move `m` from `from` `on` its pass, among `beads`, if anything.
fn check(m: &Move, from: Option<[f64; 3]>, on: &On, beads: &Beads) -> Option<String> {
    let On { pass, relief, bed, filament, envelope: e } = *on;
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
        let half = m.point(from, 0.5);
        if pass.free {
            // A free bead rests on the plate or on the beads of the layers before, its tip one
            // bead's height above what it rests on, or up to two where the bead is stretched
            // over a gap.
            let rest = |p: [f64; 3]| p[2] - beads.support([p[0], p[1]], pass.index).unwrap_or(0.0);
            let stand = [pass.height - WRITTEN, 2.0 * pass.height + WRITTEN];
            for p in [to, half] {
                if !(stand[0]..=stand[1]).contains(&rest(p)) {
                    return Some(format!(
                        "free extrusion at z = {} stands {:.3} mm over what it rests on, not one to two of its {} mm",
                        p[2],
                        rest(p),
                        pass.height
                    ));
                }
            }
        } else {
            let on = |p: [f64; 3]| pass.z + relief.at([p[0], p[1]]);
            // On a flat layer, at its height; on a bent one, within a quarter of the layer's
            // thickness of its top, short of the layer below by far.
            let stray = if relief.is_zero() { SLACK } else { pass.height / 4.0 };
            if (to[2] - on(to)).abs() > stray || (half[2] - on(half)).abs() > stray {
                return Some(format!("extrusion at z = {} on the layer whose top is at {}", to[2], on(to)));
            }
        }
        let filament_area = disc_area(filament.diameter);
        let flow = if length > 0.0 { m.e * filament_area * m.speed / length } else { f64::INFINITY };
        if flow > filament.max_volumetric_speed + SLACK {
            return Some(format!("{flow:.2} mm³/s is past the melt limit {}", filament.max_volumetric_speed));
        }
    }
    if m.e < 0.0 && length <= SLACK {
        return Some("a wipe that does not move pulls filament in place".into());
    }
    None
}

/// Every unsafe move of the program, each pass's held to its material's `filaments`; empty when
/// it may be sent to the machine.
pub fn verify(program: &Program, envelope: &Envelope, filaments: &[Filament]) -> Vec<Violation> {
    let mut found = Vec::new();
    let mut note = |(k, pass): (usize, &Pass), index: usize, reason: String| {
        if found.len() < MAX_LISTED {
            found.push(Violation { layer: pass.index, pass: k, index, reason });
        }
    };
    let bed = program.passes.iter().map(|l| l.z).fold(f64::INFINITY, f64::min);
    // The lowest the tip comes on each pass, where a move ends or the pass before left it, and
    // from each pass on.
    let mut last = f64::INFINITY;
    let lows: Vec<f64> = program
        .passes
        .iter()
        .map(|l| {
            // A free bead may rest on one up to two of its heights below its tip.
            let reach = if l.free { 2.0 * l.height } else { 0.0 };
            let low = l.moves.iter().filter_map(|m| m.to).map(|to| to[2] - reach).fold(last, f64::min);
            last = l.moves.iter().rev().find_map(|m| m.to).map_or(last, |to| to[2]);
            low
        })
        .collect();
    let mut later = f64::INFINITY;
    let mut lowest: Vec<f64> = lows
        .iter()
        .rev()
        .map(|&low| {
            later = later.min(low);
            later
        })
        .collect();
    lowest.reverse();
    let mut beads = Beads::new();
    // Where the layer before left the nozzle, which the machine's layer change keeps.
    let mut left: Option<[f64; 3]> = None;
    for (k, pass) in program.passes.iter().enumerate() {
        if let Some(&low) = lowest.get(k) {
            beads.forget_below(low);
        }
        // The layer's top over its plane, its warp on the program's carrier; a warp that does not
        // fit the carrier leaves the layer's beads nothing to lie on.
        let relief = program.carrier.relief(&pass.warp).unwrap_or_else(|e| {
            note((k, pass), 0, format!("the layer's warp: {e}"));
            Relief::zero()
        });
        let Some(filament) = filaments.get(pass.material) else {
            note((k, pass), 0, format!("no filament given for material {}", pass.material));
            continue;
        };
        // A bent layer's moves are written within the resolution of its top in space, so in
        // height within that times `√(1 + G²)`, `G` its steepest lean; a flat one's on it.
        let lift =
            if relief.is_zero() { 0.0 } else { envelope.resolution.max(0.0) * relief.steepest().hypot(1.0) };
        let allowed = Allowed {
            head: &envelope.toolhead,
            layer: pass.index,
            lift,
            ironing: envelope.ironing.max(0.0),
            written: WRITTEN,
        };
        let z = pass.z;
        if !(z.is_finite() && pass.height.is_finite() && pass.height > 0.0 && z > 0.0) {
            note((k, pass), 0, format!("layer at z = {z} with height {}", pass.height));
        }
        if z > envelope.max_z {
            note((k, pass), 0, format!("layer at z = {z} is above the printable height {}", envelope.max_z));
        }
        // After the machine's layer change the nozzle may be anywhere.
        let mut at: Option<[f64; 3]> = None;
        for (i, m) in pass.moves.iter().enumerate() {
            // A free layer's beads stand one bead's height over the plate at the lowest.
            let floor = if pass.free { bed.min(pass.height) } else { bed };
            let on = On { pass, relief: &relief, bed: floor, filament, envelope };
            if let Some(reason) = check(m, at, &on, &beads) {
                note((k, pass), i, reason);
            }
            if let Some(from) = at.or(left)
                && let Some(to) = m.to
                && to.iter().all(|v| v.is_finite())
            {
                if let Some((depth, own)) = intrusion(&beads, &allowed, m, from) {
                    let what = if own {
                        format!(
                            "the nozzle irons its own layer {depth:.3} mm deeper than the {} allowed",
                            allowed.ironing
                        )
                    } else {
                        format!("the head comes {depth:.3} mm down into what is printed")
                    };
                    note(
                        (k, pass),
                        i,
                        format!("{what}, moving to ({:.3}, {:.3}, {:.3})", to[0], to[1], to[2]),
                    );
                }
                beads.lay(m, from, pass.index, lift);
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
