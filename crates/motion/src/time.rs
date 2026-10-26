//! Print time from a look-ahead planner: trapezoidal speed profiles linked at corners.
//!
//! Each move accelerates at most `a` toward its speed and must be able to stop by the end of
//! the program. The speed through a corner between unit directions `u₁` and `u₂` is limited so
//! the velocity jumps by at most `jerk`: `v · |u₁ − u₂| ≤ jerk`. A backward pass caps each
//! entry speed by what the rest can brake from, a forward pass by what can be reached; each
//! move then takes the time of its trapezoid. A move along an arc of radius `r` enters and
//! leaves along its tangents and runs no faster than its centripetal acceleration allows,
//! `v²/r ≤ a`. A move in place takes its filament's length over its speed, the nozzle at rest.

use crate::Program;
use crate::moves::chained;
use serde::{Deserialize, Serialize};

/// The machine's motion limits.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    pub max_accel: f64,
    pub jerk: f64,
    pub max_speed_xy: f64,
    pub max_speed_z: f64,
}

struct Segment {
    /// Where this move's finishing time goes in the list of times.
    slot: usize,
    length: f64,
    /// The directions it enters and leaves with.
    dir: ([f64; 3], [f64; 3]),
    vmax: f64,
    accel: f64,
}

/// The time of a move of `length` entered at `ve`, left at `vx`, cruising at most `vmax`.
fn trapezoid(length: f64, ve: f64, vx: f64, vmax: f64, a: f64) -> f64 {
    let peak2 = (2.0 * a * length + ve * ve + vx * vx) / 2.0;
    let peak = peak2.max(0.0).sqrt();
    if peak <= vmax {
        ((peak - ve).max(0.0) + (peak - vx).max(0.0)) / a
    } else {
        let ramps = (2.0 * vmax * vmax - ve * ve - vx * vx) / (2.0 * a);
        ((vmax - ve) + (vmax - vx)) / a + (length - ramps).max(0.0) / vmax
    }
}

fn run_time(segments: &[Segment], jerk: f64) -> f64 {
    let n = segments.len();
    if n == 0 {
        return 0.0;
    }
    // Junction limits: rest at both ends of the run.
    let mut junction = vec![0.0; n + 1];
    for i in 1..n {
        let (Some(a), Some(b)) = (segments.get(i - 1), segments.get(i)) else { continue };
        let (u, v) = (a.dir.1, b.dir.0);
        let du = ((u[0] - v[0]).powi(2) + (u[1] - v[1]).powi(2) + (u[2] - v[2]).powi(2)).sqrt();
        let corner = if du < 1e-9 { f64::INFINITY } else { jerk / du };
        if let Some(j) = junction.get_mut(i) {
            *j = corner.min(a.vmax).min(b.vmax);
        }
    }
    for i in (0..n).rev() {
        let (Some(s), Some(&next)) = (segments.get(i), junction.get(i + 1)) else { continue };
        let reachable = (next * next + 2.0 * s.accel * s.length).sqrt();
        if let Some(j) = junction.get_mut(i) {
            *j = j.min(reachable);
        }
    }
    for i in 0..n {
        let (Some(s), Some(&prev)) = (segments.get(i), junction.get(i)) else { continue };
        let reachable = (prev * prev + 2.0 * s.accel * s.length).sqrt();
        if let Some(j) = junction.get_mut(i + 1) {
            *j = j.min(reachable);
        }
    }
    segments
        .iter()
        .enumerate()
        .map(|(i, s)| {
            let ve = junction.get(i).copied().unwrap_or(0.0);
            let vx = junction.get(i + 1).copied().unwrap_or(0.0);
            trapezoid(s.length, ve, vx, s.vmax, s.accel)
        })
        .sum()
}

/// The time of the program at the end of each move, layer after layer, in seconds; the last
/// is the total. The machine's own G-code between layers takes no time here.
pub fn estimate(program: &Program, limits: &Limits) -> Vec<f64> {
    let mut times = Vec::new();
    let mut done = 0.0;
    let mut run: Vec<Segment> = Vec::new();
    let flush = |run: &mut Vec<Segment>, times: &mut Vec<f64>, done: &mut f64| {
        // Spread the run's time over its moves by length, which is exact at the run's end.
        let total = run_time(run, limits.jerk);
        let length: f64 = run.iter().map(|s| s.length).sum();
        let mut acc = *done;
        for s in run.iter() {
            acc += if length > 0.0 { total * s.length / length } else { 0.0 };
            if let Some(t) = times.get_mut(s.slot) {
                *t = acc;
            }
        }
        *done += total;
        run.clear();
    };
    let mut at = [0.0; 3];
    for layer in &program.layers {
        // The machine's layer change stops the nozzle.
        flush(&mut run, &mut times, &mut done);
        for (from, m) in chained(&layer.moves, Some(at)) {
            let from = from.unwrap_or(at);
            let Some(to) = m.to else {
                flush(&mut run, &mut times, &mut done);
                done += if m.speed > 0.0 { m.e.abs() / m.speed } else { 0.0 };
                times.push(done);
                continue;
            };
            at = to;
            let length = m.length(from);
            if length <= 0.0 || !length.is_finite() {
                times.push(done);
                continue;
            }
            let dir = m.directions(from);
            let accel = m.accel.min(limits.max_accel).max(1.0);
            let mut vmax = m.speed.min(limits.max_speed_xy).min((accel * m.radius(from)).sqrt());
            let climb = dir.0[2].abs().max(dir.1[2].abs());
            if climb > 1e-9 {
                vmax = vmax.min(limits.max_speed_z / climb);
            }
            run.push(Segment { slot: times.len(), length, dir, vmax: vmax.max(1e-3), accel });
            times.push(done);
        }
    }
    flush(&mut run, &mut times, &mut done);
    // Moves after a run's last one carry the time reached so far.
    let mut best = 0.0f64;
    for t in times.iter_mut() {
        best = best.max(*t);
        *t = best;
    }
    times
}
