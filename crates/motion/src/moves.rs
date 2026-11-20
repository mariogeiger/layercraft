//! The geometry of a move from where the nozzle is: its length, its directions at both ends,
//! the point at any fraction of it, the chords that follow it and the box it sweeps; and where a
//! pass's moves take the nozzle, each from the end of the one before.
//!
//! A line runs straight. An arc of radius `r = |from − centre|` turns `θ` radians about a
//! vertical axis while climbing `dz`, evenly: its length is `√((r·θ)² + dz²)`, its tangent at
//! angle `φ` runs along `(−r·θ·sin φ, r·θ·cos φ, dz)`, and a chord turning `δ` strays
//! `r·(1 − cos(δ/2))` from it.

use crate::{Arc, Move};

/// A point in the build volume, in mm.
pub type Point3 = [f64; 3];

/// The most chords an arc is cut into.
const MAX_CHORDS: f64 = 4096.0;

fn unit(v: Point3) -> Point3 {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n > 0.0 && n.is_finite() { [v[0] / n, v[1] / n, v[2] / n] } else { [0.0; 3] }
}

impl Move {
    /// For a move along an arc, the arc, its radius from `from` and the angle it starts at.
    fn polar(&self, from: Point3) -> Option<(Arc, f64, f64)> {
        let arc = self.arc?;
        let (x, y) = (from[0] - arc.centre[0], from[1] - arc.centre[1]);
        Some((arc, x.hypot(y), y.atan2(x)))
    }

    /// The length of the move's path from `from`; nothing for a move in place.
    pub fn length(&self, from: Point3) -> f64 {
        let Some(to) = self.to else { return 0.0 };
        let dz = to[2] - from[2];
        match self.polar(from) {
            Some((arc, r, _)) => (r * arc.turn).hypot(dz),
            None => ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2) + dz * dz).sqrt(),
        }
    }

    /// The radius the path turns on, infinite for a line.
    pub fn radius(&self, from: Point3) -> f64 {
        self.polar(from).map_or(f64::INFINITY, |(_, r, _)| r)
    }

    /// The point a fraction `s` (0 to 1) along the path from `from`.
    pub fn point(&self, from: Point3, s: f64) -> Point3 {
        let Some(to) = self.to else { return from };
        let z = from[2] + (to[2] - from[2]) * s;
        match self.polar(from) {
            Some((arc, r, a)) => {
                let angle = a + arc.turn * s;
                [arc.centre[0] + r * angle.cos(), arc.centre[1] + r * angle.sin(), z]
            }
            None => [from[0] + (to[0] - from[0]) * s, from[1] + (to[1] - from[1]) * s, z],
        }
    }

    /// How many equal chords follow the path from `from` within `sag` of it: one for a line or a
    /// move in place, and along an arc of radius `r` chords each turning at most `2·acos(1 − sag/r)`.
    pub fn chords(&self, from: Point3, sag: f64) -> usize {
        let (Some(_), Some((arc, r, _))) = (self.to, self.polar(from)) else { return 1 };
        let step = 2.0 * (1.0 - (sag / r).min(1.0)).acos();
        if step > 0.0 { ((arc.turn.abs() / step).ceil().clamp(1.0, MAX_CHORDS) as usize).max(1) } else { 1 }
    }

    /// The unit directions the path leaves `from` with and arrives with.
    pub fn directions(&self, from: Point3) -> (Point3, Point3) {
        let Some(to) = self.to else { return ([0.0; 3], [0.0; 3]) };
        let dz = to[2] - from[2];
        match self.polar(from) {
            Some((arc, r, a)) => {
                let s = r * arc.turn;
                let tangent = |angle: f64| unit([-angle.sin() * s, angle.cos() * s, dz]);
                (tangent(a), tangent(a + arc.turn))
            }
            None => {
                let d = unit([to[0] - from[0], to[1] - from[1], dz]);
                (d, d)
            }
        }
    }

    /// The box `[min, max]` across the plate that the path sweeps from `from`: its ends, and for
    /// an arc each axis-aligned extreme of its circle that the turn passes.
    pub fn bounds(&self, from: Point3) -> [[f64; 2]; 2] {
        let to = self.to.unwrap_or(from);
        let mut b = [[from[0].min(to[0]), from[1].min(to[1])], [from[0].max(to[0]), from[1].max(to[1])]];
        if let Some((arc, r, a)) = self.polar(from) {
            let (lo, hi) = if arc.turn >= 0.0 { (a, a + arc.turn) } else { (a + arc.turn, a) };
            let quarter = std::f64::consts::FRAC_PI_2;
            let mut k = (lo / quarter).ceil();
            while k * quarter <= hi && k * quarter - lo <= std::f64::consts::TAU + quarter {
                let angle = k * quarter;
                let p = [arc.centre[0] + r * angle.cos(), arc.centre[1] + r * angle.sin()];
                b = [[b[0][0].min(p[0]), b[0][1].min(p[1])], [b[1][0].max(p[0]), b[1][1].max(p[1])]];
                k += 1.0;
            }
        }
        b
    }
}

/// Where a pass's `moves` leave the nozzle, entered at `start`: the end of the last that goes
/// somewhere, else `start`.
pub fn end(moves: &[Move], start: Option<Point3>) -> Option<Point3> {
    moves.iter().rev().find_map(|m| m.to).or(start)
}

/// The length of each move of a pass entered at `start`.
pub fn lengths(moves: &[Move], start: Option<Point3>) -> Vec<f64> {
    chained(moves, start).map(|(from, m)| from.map_or(0.0, |f| m.length(f))).collect()
}

/// Each move of a pass with the point it starts from: `start` for the first, which is
/// unknown after the machine's own G-code, then each move's end.
pub fn chained(moves: &[Move], start: Option<Point3>) -> impl Iterator<Item = (Option<Point3>, &Move)> {
    moves.iter().scan(start, |at, m| {
        let from = *at;
        if let Some(to) = m.to {
            *at = Some(to);
        }
        Some((from, m))
    })
}
