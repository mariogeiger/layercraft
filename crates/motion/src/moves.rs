//! The geometry of a move from where the nozzle is: its length, its directions at both ends,
//! the point at any fraction of it, and the box it sweeps.
//!
//! A line runs straight. An arc of radius `r = |from − centre|` turns `θ` radians about a
//! vertical axis while climbing `dz`, evenly: its length is `√((r·θ)² + dz²)`, and its
//! tangent at angle `φ` is `(∓sin φ, ±cos φ)` across the plate, the sign of `θ` choosing the
//! way round, with the climb spread over the length.

use crate::Move;

/// A point in the build volume, in mm.
pub type Point3 = [f64; 3];

fn unit(v: Point3) -> Point3 {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n > 0.0 && n.is_finite() { [v[0] / n, v[1] / n, v[2] / n] } else { [0.0; 3] }
}

impl Move {
    /// The arc's radius and starting angle from `from`, for a move along an arc.
    fn polar(&self, from: Point3) -> Option<(f64, f64, f64)> {
        let arc = self.arc?;
        let (x, y) = (from[0] - arc.centre[0], from[1] - arc.centre[1]);
        Some((x.hypot(y), y.atan2(x), arc.turn))
    }

    /// The length of the move's path from `from`; nothing for a move in place.
    pub fn length(&self, from: Point3) -> f64 {
        let Some(to) = self.to else { return 0.0 };
        let dz = to[2] - from[2];
        match self.polar(from) {
            Some((r, _, turn)) => (r * turn).hypot(dz),
            None => ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2) + dz * dz).sqrt(),
        }
    }

    /// The radius the path turns on, infinite for a line.
    pub fn radius(&self, from: Point3) -> f64 {
        self.polar(from).map_or(f64::INFINITY, |(r, _, _)| r)
    }

    /// The point a fraction `s` (0 to 1) along the path from `from`.
    pub fn point(&self, from: Point3, s: f64) -> Point3 {
        let Some(to) = self.to else { return from };
        let z = from[2] + (to[2] - from[2]) * s;
        match (self.polar(from), self.arc) {
            (Some((r, a, turn)), Some(arc)) => {
                let angle = a + turn * s;
                [arc.centre[0] + r * angle.cos(), arc.centre[1] + r * angle.sin(), z]
            }
            _ => [from[0] + (to[0] - from[0]) * s, from[1] + (to[1] - from[1]) * s, z],
        }
    }

    /// The unit directions the path leaves `from` with and arrives with.
    pub fn directions(&self, from: Point3) -> (Point3, Point3) {
        let Some(to) = self.to else { return ([0.0; 3], [0.0; 3]) };
        let dz = to[2] - from[2];
        match self.polar(from) {
            Some((r, a, turn)) => {
                let tangent = |angle: f64| {
                    let s = turn.signum() * r * turn.abs();
                    unit([-angle.sin() * s, angle.cos() * s, dz])
                };
                (tangent(a), tangent(a + turn))
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
        if let (Some((r, a, turn)), Some(arc)) = (self.polar(from), self.arc) {
            let (lo, hi) = if turn >= 0.0 { (a, a + turn) } else { (a + turn, a) };
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

/// The length of each move of a layer from `at`, where the nozzle starts it, which then becomes
/// where the layer leaves the nozzle.
pub fn lengths(moves: &[Move], at: &mut Option<Point3>) -> Vec<f64> {
    let lengths = chained(moves, *at).map(|(from, m)| from.map_or(0.0, |f| m.length(f))).collect();
    *at = moves.iter().rev().find_map(|m| m.to).or(*at);
    lengths
}

/// Each move of a layer with the point it starts from: `start` for the first, which is
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
