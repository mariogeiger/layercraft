//! The beads written as the fewest moves: each run of moves laying one bead (one width and
//! role, with one speed, acceleration and fan) becomes lines and, when allowed, arcs, each from a
//! vertex of the run to the farthest later one it fits, within the resolution `ε`.
//!
//! A line fits when every vertex between lies within `ε` of it, in space; so then does every
//! edge, the distance to a segment being convex. An arc runs through both ends across the plate,
//! climbing evenly, each vertex within `ε` of the height it passes it at; its centre `c = m + t·n`
//! on their bisector, `t` by least squares on `|q − c|² − r²` over the vertices between, linear
//! in `t`: the algebraic circle fit, held to its ends. It fits when `c` lies within `MAX_RADIUS`
//! and the vertices turn one way about it, by less than a turn, each vertex and edge within `ε`
//! of the circle: along an edge the distance to `c` is convex, so it strays most at its ends or
//! at its foot nearest `c`, and every ray from `c` meets arc and polyline within `ε`.
//!
//! A move lays the mean plastic per mm across the plate of the moves it replaces, and fits only
//! if each lays within `2ε/w` of it, `w` the bead's width: on a bent layer the slab, and so the
//! plastic, changes along a bead. The nozzle holding a bead's top, plastic amiss widens it: a
//! share `2ε/w` of a bead no larger than `w` by its thickness widens it by at most `2ε`, each
//! edge by `ε`, as the path strays. The machine's limits come after (`limit`).

use crate::moves::chained;
use crate::{Arc, Move, Program};
use std::f64::consts::TAU;

type P2 = [f64; 2];

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn norm(a: P2) -> f64 {
    a[0].hypot(a[1])
}

/// The way a move from the first vertex to the last of a run of them goes.
enum Shape {
    Line,
    Arc(Arc),
}

/// The widest circle an arc may run about, in mm. The machine works its arcs in single
/// precision, 24 bits, which hold a circle of 1 m to well under a micrometre; far wider, an arc
/// barely bows from its chord, and its centre, far off, is lost to rounding, here and there.
const MAX_RADIUS: f64 = 1000.0;

/// The arc from the first vertex to the last, if it lies within `eps` of the polyline.
fn arc(v: &[P2], eps: f64) -> Option<Arc> {
    let (&a, &b) = (v.first()?, v.last()?);
    let chord = sub(b, a);
    let l = norm(chord);
    if l <= 0.0 {
        return None;
    }
    let m = [chord[0] / 2.0, chord[1] / 2.0];
    let n = [-chord[1] / l, chord[0] / l];
    let (mut num, mut den) = (0.0, 0.0);
    for &p in v.get(1..v.len() - 1)? {
        let q = sub(p, a);
        let (ak, bk) = (dot(q, q) - 2.0 * dot(m, q), 2.0 * dot(n, q));
        num += ak * bk;
        den += bk * bk;
    }
    if den <= 0.0 {
        return None;
    }
    let t = num / den;
    let centre = [a[0] + m[0] + t * n[0], a[1] + m[1] + t * n[1]];
    let r = norm(sub(a, centre));
    if r > MAX_RADIUS {
        return None;
    }
    let mut turn = 0.0;
    for w in v.windows(2) {
        let (&p, &q) = (w.first()?, w.get(1)?);
        let (cp, cq) = (sub(p, centre), sub(q, centre));
        let step = (cp[0] * cq[1] - cp[1] * cq[0]).atan2(dot(cp, cq));
        if (norm(cp) - r).abs() > eps || step == 0.0 || step * turn < 0.0 {
            return None;
        }
        turn += step;
        let e = sub(q, p);
        let s = dot(sub(centre, p), e) / dot(e, e);
        if s > 0.0 && s < 1.0 && r - norm(sub([p[0] + s * e[0], p[1] + s * e[1]], centre)) > eps {
            return None;
        }
    }
    (turn.abs() < TAU).then_some(Arc { centre, turn })
}
type P3 = [f64; 3];

/// The distance from `p` to the segment from `a` to `b`, in space.
fn distance3(p: P3, a: P3, b: P3) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let q = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    let t = if len2 > 0.0 { ((q[0] * d[0] + q[1] * d[1] + q[2] * d[2]) / len2).clamp(0.0, 1.0) } else { 0.0 };
    ((q[0] - t * d[0]).powi(2) + (q[1] - t * d[1]).powi(2) + (q[2] - t * d[2]).powi(2)).sqrt()
}

/// The line from the first vertex to the last, if every vertex lies within `eps` of it.
fn line3(v: &[P3], eps: f64) -> bool {
    let (Some(&a), Some(&b)) = (v.first(), v.last()) else { return false };
    v.iter().all(|&p| distance3(p, a, b) <= eps)
}

/// Whether the vertices' heights lie within `eps` of the helix that climbs evenly along the arc
/// from the first to the last.
fn climbs(w: &[P3], arc: &Arc, eps: f64) -> bool {
    let (Some(&a), Some(&b)) = (w.first(), w.last()) else { return false };
    let start = (a[1] - arc.centre[1]).atan2(a[0] - arc.centre[0]);
    w.iter().all(|p| {
        let angle = (p[1] - arc.centre[1]).atan2(p[0] - arc.centre[0]);
        // The share of the turn made by the vertex, the turn's sign taken.
        let mut swept = angle - start;
        if arc.turn > 0.0 {
            swept = swept.rem_euclid(TAU);
        } else {
            swept = -(-swept).rem_euclid(TAU);
        }
        let share = if arc.turn != 0.0 { (swept / arc.turn).clamp(0.0, 1.0) } else { 0.0 };
        (p[2] - (a[2] + share * (b[2] - a[2]))).abs() <= eps
    })
}

/// What one move of a run lays: its plastic and its length across the plate, both positive.
#[derive(Clone, Copy)]
struct Piece {
    e: f64,
    plate: f64,
}

impl Piece {
    /// What a move from `from` lays, if it lays a bead along a line.
    fn of(from: Option<P3>, m: &Move) -> Option<Piece> {
        let (from, to) = (from?, m.to?);
        let plate = (to[0] - from[0]).hypot(to[1] - from[1]);
        let piece = Piece { e: m.e, plate };
        (m.bead.is_some() && m.arc.is_none() && m.e > 0.0 && plate > 0.0).then_some(piece)
    }
}

/// A run of moves laying one bead the same way: the first move, the run's vertices from where it
/// starts, and what the move ending at each later vertex lays.
struct Run {
    first: Move,
    vertices: Vec<P3>,
    pieces: Vec<Piece>,
}

impl Run {
    /// The move along `shape` in place of the run's moves from vertex `i` to vertex `j`, if
    /// each lays within `spread` of it, as a share of its own.
    fn replace(&self, i: usize, j: usize, shape: &Shape, spread: f64) -> Option<Move> {
        let (&from, &to) = (self.vertices.get(i)?, self.vertices.get(j)?);
        let pieces = self.pieces.get(i..j)?;
        let (arc, plate) = match shape {
            Shape::Line => (None, norm(sub([to[0], to[1]], [from[0], from[1]]))),
            Shape::Arc(a) => (Some(*a), norm(sub([from[0], from[1]], a.centre)) * a.turn.abs()),
        };
        let covered: f64 = pieces.iter().map(|p| p.plate).sum();
        if covered <= 0.0 {
            return None;
        }
        let per_mm = pieces.iter().map(|p| p.e).sum::<f64>() / covered;
        let near = |p: &Piece| (p.e / p.plate - per_mm).abs() <= spread * p.e / p.plate;
        pieces.iter().all(near).then(|| Move { to: Some(to), arc, e: per_mm * plate, ..self.first.clone() })
    }

    /// The fewest moves through the run's vertices, each within `eps` of the run; arcs too, with
    /// `arcs`.
    fn fit(&self, eps: f64, arcs: bool, out: &mut Vec<Move>) {
        let v = &self.vertices;
        let spread = self.first.bead.map_or(0.0, |b| 2.0 * eps / b.width);
        let shape = |w: &[P3]| {
            if line3(w, eps) {
                Some(Shape::Line)
            } else if arcs {
                let plane: Vec<P2> = w.iter().map(|p| [p[0], p[1]]).collect();
                arc(&plane, eps).filter(|a| climbs(w, a, eps)).map(Shape::Arc)
            } else {
                None
            }
        };
        let fitted =
            |i: usize, j: usize| v.get(i..=j).and_then(shape).and_then(|s| self.replace(i, j, &s, spread));
        let mut i = 0;
        while i + 1 < v.len() {
            // A move alone stays as it is.
            let Some(mut best) = self.replace(i, i + 1, &Shape::Line, f64::INFINITY) else { break };
            let mut j = i + 1;
            while let Some(m) = fitted(i, j + 1) {
                (j, best) = (j + 1, m);
            }
            out.push(best);
            i = j;
        }
    }
}

/// Writes every run of moves laying one bead the same way as the fewest lines (and arcs, with
/// `arcs`) within `resolution` mm of it; a resolution of 0 leaves the program as it is.
pub fn fit(program: &mut Program, resolution: f64, arcs: bool) {
    if !(resolution > 0.0 && resolution.is_finite()) {
        return;
    }
    let mut at = None;
    for layer in &mut program.layers {
        let mut out = Vec::with_capacity(layer.moves.len());
        let mut run: Option<Run> = None;
        let start = at;
        at = layer.moves.iter().rev().find_map(|m| m.to).or(at);
        for (from, m) in chained(&layer.moves, start) {
            let piece = Piece::of(from, m);
            let joins = match (&run, piece) {
                (Some(r), Some(_)) => {
                    let f = &r.first;
                    (f.speed, f.accel, f.fan, f.bead) == (m.speed, m.accel, m.fan, m.bead)
                }
                _ => false,
            };
            if !joins {
                if let Some(r) = run.take() {
                    r.fit(resolution, arcs, &mut out);
                }
                if let (Some(_), Some(from)) = (piece, from) {
                    run = Some(Run { first: m.clone(), vertices: vec![from], pieces: Vec::new() });
                }
            }
            match (&mut run, m.to, piece) {
                (Some(r), Some(to), Some(p)) => {
                    r.vertices.push(to);
                    r.pieces.push(p);
                }
                _ => out.push(m.clone()),
            }
        }
        if let Some(r) = run.take() {
            r.fit(resolution, arcs, &mut out);
        }
        layer.moves = out;
    }
}
