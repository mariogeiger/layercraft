//! The beads written as the fewest moves: each run of moves laying one bead the same way (at one
//! height, with one speed, acceleration, fan and plastic per mm) becomes lines and, when allowed,
//! arcs, each from a vertex of the run to a later one and within the resolution `ε` of the run.
//!
//! From a vertex, a move reaches the farthest vertex it can. A line fits when every vertex
//! between lies within `ε` of it; so then does every edge, the distance to a segment being
//! convex. An arc runs through both ends, its centre `c = m + t·n` on their bisector, with `t` by
//! least squares on `|q − c|² − r²` over the vertices between, which is linear in `t`: the
//! algebraic circle fit, held to its ends. It fits when the vertices turn one way about `c`, by
//! less than a turn, each within `ε` of the circle, and each edge too: along an edge the distance
//! to `c` is convex, so it strays most at its ends or at its foot nearest `c`. Every ray from `c`
//! then meets the arc and the polyline within `ε` of each other: the arc strays no more from the
//! polyline than it from the arc. A move keeps its plastic per mm, which its length multiplies.

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

/// The distance from `p` to the segment `a`–`b`.
fn to_segment(p: P2, a: P2, b: P2) -> f64 {
    let e = sub(b, a);
    let l2 = dot(e, e);
    let s = if l2 > 0.0 { (dot(sub(p, a), e) / l2).clamp(0.0, 1.0) } else { 0.0 };
    norm(sub(p, [a[0] + s * e[0], a[1] + s * e[1]]))
}

/// The line from the first vertex to the last, if every vertex lies within `eps` of it.
fn line(v: &[P2], eps: f64) -> bool {
    let (Some(&a), Some(&b)) = (v.first(), v.last()) else { return false };
    v.iter().all(|&p| to_segment(p, a, b) <= eps)
}

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

/// The fewest moves through the vertices `v` of a run laying `bead`'s plastic per mm at `z`,
/// each within `eps` of the run.
fn fit_run(v: &[P2], z: f64, bead: &Move, per_mm: f64, eps: f64, arcs: bool, out: &mut Vec<Move>) {
    let shape = |w: &[P2]| {
        if line(w, eps) {
            Some(Shape::Line)
        } else if arcs {
            arc(w, eps).map(Shape::Arc)
        } else {
            None
        }
    };
    let mut i = 0;
    while i + 1 < v.len() {
        let (mut j, mut best) = (i + 1, Shape::Line);
        while let Some(s) = v.get(i..j + 2).and_then(shape) {
            (j, best) = (j + 1, s);
        }
        let (Some(&from), Some(&to)) = (v.get(i), v.get(j)) else { break };
        let (arc, length) = match best {
            Shape::Line => (None, norm(sub(to, from))),
            Shape::Arc(a) => (Some(a), norm(sub(from, a.centre)) * a.turn.abs()),
        };
        out.push(Move { to: Some([to[0], to[1], z]), arc, e: per_mm * length, ..bead.clone() });
        i = j;
    }
}

/// A move that lays a bead along a line at one height, with its plastic per mm and the height.
fn bead_line(from: Option<[f64; 3]>, m: &Move) -> Option<(f64, f64)> {
    let (from, to) = (from?, m.to?);
    let length = (to[0] - from[0]).hypot(to[1] - from[1]);
    (m.bead.is_some() && m.arc.is_none() && m.e > 0.0 && from[2] == to[2] && length > 0.0)
        .then(|| (m.e / length, to[2]))
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
        // The run so far: its vertices, its first move, its plastic per mm and height.
        let mut run: Vec<P2> = Vec::new();
        let mut first: Option<(Move, f64, f64)> = None;
        let start = at;
        at = layer.moves.iter().rev().find_map(|m| m.to).or(at);
        for (from, m) in chained(&layer.moves, start) {
            let line = bead_line(from, m);
            let same = match (&first, line) {
                (Some((f, per_mm, z)), Some((pm, mz))) => {
                    (f.speed, f.accel, f.fan, f.bead) == (m.speed, m.accel, m.fan, m.bead)
                        && *z == mz
                        && (pm - per_mm).abs() <= 1e-9 * per_mm
                }
                _ => false,
            };
            if !same {
                if let Some((f, per_mm, z)) = first.take() {
                    fit_run(&run, z, &f, per_mm, resolution, arcs, &mut out);
                }
                run.clear();
                if let (Some((per_mm, z)), Some(from)) = (line, from) {
                    run.push([from[0], from[1]]);
                    first = Some((m.clone(), per_mm, z));
                }
            }
            match (&first, m.to) {
                (Some(_), Some(to)) => run.push([to[0], to[1]]),
                _ => out.push(m.clone()),
            }
        }
        if let Some((f, per_mm, z)) = first.take() {
            fit_run(&run, z, &f, per_mm, resolution, arcs, &mut out);
        }
        layer.moves = out;
    }
}
