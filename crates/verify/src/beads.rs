//! The beads laid so far, against the nozzle that passes over them: within its column, none may
//! stand above its underside, those of the layer it prints by more than the ironing allowed,
//! those of the layers before by more than the rounding of written positions; beyond, on a bent
//! layer, by what the tip and the bead may each stand off their layer's top, written within the
//! resolution in space: `resolution · √(1 + G²)` in height, the layer leaning `G` at most.
//!
//! A bead is a capsule, the segment the nozzle swept widened by half its width, flat across and
//! as high as the nozzle where it passed, rising evenly along it; an arc's, along chords
//! widened by their sagitta, raised by what its climb, even in the angle, departs from even
//! along the chord, under `|Δz| · (1 − cos(Δφ/2))`. The nozzle's underside lies
//! `floor(r)` above its tip, nothing on its face, then its flank, up to its column. Over each
//! cross-section of a bead the underside comes lowest at the point nearest the axis, at
//! `d(s) = √(a(s)² + c²)`, `a` the offset along the bead and `c` across it past its edge, convex
//! in the place `s` along the bead; the bead's height less the floor there is concave on the
//! flank and linear on the face, or the floor its cap past both, so its greatest lies at the
//! bead's ends, where the column's or the face's rim crosses it, or where its derivative
//! `g + k·a/d` vanishes, `a = m·c/√(1 − m²)` for `m = −g/k`; past the ends, the caps stand at the
//! ends' heights. Every answer is exact. Beads are filed by place in pieces a cell long, and
//! forgotten once lower than every tip to come.

use crate::Toolhead;
use layercraft_motion::Move;
use std::collections::HashMap;

/// The side of the cells beads are filed in, and the longest piece filed, in mm.
const CELL: f64 = 1.0;
/// The longest chord an arc's bead is laid along, in mm.
const CHORD: f64 = 0.1;
/// The most pieces one move's bead is filed in: a plate 400 mm across, at a chord each.
const MAX_PIECES: usize = 1 << 13;

/// A piece of a bead: from `a` to `b`, the nozzle at `za` and `zb`, `half` its half-width.
#[derive(Debug, Clone, Copy)]
struct Piece {
    a: [f64; 2],
    b: [f64; 2],
    za: f64,
    zb: f64,
    half: f64,
    layer: usize,
    /// How far above its layer's top it may stand, written off it.
    lift: f64,
    /// The cells it is filed in: `[i0, j0]` to `[i1, j1]`.
    cells: [[i64; 2]; 2],
}

/// What a nozzle may come into, and how far.
pub struct Allowed<'a> {
    pub head: &'a Toolhead,
    /// The layer it prints, and how far below its top the tip may pass, written off it.
    pub layer: usize,
    pub lift: f64,
    /// How deep the face may iron the beads of that layer.
    pub ironing: f64,
    /// How far into those before: the rounding of written positions.
    pub written: f64,
}

/// The beads laid so far, filed by place.
#[derive(Default)]
pub struct Beads {
    pieces: Vec<Piece>,
    cells: HashMap<[i64; 2], Vec<usize>>,
    /// The widest half of a bead filed.
    widest: f64,
    /// The highest any bead's layer stands under it, forgotten or not: as high as the bead, less
    /// how far it may stand above it.
    pub highest: f64,
}

fn cell_of(v: f64) -> i64 {
    (v / CELL).floor() as i64
}

impl Beads {
    pub fn new() -> Beads {
        Beads { highest: f64::NEG_INFINITY, ..Beads::default() }
    }

    fn file(&mut self, mut piece: Piece) {
        let h = piece.half;
        let lo = [cell_of(piece.a[0].min(piece.b[0]) - h), cell_of(piece.a[1].min(piece.b[1]) - h)];
        let hi = [cell_of(piece.a[0].max(piece.b[0]) + h), cell_of(piece.a[1].max(piece.b[1]) + h)];
        piece.cells = [lo, hi];
        let id = self.pieces.len();
        for j in lo[1]..=hi[1] {
            for i in lo[0]..=hi[0] {
                self.cells.entry([i, j]).or_default().push(id);
            }
        }
        self.widest = self.widest.max(h);
        self.highest = self.highest.max(piece.za.max(piece.zb) - piece.lift);
        self.pieces.push(piece);
    }

    /// Lays the bead a move from `from` lays on `layer`, if any, standing up to `lift` above the
    /// layer's top.
    pub fn lay(&mut self, m: &Move, from: [f64; 3], layer: usize, lift: f64) {
        let (Some(bead), Some(_)) = (m.bead, m.to) else { return };
        if m.e <= 0.0 || !(bead.width.is_finite() && bead.width > 0.0) {
            return;
        }
        let length = m.length(from);
        let half = bead.width / 2.0;
        let piece = |a: [f64; 3], b: [f64; 3], half: f64| Piece {
            a: [a[0], a[1]],
            b: [b[0], b[1]],
            za: a[2],
            zb: b[2],
            half,
            layer,
            lift,
            cells: [[0, 0]; 2],
        };
        match m.arc {
            None => {
                let n = ((length / CELL).ceil() as usize).clamp(1, MAX_PIECES);
                for k in 0..n {
                    let (p, q) =
                        (m.point(from, k as f64 / n as f64), m.point(from, (k + 1) as f64 / n as f64));
                    self.file(piece(p, q, half));
                }
            }
            Some(arc) => {
                let r = (from[0] - arc.centre[0]).hypot(from[1] - arc.centre[1]);
                let n = ((length / CHORD).ceil() as usize).clamp(1, MAX_PIECES);
                for k in 0..n {
                    let (p, q) =
                        (m.point(from, k as f64 / n as f64), m.point(from, (k + 1) as f64 / n as f64));
                    let chord = (q[0] - p[0]).hypot(q[1] - p[1]);
                    let sagitta = r - (r * r - (chord / 2.0).powi(2)).max(0.0).sqrt();
                    let swept = (arc.turn / n as f64).abs().min(std::f64::consts::PI);
                    let raise = (q[2] - p[2]).abs() * (1.0 - (swept / 2.0).cos());
                    self.file(piece([p[0], p[1], p[2] + raise], [q[0], q[1], q[2] + raise], half + sagitta));
                }
            }
        }
    }

    /// Forgets every bead no higher than `z`: no tip to come is lower.
    pub fn forget_below(&mut self, z: f64) {
        let kept: Vec<Piece> = self.pieces.iter().filter(|p| p.za.max(p.zb) > z).copied().collect();
        if kept.len() == self.pieces.len() {
            return;
        }
        let highest = self.highest;
        *self = Beads::new();
        for p in kept {
            self.file(p);
        }
        self.highest = highest;
    }

    /// How far, at worst, the nozzle with its tip at `tip`, anywhere within `blur` of it across
    /// the plate and no lower, comes into a bead beyond what is allowed; and whether that bead
    /// is of its layer. Positive only where it does.
    pub fn contact(&self, allowed: &Allowed, tip: [f64; 3], blur: f64) -> (f64, bool) {
        let head = allowed.head;
        let reach = head.column_radius + blur + self.widest;
        let lo = [cell_of(tip[0] - reach), cell_of(tip[1] - reach)];
        let hi = [cell_of(tip[0] + reach), cell_of(tip[1] + reach)];
        let mut worst = (f64::NEG_INFINITY, false);
        for j in lo[1]..=hi[1] {
            for i in lo[0]..=hi[0] {
                for &id in self.cells.get(&[i, j]).into_iter().flatten() {
                    let Some(p) = self.pieces.get(id) else { continue };
                    // Each piece once: in the first of its cells the query meets.
                    if [i, j] != [p.cells[0][0].max(lo[0]), p.cells[0][1].max(lo[1])] {
                        continue;
                    }
                    let own = p.layer == allowed.layer;
                    let room = if own { allowed.ironing } else { allowed.written };
                    let allowed = tip[2] + room + allowed.lift + p.lift;
                    // The floor is nowhere below the tip: a piece no higher than allowed, or than
                    // the worst found, changes nothing that matters.
                    if p.za.max(p.zb) - allowed <= worst.0.max(0.0) {
                        continue;
                    }
                    if let Some(over) = over(p, head, [tip[0], tip[1]], blur) {
                        let excess = over - allowed;
                        if excess > worst.0 {
                            worst = (excess, own);
                        }
                    }
                }
            }
        }
        worst
    }
}

/// The most the piece stands above the nozzle's underside about `x`, the underside's rims
/// widened by `blur`; `None` when no part of it lies within the column.
fn over(p: &Piece, head: &Toolhead, x: [f64; 2], blur: f64) -> Option<f64> {
    let (column, face, k) = (head.column_radius + blur, head.face_radius + blur, head.flank_slope);
    let floor = |d: f64| head.floor((d - blur).max(0.0));
    let mut best: Option<f64> = None;
    let mut consider = |height: f64, d: f64| {
        if d <= column {
            let v = height - floor(d);
            best = Some(best.map_or(v, |b: f64| b.max(v)));
        }
    };
    let d = [p.b[0] - p.a[0], p.b[1] - p.a[1]];
    let length = d[0].hypot(d[1]);
    if length <= 0.0 {
        consider(p.za.max(p.zb), ((x[0] - p.a[0]).hypot(x[1] - p.a[1]) - p.half).max(0.0));
        return best;
    }
    let (u, n) = ([d[0] / length, d[1] / length], [-d[1] / length, d[0] / length]);
    let rel = [x[0] - p.a[0], x[1] - p.a[1]];
    let (a0, across) = (rel[0] * u[0] + rel[1] * u[1], rel[0] * n[0] + rel[1] * n[1]);
    let c = (across.abs() - p.half).max(0.0);
    let g = (p.zb - p.za) / length;
    // Where the cross-sections lie within the column.
    if c <= column {
        let q = (column * column - c * c).sqrt();
        let (lo, hi) = ((a0 - q).max(0.0), (a0 + q).min(length));
        if lo <= hi {
            let mut places = vec![lo, hi];
            if c <= face {
                let q = (face * face - c * c).sqrt();
                places.extend([a0 - q, a0 + q]);
            }
            if c > 0.0 && k > 0.0 {
                let m = -g / k;
                if m.abs() < 1.0 {
                    places.push(a0 - m * c / (1.0 - m * m).sqrt());
                }
            }
            for s in places {
                let s = s.clamp(lo, hi);
                consider(p.za + g * s, (a0 - s).hypot(c));
            }
        }
    }
    // The caps, past each end, outward.
    if a0 < 0.0 {
        consider(p.za, (rel[0].hypot(rel[1]) - p.half).max(0.0));
    }
    if a0 > length {
        consider(p.zb, ((x[0] - p.b[0]).hypot(x[1] - p.b[1]) - p.half).max(0.0));
    }
    best
}
