//! Triangles of about one size over a plane cut by lines, conforming across every line.
//!
//! The lines and the outline make one planar subdivision, computed once, so that neighbouring
//! faces share every corner exactly and no gap or overlap can open between them. Before that each
//! chain of edges between two junctions is walked in its fewest straight pieces, however finely
//! the solid it came from was cut there: a corner stays where it lies further off the chord of its
//! chain than a result's debris reaches, split at the farthest corner first, which reads the same
//! from either end and turns with the chain. Each piece is cut into equal pieces no longer than
//! the spacing, computed from its lower end. A face that is filled takes the points of an
//! equilateral lattice laid along its own axes (`Moments`) through its centroid, those at least
//! half a row of it inside, and is triangulated with them (Delaunay): equilateral triangles
//! inside, a band of others along its edges. Where Delaunay's rule cannot choose, four corners or
//! more on one circle, as where a row of the lattice runs along an edge cut at its pitch, their
//! polygon is fanned from its centroid. Everything lies on the nanometre lattice and depends only
//! on the lines: turned or mirrored lines are cut at the turned or mirrored points. (Relaxing the
//! inner points toward their neighbourhoods' centroids would even the band out, but the
//! relaxation at hand depends on the order it visits them in.)

use crate::ops::{from_lattice, region_to_lattice};
use crate::{GRID_MM, GeomError, MAX_POINTS, Moments, Point, RESOLUTION_MM, Region, check_value};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::IntOverlayOptions;
use i_overlay::i_float::int::point::IntPoint;
use i_overlay::string::overlay::StringOverlay;
use i_overlay::string::rule::StringRule;
use i_triangle::int::custom::IntCustomTriangulatable;
use i_triangle::int::validation::Validation;
use std::collections::{HashMap, HashSet};

type Lattice = IntPoint<i64>;

/// How close to a cell's edge, in spacings, no lattice point is kept: half a row of the lattice,
/// so that the band between an edge and the first row kept is a row wide, give or take half.
const KEEP_OFF: f64 = 0.433_012_701_892_219_3;
/// How much longer than it is the debris at both its ends can make a straight edge, in steps.
const ROUNDED_ENDS: f64 = 2.0 * ON_EDGE_STEPS;

/// Triangles over the faces of a subdivision: their corners, counterclockwise index triples, each
/// one's face, a point inside each face, and how many of the triangles lie in filled faces, which
/// come first.
#[derive(Debug, Clone, Default)]
pub struct Triangles {
    pub points: Vec<Point>,
    pub triangles: Vec<[u32; 3]>,
    pub cells: Vec<u32>,
    pub inside: Vec<Point>,
    pub filled: usize,
}

/// The lattice points of `0..=n` equal pieces of the edge from `a` to `b`, from its lower end.
fn cut(a: Lattice, b: Lattice, n: i64) -> Vec<Lattice> {
    let (lo, hi, reversed) = if (a.x, a.y) <= (b.x, b.y) { (a, b, false) } else { (b, a, true) };
    let mut out: Vec<Lattice> = (0..=n)
        .map(|k| {
            let at = |l: i64, h: i64| {
                let num = l as i128 * (n - k) as i128 + h as i128 * k as i128;
                let (q, r) = (num.div_euclid(n as i128), num.rem_euclid(n as i128));
                (q + if 2 * r >= n as i128 { 1 } else { 0 }) as i64
            };
            IntPoint::new(at(lo.x, hi.x), at(lo.y, hi.y))
        })
        .collect();
    if reversed {
        out.reverse();
    }
    out
}

/// How far off a chord a corner can lie and still lie on it, in steps. Boolean operations snap
/// corners together, moving them up to half a micrometre (0.47 µm measured on a turned gable), so
/// that what was straight is straight only to the thinnest thing a result keeps.
const ON_EDGE_STEPS: f64 = RESOLUTION_MM / GRID_MM;

/// How far `p` lies from the segment from `a` to `b`, in steps.
fn off_chord(a: Lattice, b: Lattice, p: Lattice) -> f64 {
    let (d, e) = ((b.x - a.x) as i128, (b.y - a.y) as i128);
    let (u, v) = ((p.x - a.x) as i128, (p.y - a.y) as i128);
    let (cross, along, length2) = (d * v - e * u, d * u + e * v, d * d + e * e);
    if along <= 0 || length2 == 0 {
        (u as f64).hypot(v as f64)
    } else if along >= length2 {
        ((p.x - b.x) as f64).hypot((p.y - b.y) as f64)
    } else {
        (cross as f64).abs() / (length2 as f64).sqrt()
    }
}

/// Whether `p` lies on the segment from `a` to `b`, strictly between its ends, but for debris.
fn between(a: Lattice, b: Lattice, p: Lattice) -> bool {
    let (d, e) = ((b.x - a.x) as i128, (b.y - a.y) as i128);
    let (u, v) = ((p.x - a.x) as i128, (p.y - a.y) as i128);
    let (cross, along, length2) = (d * v - e * u, d * u + e * v, d * d + e * e);
    along > 0 && along < length2 && (cross as f64).abs() <= ON_EDGE_STEPS * (length2 as f64).sqrt()
}

/// The lines' corners, filed by square buckets for the search along an edge.
struct Corners {
    size: i64,
    buckets: HashMap<(i64, i64), Vec<Lattice>>,
}

impl Corners {
    fn of(lines: &[Vec<Lattice>], size: i64) -> Corners {
        let mut corners = Corners { size, buckets: HashMap::new() };
        for &p in lines.iter().flatten() {
            let key = corners.key(p);
            corners.buckets.entry(key).or_default().push(p);
        }
        corners
    }

    fn key(&self, p: Lattice) -> (i64, i64) {
        (p.x.div_euclid(self.size), p.y.div_euclid(self.size))
    }

    /// The corners strictly inside the edge from `a` to `b`, in order from `a`.
    fn on(&self, a: Lattice, b: Lattice) -> Vec<Lattice> {
        let (k0, k1) = (
            self.key(IntPoint::new(a.x.min(b.x), a.y.min(b.y))),
            self.key(IntPoint::new(a.x.max(b.x), a.y.max(b.y))),
        );
        let mut found: Vec<Lattice> = Vec::new();
        // An edge whose box spans many buckets looks at every corner instead.
        if (k1.0 - k0.0 + 1).saturating_mul(k1.1 - k0.1 + 1) > 4096 {
            for list in self.buckets.values() {
                found.extend(list.iter().copied().filter(|&p| between(a, b, p)));
            }
        } else {
            for i in k0.0..=k1.0 {
                for j in k0.1..=k1.1 {
                    if let Some(list) = self.buckets.get(&(i, j)) {
                        found.extend(list.iter().copied().filter(|&p| between(a, b, p)));
                    }
                }
            }
        }
        let along = |p: &Lattice| {
            (p.x - a.x) as i128 * (b.x - a.x) as i128 + (p.y - a.y) as i128 * (b.y - a.y) as i128
        };
        found.sort_by_key(along);
        found.dedup();
        found
    }
}

/// The corners of a chain that lie off the chords between them, its ends always: each chord is
/// split at the corner farthest off it while that lies beyond the debris, and the chain reads the
/// same both ways.
fn fewest_chords(chain: &[Lattice]) -> Vec<Lattice> {
    let mut keep = vec![false; chain.len()];
    let Some(last) = chain.len().checked_sub(1) else { return Vec::new() };
    for end in [0, last] {
        if let Some(slot) = keep.get_mut(end) {
            *slot = true;
        }
    }
    let mut open = vec![(0, last)];
    while let Some((lo, hi)) = open.pop() {
        let (Some(&a), Some(&b)) = (chain.get(lo), chain.get(hi)) else { continue };
        let mut far = (ON_EDGE_STEPS, None);
        for k in lo + 1..hi {
            if let Some(&p) = chain.get(k) {
                let off = off_chord(a, b, p);
                if off > far.0 {
                    far = (off, Some(k));
                }
            }
        }
        if let (_, Some(k)) = far {
            if let Some(slot) = keep.get_mut(k) {
                *slot = true;
            }
            open.push((lo, k));
            open.push((k, hi));
        }
    }
    let mut corners: Vec<Lattice> = chain.iter().zip(&keep).filter(|(_, k)| **k).map(|(p, _)| *p).collect();
    // A chord parallel to an edge finds every corner of that edge equally far, and may have split
    // at one in its middle: the corners left on the chord of their neighbours go, nearest first.
    loop {
        let nearest = (1..corners.len().saturating_sub(1))
            .filter_map(|k| {
                let [a, m, b] = [corners.get(k - 1)?, corners.get(k)?, corners.get(k + 1)?];
                Some((off_chord(*a, *b, *m), k))
            })
            .min_by(|x, y| x.0.total_cmp(&y.0));
        match nearest {
            Some((off, k)) if off <= ON_EDGE_STEPS => {
                corners.remove(k);
            }
            _ => return corners,
        }
    }
}

/// A chain walked in its fewest chords, each cut into pieces no longer than `steps`, however the
/// ends were rounded.
fn walked(chain: &[Lattice], steps: f64) -> Vec<Lattice> {
    let corners = fewest_chords(chain);
    let mut out: Vec<Lattice> = Vec::with_capacity(corners.len());
    for w in corners.windows(2) {
        let (Some(&p), Some(&q)) = (w.first(), w.get(1)) else { continue };
        let length = ((q.x - p.x) as f64).hypot((q.y - p.y) as f64);
        let n = (((length - ROUNDED_ENDS) / steps).ceil() as i64).max(1);
        let mut piece = cut(p, q, n);
        piece.pop();
        out.extend(piece);
    }
    out.extend(corners.last().copied());
    out.dedup();
    out
}

/// A closed ring opened at a corner no chord can drop, the one farthest from its corners' mean:
/// nothing that lies along a straight edge is ever the farthest from any point. It ends where it
/// begins.
fn opened(ring: &[Lattice]) -> Vec<Lattice> {
    let n = ring.len() as f64;
    let (x, y) = ring.iter().fold((0.0, 0.0), |(x, y), p| (x + p.x as f64 / n, y + p.y as f64 / n));
    let distance = |p: &Lattice| (p.x as f64 - x).hypot(p.y as f64 - y);
    // Equally far corners are all kept whichever comes first: the lowest, to be the same each run.
    let start = ring
        .iter()
        .enumerate()
        .max_by(|(_, p), (_, q)| distance(p).total_cmp(&distance(q)).then((q.x, q.y).cmp(&(p.x, p.y))))
        .map_or(0, |(k, _)| k);
    let mut out: Vec<Lattice> = ring.iter().cycle().skip(start).take(ring.len()).copied().collect();
    out.extend(ring.get(start).copied());
    out
}

/// The lines as chains between junctions, corners where other than two edges meet, each edge in
/// one chain however many lines walk it, once each corner lying on an edge, but for rounding, has
/// cut it; a ring with no junction as a chain from a corner of its own.
fn chains(lines: &[Vec<Lattice>], steps: f64) -> Vec<Vec<Lattice>> {
    type Key = (i64, i64);
    let mut next: HashMap<Key, Vec<Lattice>> = HashMap::new();
    let mut seen: HashSet<(Key, Key)> = HashSet::new();
    let pair = |p: Key, q: Key| if p <= q { (p, q) } else { (q, p) };
    let corners = Corners::of(lines, (steps.ceil() as i64).max(1));
    for line in lines {
        for w in line.windows(2) {
            let (Some(&a), Some(&b)) = (w.first(), w.get(1)) else { continue };
            let mut stops = vec![a];
            stops.extend(corners.on(a, b));
            stops.push(b);
            for piece in stops.windows(2) {
                let (Some(&p), Some(&q)) = (piece.first(), piece.get(1)) else { continue };
                let (kp, kq) = ((p.x, p.y), (q.x, q.y));
                if kp != kq && seen.insert(pair(kp, kq)) {
                    next.entry(kp).or_default().push(q);
                    next.entry(kq).or_default().push(p);
                }
            }
        }
    }
    let mut order: Vec<Key> = next.keys().copied().collect();
    order.sort_unstable();
    let mut walked_edges: HashSet<(Key, Key)> = HashSet::with_capacity(seen.len());
    let mut out = Vec::new();
    // From each junction along each edge not yet walked; then what is left, rings of two-edge
    // corners, from wherever each is met.
    for rings in [false, true] {
        for &from in &order {
            let around = next.get(&from).map_or(&[][..], Vec::as_slice);
            if !rings && around.len() == 2 {
                continue;
            }
            for &first in around {
                if !walked_edges.insert(pair(from, (first.x, first.y))) {
                    continue;
                }
                let mut chain = vec![IntPoint::new(from.0, from.1), first];
                let mut at = first;
                loop {
                    let here = (at.x, at.y);
                    let around = next.get(&here).map_or(&[][..], Vec::as_slice);
                    if around.len() != 2 {
                        break;
                    }
                    let Some(&on) = around.iter().find(|p| !walked_edges.contains(&pair(here, (p.x, p.y))))
                    else {
                        break;
                    };
                    walked_edges.insert(pair(here, (on.x, on.y)));
                    chain.push(on);
                    at = on;
                }
                if rings {
                    chain.pop();
                    out.push(opened(&chain));
                } else {
                    out.push(chain);
                }
            }
        }
    }
    out
}

/// How far a corner can lie from where it should, along each axis, in steps: half a step for its
/// own rounding, and for an inner point as much again for the rounding of the face's corners, which
/// moves the centroid its lattice is laid through. Four corners on one circle come out within
/// three quarters of a step of it by this measure, others a thousand steps away, so that the
/// margin taken here decides nothing.
const ROUNDING_STEPS: f64 = 2.0;

/// Whether `d` may lie on the circle through `a`, `b` and `c`, but for rounding, when that circle
/// is at most `largest` steps in radius: whether the in-circle determinant is no larger than
/// rounding the four corners can make it, which follows from its gradient at each of them.
fn cocircular(a: Lattice, b: Lattice, c: Lattice, d: Lattice, largest: f64) -> bool {
    let len = |p: Lattice, q: Lattice| ((p.x - q.x) as f64).hypot((p.y - q.y) as f64);
    let orient = (b.x - a.x) as i128 * (c.y - a.y) as i128 - (b.y - a.y) as i128 * (c.x - a.x) as i128;
    if orient == 0 {
        return false;
    }
    let r = len(a, b) * len(b, c) * len(c, a) / (2.0 * (orient as f64).abs());
    if !(r <= largest && len(a, d) <= 2.0 * (r + ON_EDGE_STEPS)) {
        return false;
    }
    // The determinant of the rows `(x, y, x² + y²)` of `a`, `b` and `c`, taken from `d`: exact on
    // the lattice.
    let row = |p: Lattice| {
        let (x, y) = ((p.x - d.x) as i128, (p.y - d.y) as i128);
        (x, y, x * x + y * y)
    };
    let (ra, rb, rc) = (row(a), row(b), row(c));
    let ((ax, ay, a2), (bx, by, b2), (cx, cy, c2)) = (ra, rb, rc);
    let det = ax * (by * c2 - b2 * cy) - ay * (bx * c2 - b2 * cx) + a2 * (bx * cy - by * cx);
    // Its gradient at each of the three, and at `d`, which moves all three the other way.
    let mut against_d = [0.0f64; 2];
    let mut reach = 0.0f64;
    for ((x, y, _), (px, py, p2), (qx, qy, q2)) in [(ra, rb, rc), (rb, rc, ra), (rc, ra, rb)] {
        let (mx, my, m2) = (py * q2 - p2 * qy, p2 * qx - px * q2, px * qy - py * qx);
        let along = [mx as f64 + 2.0 * x as f64 * m2 as f64, my as f64 + 2.0 * y as f64 * m2 as f64];
        reach += along[0].abs() + along[1].abs();
        against_d = [against_d[0] + along[0], against_d[1] + along[1]];
    }
    (det as f64).abs() <= ROUNDING_STEPS * (reach + against_d[0].abs() + against_d[1].abs())
}

/// The triangles with each set of neighbours whose corners lie on one circle, between which
/// Delaunay's rule cannot choose and the triangulation chose by the order it met them, fanned
/// from their corners' centroid instead, a corner added: the same triangles however the cell
/// is turned or mirrored. Only circles of the cell's own size are looked for.
fn canonical(points: &mut Vec<Lattice>, triangles: &[[u32; 3]], steps: f64) -> Vec<[u32; 3]> {
    let at = |points: &[Lattice], i: u32| points.get(i as usize).copied();
    let mut parent: Vec<usize> = (0..triangles.len()).collect();
    let find = |parent: &mut Vec<usize>, mut i: usize| {
        while let Some(&p) = parent.get(i) {
            if p == i {
                break;
            }
            let up = parent.get(p).copied().unwrap_or(p);
            if let Some(slot) = parent.get_mut(i) {
                *slot = up;
            }
            i = up;
        }
        i
    };
    let mut holder: HashMap<(u32, u32), usize> = HashMap::with_capacity(3 * triangles.len());
    for (k, &[a, b, c]) in triangles.iter().enumerate() {
        for (p, q) in [(a, b), (b, c), (c, a)] {
            if let Some(&other) = holder.get(&(q, p))
                && let Some(&o) = triangles.get(other)
                && let Some(&far) = o.iter().find(|&&v| v != p && v != q)
                && let (Some(pa), Some(pb), Some(pc), Some(pd)) =
                    (at(points, a), at(points, b), at(points, c), at(points, far))
                && cocircular(pa, pb, pc, pd, steps)
            {
                let (x, y) = (find(&mut parent, k), find(&mut parent, other));
                if let Some(slot) = parent.get_mut(x) {
                    *slot = y;
                }
            }
            holder.insert((p, q), k);
        }
    }
    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for k in 0..triangles.len() {
        let root = find(&mut parent, k);
        groups.entry(root).or_default().push(k);
    }
    let mut roots: Vec<usize> = groups.keys().copied().collect();
    roots.sort_unstable();
    let mut out = Vec::with_capacity(triangles.len());
    for root in roots {
        let members: Vec<[u32; 3]> =
            groups.get(&root).into_iter().flatten().filter_map(|&k| triangles.get(k).copied()).collect();
        if let Some(fan) = fanned(points, &members) {
            out.extend(fan);
        } else {
            out.extend(members);
        }
    }
    out
}

/// The polygon a set of neighbouring triangles covers, fanned from its corners' centroid, added
/// to the points; `None` for one triangle, or when they do not cover one polygon.
fn fanned(points: &mut Vec<Lattice>, members: &[[u32; 3]]) -> Option<Vec<[u32; 3]>> {
    if members.len() < 2 {
        return None;
    }
    let edges: HashSet<(u32, u32)> = members.iter().flat_map(|&[a, b, c]| [(a, b), (b, c), (c, a)]).collect();
    let next: HashMap<u32, u32> =
        edges.iter().filter(|(p, q)| !edges.contains(&(*q, *p))).map(|&(p, q)| (p, q)).collect();
    let &start = next.keys().min()?;
    let mut ring = vec![start];
    while let Some(&v) = next.get(ring.last()?) {
        if v == start {
            break;
        }
        if ring.len() > next.len() {
            return None;
        }
        ring.push(v);
    }
    if ring.len() != next.len() {
        return None;
    }
    let corners: Vec<Lattice> = ring.iter().filter_map(|&i| points.get(i as usize).copied()).collect();
    let n = corners.len() as f64;
    let (x, y) = corners.iter().fold((0.0, 0.0), |(x, y), p| (x + p.x as f64 / n, y + p.y as f64 / n));
    let c = IntPoint::new(x.round() as i64, y.round() as i64);
    // Every piece counterclockwise, as a polygon too small for the lattice might not leave them.
    let turns = |p: Lattice, q: Lattice| {
        (p.x - c.x) as i128 * (q.y - c.y) as i128 - (p.y - c.y) as i128 * (q.x - c.x) as i128
    };
    if corners.iter().zip(corners.iter().cycle().skip(1)).any(|(&p, &q)| turns(p, q) <= 0) {
        return None;
    }
    let centre = u32::try_from(points.len()).ok()?;
    points.push(c);
    Some(ring.iter().zip(ring.iter().cycle().skip(1)).map(|(&p, &q)| [centre, p, q]).collect())
}

/// How far past the lines the frame of their subdivision lies, in steps: any number will do.
const FRAME_STEPS: i64 = 1000;

/// The faces `lines` cut the plate into, all from one pass: neighbours share every corner, and
/// every corner of a line is a corner of the faces along it. The face around them all is left in.
fn faces(lines: &[Vec<Lattice>], steps: f64) -> Vec<Vec<Vec<Lattice>>> {
    let paths: Vec<Vec<Lattice>> = chains(lines, steps).iter().map(|c| walked(c, steps)).collect();
    let (mut lo, mut hi) = ((i64::MAX, i64::MAX), (i64::MIN, i64::MIN));
    for p in paths.iter().flatten() {
        lo = (lo.0.min(p.x), lo.1.min(p.y));
        hi = (hi.0.max(p.x), hi.1.max(p.y));
    }
    let (x0, y0, x1, y1) = (lo.0 - FRAME_STEPS, lo.1 - FRAME_STEPS, hi.0 + FRAME_STEPS, hi.1 + FRAME_STEPS);
    let frame = vec![vec![vec![
        IntPoint::new(x0, y0),
        IntPoint::new(x1, y0),
        IntPoint::new(x1, y1),
        IntPoint::new(x0, y1),
    ]]];
    let mut overlay = StringOverlay::from_shape_custom(&frame, IntOverlayOptions::keep_all_points());
    overlay.add_string_source(&paths);
    overlay
        .build_graph_view(FillRule::NonZero)
        .map(|graph| graph.extract_shapes_custom(StringRule::Slice, IntOverlayOptions::keep_all_points()))
        .unwrap_or_default()
}

/// A point inside a face triangulated as `indices` over `corners`: the centre of the largest of
/// its triangles. `None` when it holds no area.
fn inside(corners: &[Lattice], indices: &[u32]) -> Option<Point> {
    let at = |i: u32| corners.get(i as usize).copied();
    let mut best: Option<(i128, Point)> = None;
    for t in indices.as_chunks::<3>().0 {
        let (Some(a), Some(b), Some(c)) = (at(t[0]), at(t[1]), at(t[2])) else { continue };
        let twice =
            ((b.x - a.x) as i128 * (c.y - a.y) as i128 - (b.y - a.y) as i128 * (c.x - a.x) as i128).abs();
        if best.is_none_or(|(most, _)| twice > most) {
            let [pa, pb, pc] = [a, b, c].map(|p| from_lattice(&p));
            best = Some((twice, [(pa[0] + pb[0] + pc[0]) / 3.0, (pa[1] + pb[1] + pc[1]) / 3.0]));
        }
    }
    best.filter(|(twice, _)| *twice > 0).map(|(_, p)| p)
}

fn validation() -> Validation<i64> {
    Validation { fill_rule: FillRule::NonZero, options: IntOverlayOptions::keep_all_points() }
}

/// A face of the subdivision: its contours on the lattice and as a region, a point inside it,
/// whether it takes inner points, and its area.
struct Face {
    contours: Vec<Vec<Vec<Lattice>>>,
    region: Region,
    point: Point,
    full: bool,
    area: f64,
}

/// Triangles about `spacing` mm wide over `outline`, conforming across `lines`, each an open or a
/// closed polyline (the last corner the first), of which they hold every corner a chord keeps.
/// A face takes inner points where `filled` says so of a point inside it, or only those of its
/// edges.
pub fn triangulate_evenly(
    outline: &Region,
    lines: &[Vec<Point>],
    filled: &dyn Fn(Point) -> bool,
    spacing: f64,
) -> Result<Triangles, GeomError> {
    if check_value(spacing)? <= GRID_MM * 1e3 {
        return Err(GeomError::OutOfRange(spacing));
    }
    crate::check_points(outline.iter().flatten())?;
    crate::check_points(lines)?;
    let steps = spacing / GRID_MM;
    let closed = |ring: &Vec<Point>| -> Vec<Point> { ring.iter().chain(ring.first()).copied().collect() };
    let mut all: Vec<Vec<Point>> = outline.iter().flatten().map(closed).collect();
    let rim: Vec<(Point, Point)> = all
        .iter()
        .flat_map(|ring| ring.windows(2).filter_map(|w| Some((*w.first()?, *w.get(1)?))).collect::<Vec<_>>())
        .collect();
    all.extend(lines.iter().cloned());
    let on_lattice: Vec<Vec<Lattice>> = region_to_lattice(&vec![all]).into_iter().flatten().collect();
    let mut cells: Vec<Face> = Vec::new();
    for shape in faces(&on_lattice, steps) {
        let contours = vec![shape];
        let plain = contours
            .custom_triangulate_with_steiner_points(&Vec::new(), validation())
            .into_delaunay()
            .into_raw();
        let Some(point) = inside(plain.points(), &plain.triangle_indices::<u32>()) else { continue };
        if !crate::encloses(rim.iter().copied(), point) {
            continue;
        }
        let region: Region = contours
            .iter()
            .map(|s| s.iter().map(|ring| ring.iter().map(from_lattice).collect()).collect())
            .collect();
        let area = crate::area(&region);
        cells.push(Face { contours, region, point, full: filled(point), area });
    }
    // Filled faces first, the largest first in each: the order does not depend on how they lie.
    cells.sort_by(|a, b| b.full.cmp(&a.full).then(b.area.total_cmp(&a.area)));
    let mut out = Triangles::default();
    // Each boundary point's index, shared by every face it bounds.
    let mut shared: HashMap<(i64, i64), u32> = HashMap::new();
    for (index, Face { contours, region, point, full, .. }) in cells.iter().enumerate() {
        out.inside.push(*point);
        let inner = if *full { fill(region, contours, spacing)? } else { Vec::new() };
        let raw =
            contours.custom_triangulate_with_steiner_points(&inner, validation()).into_delaunay().into_raw();
        let mut local: Vec<Lattice> = raw.points().clone();
        let indices: Vec<u32> = raw.triangle_indices::<u32>();
        let triangles = canonical(&mut local, indices.as_chunks::<3>().0, steps);
        let boundary: HashSet<(i64, i64)> = contours.iter().flatten().flatten().map(|p| (p.x, p.y)).collect();
        let mut ids = Vec::with_capacity(local.len());
        for p in &local {
            let key = (p.x, p.y);
            let id = if boundary.contains(&key) {
                *shared.entry(key).or_insert_with(|| {
                    out.points.push(from_lattice(p));
                    (out.points.len() - 1) as u32
                })
            } else {
                out.points.push(from_lattice(p));
                (out.points.len() - 1) as u32
            };
            ids.push(id);
            if out.points.len() > MAX_POINTS {
                return Err(GeomError::Engine(format!("more than {MAX_POINTS} points to triangulate")));
            }
        }
        for t in &triangles {
            let corner = |i: u32| ids.get(i as usize).copied();
            if let (Some(a), Some(b), Some(c)) = (corner(t[0]), corner(t[1]), corner(t[2])) {
                out.triangles.push([a, b, c]);
                out.cells.push(index as u32);
                if *full {
                    out.filled += 1;
                }
            }
        }
    }
    Ok(out)
}

/// The points of an equilateral lattice `spacing` apart along the cell's own axes through its
/// centroid that lie inside it, at least `KEEP_OFF` spacings from its edges.
fn fill(region: &Region, contours: &[Vec<Vec<Lattice>>], spacing: f64) -> Result<Vec<Lattice>, GeomError> {
    let Some(moments) = Moments::of(region.iter().map(|s| (s.as_slice(), 1.0))) else {
        return Ok(Vec::new());
    };
    let first = moments.axes.map_or([1.0, 0.0], |a| a.first);
    let (u, v) = (
        [spacing * first[0], spacing * first[1]],
        [
            spacing * (0.5 * first[0] - 0.75f64.sqrt() * first[1]),
            spacing * (0.75f64.sqrt() * first[0] + 0.5 * first[1]),
        ],
    );
    let edges: Vec<([f64; 2], [f64; 2])> = contours
        .iter()
        .flatten()
        .flat_map(|ring| {
            ring.iter().zip(ring.iter().cycle().skip(1)).map(|(a, b)| (from_lattice(a), from_lattice(b)))
        })
        .collect();
    let Some([lo, hi]) = crate::bounds(region) else { return Ok(Vec::new()) };
    // Edges filed by buckets a spacing wide, for the distance test.
    let key =
        |p: [f64; 2]| (((p[0] - lo[0]) / spacing).floor() as i64, ((p[1] - lo[1]) / spacing).floor() as i64);
    let mut near: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (k, (a, b)) in edges.iter().enumerate() {
        let (k0, k1) = (key([a[0].min(b[0]), a[1].min(b[1])]), key([a[0].max(b[0]), a[1].max(b[1])]));
        for i in k0.0..=k1.0 {
            for j in k0.1..=k1.1 {
                near.entry((i, j)).or_default().push(k);
            }
        }
    }
    let c = moments.centroid;
    let reach = (hi[0] - lo[0]).hypot(hi[1] - lo[1]);
    let n = (reach / (spacing * 0.75f64.sqrt())).ceil() as i64 + 2;
    if (2 * n + 1).saturating_mul(2 * n + 1) as usize > 4 * MAX_POINTS {
        return Err(GeomError::Engine(format!("a cell {reach} mm across is too large to triangulate")));
    }
    let keep = KEEP_OFF * spacing;
    let mut out = Vec::new();
    for j in -n..=n {
        for i in -n..=n {
            let p = [c[0] + i as f64 * u[0] + j as f64 * v[0], c[1] + i as f64 * u[1] + j as f64 * v[1]];
            if p[0] < lo[0] + keep || p[0] > hi[0] - keep || p[1] < lo[1] + keep || p[1] > hi[1] - keep {
                continue;
            }
            if !crate::encloses(edges.iter().copied(), p) {
                continue;
            }
            let (ki, kj) = key(p);
            let close = (ki - 1..=ki + 1).any(|a| {
                (kj - 1..=kj + 1).any(|b| {
                    near.get(&(a, b)).is_some_and(|list| {
                        list.iter().any(|&e| {
                            edges.get(e).is_some_and(|&(x, y)| crate::segment_distance(p, x, y) < keep)
                        })
                    })
                })
            });
            if !close {
                out.push(IntPoint::new((p[0] / GRID_MM).round() as i64, (p[1] / GRID_MM).round() as i64));
            }
        }
    }
    Ok(out)
}
