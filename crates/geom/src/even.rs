//! Triangles of about one size over cells that share edges, with the same corners along every
//! shared edge.
//!
//! The cells are regions meeting only along their edges, each contour walking a straight edge in
//! one piece however finely it was cut. Each corner of a cell that lies on an edge of another is
//! added to that edge, so that both walk their common edge through the same corners; every edge
//! is then cut into equal pieces no longer than the spacing, the cut points computed from the
//! edge's lower end, so both sides get them bit for bit. Inside, a cell takes the points of an
//! equilateral lattice laid along its own axes (`Moments`) through its centroid, those at least
//! half a row of it inside, and is triangulated with them (Delaunay): equilateral triangles
//! inside, a band of others along its edges. Where Delaunay's rule cannot choose, four corners
//! or more on one circle, as where a row of the lattice runs along an edge cut at its pitch,
//! their polygon is fanned from its centroid. Everything lies on the nanometre lattice and
//! depends only on the cells: turned or mirrored cells are cut at the turned or mirrored points.
//! (Relaxing the inner points toward their neighbourhoods' centroids would even the band out,
//! but the relaxation at hand depends on the order it visits them in.)

use crate::ops::{from_lattice, region_to_lattice};
use crate::{GRID_MM, GeomError, MAX_POINTS, Moments, Point, Region, check_value};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::IntOverlayOptions;
use i_overlay::i_float::int::point::IntPoint;
use i_triangle::int::custom::IntCustomTriangulatable;
use i_triangle::int::validation::Validation;
use std::collections::HashMap;

type Lattice = IntPoint<i64>;

/// How close to a cell's edge, in spacings, no lattice point is kept: half a row of the lattice,
/// so that the band between an edge and the first row kept is a row wide, give or take half.
const KEEP_OFF: f64 = 0.433_012_701_892_219_3;
/// How much longer than it is rounding both its ends to the lattice can make an edge, in steps.
const ROUNDED_ENDS: f64 = std::f64::consts::SQRT_2;

/// Triangles over cells: their corners, counterclockwise index triples, and each one's cell.
#[derive(Debug, Clone, Default)]
pub struct Triangles {
    pub points: Vec<Point>,
    pub triangles: Vec<[u32; 3]>,
    pub cells: Vec<u32>,
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

/// How far off an edge rounding to the lattice can leave a corner that lies on it, in steps:
/// half a step's diagonal for the corner, and as much for the edge where it passes.
const ON_EDGE_STEPS: f64 = std::f64::consts::SQRT_2;

/// Whether `p` lies on the segment from `a` to `b`, strictly between its ends, but for the
/// lattice's rounding.
fn between(a: Lattice, b: Lattice, p: Lattice) -> bool {
    let (d, e) = ((b.x - a.x) as i128, (b.y - a.y) as i128);
    let (u, v) = ((p.x - a.x) as i128, (p.y - a.y) as i128);
    let (cross, along, length2) = (d * v - e * u, d * u + e * v, d * d + e * e);
    along > 0 && along < length2 && (cross as f64).abs() <= ON_EDGE_STEPS * (length2 as f64).sqrt()
}

/// The contour walking each of its straight edges in one piece: a corner lying between its
/// neighbours, but for the lattice's rounding, is none, however finely the solid it came from
/// was cut there.
fn straight(contour: &[Lattice]) -> Vec<Lattice> {
    let mut out: Vec<Lattice> = Vec::with_capacity(contour.len());
    for &p in contour {
        while let [.., a, m] = out.as_slice() {
            if !between(*a, p, *m) {
                break;
            }
            out.pop();
        }
        out.push(p);
    }
    // Round the seam: the last corners against the first.
    while out.len() > 3 {
        if let [.., a, m] = out.as_slice()
            && let Some(&b) = out.first()
            && between(*a, b, *m)
        {
            out.pop();
            continue;
        }
        if let (Some(&a), [m, b, ..]) = (out.last(), out.as_slice())
            && between(a, *b, *m)
        {
            out.remove(0);
            continue;
        }
        break;
    }
    out
}

/// How far off the circle through three corners rounding to the lattice can leave a fourth that
/// lies on it, in steps: the half-diagonal each of the four may move, the circle following its
/// three.
const COCIRCULAR_STEPS: f64 = 2.0 * std::f64::consts::SQRT_2;

/// Whether `d` lies on the circle through `a`, `b` and `c`, but for rounding, when that circle
/// is at most `largest` steps in radius.
fn cocircular(a: Lattice, b: Lattice, c: Lattice, d: Lattice, largest: f64) -> bool {
    let len = |p: Lattice, q: Lattice| ((p.x - q.x) as f64).hypot((p.y - q.y) as f64);
    let orient = (b.x - a.x) as i128 * (c.y - a.y) as i128 - (b.y - a.y) as i128 * (c.x - a.x) as i128;
    if orient == 0 {
        return false;
    }
    let r = len(a, b) * len(b, c) * len(c, a) / (2.0 * (orient as f64).abs());
    if !(r <= largest && len(a, d) <= 2.0 * r + COCIRCULAR_STEPS) {
        return false;
    }
    // The in-circle determinant, exact on the lattice: `orient · (R² − |d − O|²)`.
    let row = |p: Lattice| {
        let (x, y) = ((p.x - d.x) as i128, (p.y - d.y) as i128);
        (x, y, x * x + y * y)
    };
    let ((ax, ay, a2), (bx, by, b2), (cx, cy, c2)) = (row(a), row(b), row(c));
    let det = ax * (by * c2 - b2 * cy) - ay * (bx * c2 - b2 * cx) + a2 * (bx * cy - by * cx);
    // How far `d` lies off the circle: `(R² − |d − O|²) / 2R`.
    (det as f64 / orient as f64 / (2.0 * r)).abs() <= COCIRCULAR_STEPS
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
    let edges: std::collections::HashSet<(u32, u32)> =
        members.iter().flat_map(|&[a, b, c]| [(a, b), (b, c), (c, a)]).collect();
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

/// The corners of every cell, filed by square buckets for the corners-on-edges search.
struct Corners {
    size: i64,
    buckets: HashMap<(i64, i64), Vec<Lattice>>,
}

impl Corners {
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

/// Triangles about `spacing` mm wide over the cells, conforming along their shared edges: each
/// cell a region and whether it takes inner points, or only those of its edges.
pub fn triangulate_evenly(cells: &[(Region, bool)], spacing: f64) -> Result<Triangles, GeomError> {
    if check_value(spacing)? <= GRID_MM * 1e3 {
        return Err(GeomError::OutOfRange(spacing));
    }
    for (cell, _) in cells {
        crate::check_points(cell.iter().flatten())?;
    }
    let steps = spacing / GRID_MM;
    let lattice: Vec<Vec<Vec<Vec<Lattice>>>> = cells
        .iter()
        .map(|(r, _)| {
            region_to_lattice(r)
                .into_iter()
                .map(|shape| shape.into_iter().map(|c| straight(&c)).collect())
                .collect()
        })
        .collect();
    let mut corners = Corners { size: (steps.ceil() as i64).max(1), buckets: HashMap::new() };
    for p in lattice.iter().flatten().flatten().flatten() {
        let key = corners.key(*p);
        corners.buckets.entry(key).or_default().push(*p);
    }
    let mut out = Triangles::default();
    // Each boundary point's index, shared by every cell it bounds.
    let mut shared: HashMap<(i64, i64), u32> = HashMap::new();
    for (index, (cell, (region, filled))) in lattice.iter().zip(cells).enumerate() {
        // The cell's contours through every corner on them, each piece cut to the spacing.
        let mut contours: Vec<Vec<Vec<Lattice>>> = Vec::with_capacity(cell.len());
        for shape in cell {
            let mut rings = Vec::with_capacity(shape.len());
            for contour in shape {
                let mut ring: Vec<Lattice> = Vec::new();
                for (k, &a) in contour.iter().enumerate() {
                    let Some(&b) = contour.get((k + 1) % contour.len()) else { continue };
                    let mut stops = vec![a];
                    stops.extend(corners.on(a, b));
                    stops.push(b);
                    for w in stops.windows(2) {
                        let (Some(&p), Some(&q)) = (w.first(), w.get(1)) else { continue };
                        let length = ((q.x - p.x) as f64).hypot((q.y - p.y) as f64);
                        // Pieces no longer than the spacing, however the ends were rounded.
                        let n = (((length - ROUNDED_ENDS) / steps).ceil() as i64).max(1);
                        let mut piece = cut(p, q, n);
                        piece.pop();
                        ring.extend(piece);
                    }
                }
                ring.dedup();
                if ring.len() >= 3 {
                    rings.push(ring);
                }
            }
            if !rings.is_empty() {
                contours.push(rings);
            }
        }
        if contours.is_empty() {
            continue;
        }
        let inner = if *filled { fill(region, &contours, spacing)? } else { Vec::new() };
        let validation =
            Validation { fill_rule: FillRule::NonZero, options: IntOverlayOptions::keep_all_points() };
        let raw =
            contours.custom_triangulate_with_steiner_points(&inner, validation).into_delaunay().into_raw();
        let mut local: Vec<Lattice> = raw.points().clone();
        let indices: Vec<u32> = raw.triangle_indices::<u32>();
        let triangles = canonical(&mut local, indices.as_chunks::<3>().0, steps);
        let boundary: std::collections::HashSet<(i64, i64)> =
            contours.iter().flatten().flatten().map(|p| (p.x, p.y)).collect();
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
