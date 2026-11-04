//! Reliefs: piecewise linear heights over the plate, on triangles that any number of them share.
//!
//! A carrier holds triangles and their corners; a relief on it, its value at each corner. The
//! relief is linear on each triangle, zero past them and zero along their outer edges, so it is
//! continuous. A triangle where it is zero at every corner takes no part: every query passes it
//! by and the relief is stored without it, so a relief behaves as the triangles where it is not
//! zero alone, whatever else its carrier holds. Along a segment it is linear between the points
//! where the segment crosses those triangles' edges: cut there (`Relief::breaks`), the segment
//! carries it exactly, as straight pieces. Its extremes over a box lie at the box's corners,
//! where the box's sides cross those edges, or at corners inside it.
//!
//! Stage files from before reliefs had triangles of their own hold a block of values on a
//! regular grid, each square split by its diagonal from `(i, j)` to `(i + 1, j + 1)` and the
//! values zero a square past the block: read, they become those very triangles.

use crate::locate::Buckets;
use crate::orient::orientation;
use crate::{GRID_MM, GeomError, LIMIT_MM, Point, check_value};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

/// The most corners a relief holds: a plate 400 mm square, a corner every 0.1 mm.
pub const MAX_POINTS: usize = 1 << 24;
/// How far outside a triangle, as a fraction of it, a point still counts as on it: rounding.
const ON_TRIANGLE: f64 = 1e-9;
/// Breaks closer than this along a segment, in mm, are one: a tenth of the lattice step.
const SAME_BREAK_MM: f64 = GRID_MM / 10.0;

/// Triangles over the plate with their corners, shared by every relief on them.
#[derive(Debug, Clone, Default)]
pub struct Carrier(Arc<Tiling>);

/// A carrier's triangles, as index triples into its corners, with the buckets that find them.
#[derive(Debug, Default)]
struct Tiling {
    points: Vec<Point>,
    triangles: Vec<[u32; 3]>,
    buckets: Buckets,
    /// The corners on no triangle's inside: along an outer edge, or on no triangle at all.
    rim: Vec<u32>,
}

/// A piecewise linear height over the plate.
#[derive(Debug, Clone, Default)]
pub struct Relief {
    carrier: Carrier,
    /// The value at each of the carrier's corners.
    values: Arc<[f64]>,
    /// The lowest and highest values, zero included.
    range: [f64; 2],
}

impl PartialEq for Relief {
    /// Equal when they hold the same triangles, corners and values where they are not zero.
    fn eq(&self, other: &Relief) -> bool {
        if Arc::ptr_eq(&self.carrier.0, &other.carrier.0) {
            return self.values == other.values;
        }
        self.trimmed() == other.trimmed()
    }
}

/// A triangle of a relief holding a point: its corners, the relief's values there and the
/// point's barycentric coordinates in it.
struct Located {
    corners: [Point; 3],
    values: [f64; 3],
    weights: [f64; 3],
}

/// A relief as stage files hold it: its triangles where it is not zero, their corners and its
/// values there.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
struct Stored {
    points: Vec<Point>,
    triangles: Vec<[u32; 3]>,
    values: Vec<f64>,
}

/// What stage files hold for a relief: its triangles, or, from older files, a block on a grid.
#[derive(Deserialize)]
#[serde(untagged)]
enum Read {
    Triangles(Stored),
    Grid { grid: OldGrid, start: [i64; 2], size: [usize; 2], values: Vec<f64> },
}

/// The regular grid older stage files put reliefs on: vertex `(i, j)` at `origin + cell·(i, j)`.
#[derive(Deserialize)]
struct OldGrid {
    origin: Point,
    cell: f64,
}

impl Serialize for Relief {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.trimmed().serialize(s)
    }
}

impl<'de> Deserialize<'de> for Relief {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Relief, D::Error> {
        let made = match Read::deserialize(d)? {
            Read::Triangles(Stored { points, triangles, values }) => Relief::new(points, triangles, values),
            Read::Grid { grid, start, size, values } => {
                Relief::from_grid(grid.origin, grid.cell, start, size, &values)
            }
        };
        made.map_err(serde::de::Error::custom)
    }
}

/// The sides of a triangle, each from a corner to the next.
fn sides<T: Copy>([p, q, r]: [T; 3]) -> [(T, T); 3] {
    [(p, q), (q, r), (r, p)]
}

/// The corners of the triangle `[i, j, k]` of `points`, when they are there.
fn corners_of(points: &[Point], &[i, j, k]: &[u32; 3]) -> Option<[Point; 3]> {
    Some([*points.get(i as usize)?, *points.get(j as usize)?, *points.get(k as usize)?])
}

/// The box from the lowest to the highest coordinates of the corners.
fn bounds(c: [Point; 3]) -> (Point, Point) {
    (
        [c[0][0].min(c[1][0]).min(c[2][0]), c[0][1].min(c[1][1]).min(c[2][1])],
        [c[0][0].max(c[1][0]).max(c[2][0]), c[0][1].max(c[1][1]).max(c[2][1])],
    )
}

/// The barycentric coordinates of `p` in the triangle, when it has an area.
fn barycentric([a, b, c]: [Point; 3], p: Point) -> Option<[f64; 3]> {
    let det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
    if !(det.is_finite() && det.abs() > f64::MIN_POSITIVE) {
        return None;
    }
    let l1 = ((p[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (p[1] - a[1])) / det;
    let l2 = ((b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1])) / det;
    Some([1.0 - l1 - l2, l1, l2])
}

/// The gradient of the linear function taking `v` at the corners, when they span an area.
fn gradient_of([a, b, c]: [Point; 3], v: [f64; 3]) -> Option<[f64; 2]> {
    let (e1, e2) = ([b[0] - a[0], b[1] - a[1]], [c[0] - a[0], c[1] - a[1]]);
    let det = e1[0] * e2[1] - e1[1] * e2[0];
    if !(det.is_finite() && det.abs() > f64::MIN_POSITIVE) {
        return None;
    }
    let (d1, d2) = (v[1] - v[0], v[2] - v[0]);
    Some([(d1 * e2[1] - d2 * e1[1]) / det, (e1[0] * d2 - e2[0] * d1) / det])
}

/// Where the segment from `a` to `b` crosses the edge from `u` to `v`: the fraction of the way
/// along the segment, when they cross at a single point.
fn crossing(a: Point, b: Point, u: Point, v: Point) -> Option<f64> {
    let (d, e) = ([b[0] - a[0], b[1] - a[1]], [v[0] - u[0], v[1] - u[1]]);
    let det = d[0] * e[1] - d[1] * e[0];
    if !(det.is_finite() && det.abs() > f64::MIN_POSITIVE) {
        return None;
    }
    let w = [u[0] - a[0], u[1] - a[1]];
    let (t, s) = ((w[0] * e[1] - w[1] * e[0]) / det, (w[0] * d[1] - w[1] * d[0]) / det);
    ((0.0..=1.0).contains(&s) && t > 0.0 && t < 1.0).then_some(t)
}

/// Whether two triangles' insides meet: no side of either has the other wholly on its outer
/// side, or on it, decided exactly.
fn overlap(p: [Point; 3], q: [Point; 3]) -> bool {
    let separates = |s: [Point; 3], o: [Point; 3]| {
        let turn = orientation(s[0], s[1], s[2]);
        sides(s).into_iter().any(|(a, b)| o.iter().all(|&c| orientation(a, b, c) != turn))
    };
    !(separates(p, q) || separates(q, p))
}

impl Carrier {
    /// The carrier of `triangles`, index triples into `points`. It refuses coordinates that are
    /// not finite or lie past `LIMIT_MM`, more than `MAX_POINTS` corners, and triangles naming a
    /// corner that is not there.
    pub fn new(points: Vec<Point>, triangles: Vec<[u32; 3]>) -> Result<Carrier, GeomError> {
        if points.len() > MAX_POINTS || triangles.len() > 2 * MAX_POINTS {
            return Err(GeomError::Engine(format!(
                "a relief of {} corners and {} triangles is too large",
                points.len(),
                triangles.len()
            )));
        }
        for p in &points {
            check_value(p[0])?;
            check_value(p[1])?;
        }
        if triangles.iter().flatten().any(|&i| i as usize >= points.len()) {
            return Err(GeomError::Engine("a relief names a corner it does not hold".into()));
        }
        // Inside: the corners of a triangle, but for those along an edge one triangle alone has.
        let mut inside = vec![false; points.len()];
        for &i in triangles.iter().flatten() {
            if let Some(slot) = inside.get_mut(i as usize) {
                *slot = true;
            }
        }
        let mut edges: Vec<(u32, u32)> =
            triangles.iter().flat_map(|&t| sides(t)).map(|(a, b)| (a.min(b), a.max(b))).collect();
        edges.sort_unstable();
        for run in edges.chunk_by(|e, f| e == f) {
            if let [(a, b)] = run {
                for i in [a, b] {
                    if let Some(slot) = inside.get_mut(*i as usize) {
                        *slot = false;
                    }
                }
            }
        }
        let rim = (0..points.len() as u32).filter(|&i| inside.get(i as usize) != Some(&true)).collect();
        let buckets = Buckets::over(triangles.len(), |t| corners_of(&points, triangles.get(t)?));
        Ok(Carrier(Arc::new(Tiling { points, triangles, buckets, rim })))
    }

    /// The relief taking `values` at the corners. It refuses one value more or fewer than the
    /// corners, values that are not finite or lie past `LIMIT_MM`, and values other than zero
    /// along the triangles' outer edges or at corners of none.
    pub fn relief(&self, values: Vec<f64>) -> Result<Relief, GeomError> {
        let count = self.0.points.len();
        if values.len() != count {
            return Err(GeomError::Engine(format!(
                "a relief of {count} corners holds {} values",
                values.len()
            )));
        }
        let mut range = [0.0f64; 2];
        for &v in &values {
            check_value(v)?;
            range = [range[0].min(v), range[1].max(v)];
        }
        if self.0.rim.iter().any(|&i| values.get(i as usize).is_some_and(|&v| v != 0.0)) {
            return Err(GeomError::Engine("a relief is not zero along its triangles' outer edges".into()));
        }
        Ok(Relief { carrier: self.clone(), values: values.into(), range })
    }
}

impl Relief {
    /// The relief that is zero everywhere.
    pub fn zero() -> Relief {
        Relief::default()
    }

    /// The relief taking `values` at `points` and linear on `triangles`, index triples into
    /// them, on a carrier of its own: as `Carrier::new` and `Carrier::relief` check them.
    pub fn new(points: Vec<Point>, triangles: Vec<[u32; 3]>, values: Vec<f64>) -> Result<Relief, GeomError> {
        Carrier::new(points, triangles)?.relief(values)
    }

    /// The relief with `values`, row after row, on the block of `size` vertices from `start` of
    /// the grid whose vertex `(i, j)` lies at `origin + cell·(i, j)`, zero a square past the
    /// block, each square split by its diagonal from `(i, j)` to `(i + 1, j + 1)`.
    pub fn from_grid(
        origin: Point,
        cell: f64,
        start: [i64; 2],
        size: [usize; 2],
        values: &[f64],
    ) -> Result<Relief, GeomError> {
        check_value(origin[0])?;
        check_value(origin[1])?;
        if check_value(cell)? < 1e-3 {
            return Err(GeomError::OutOfRange(cell));
        }
        let count = size[0].checked_mul(size[1]).filter(|&n| n <= MAX_POINTS);
        if count != Some(values.len()) {
            return Err(GeomError::Engine(format!("a relief of {size:?} vertices holds {}", values.len())));
        }
        let reach = (LIMIT_MM / cell) as i64;
        if start.iter().any(|s| s.abs() > reach) {
            return Err(GeomError::OutOfRange(start[0] as f64 * cell));
        }
        if values.is_empty() {
            return Ok(Relief::zero());
        }
        // The block widened by a ring of zeros.
        let (w, h) = (size[0] + 2, size[1] + 2);
        let value = |i: usize, j: usize| -> f64 {
            if i == 0 || j == 0 || i > size[0] || j > size[1] {
                return 0.0;
            }
            values.get((j - 1) * size[0] + (i - 1)).copied().unwrap_or(0.0)
        };
        let mut points = Vec::with_capacity(w * h);
        let mut at = Vec::with_capacity(w * h);
        for j in 0..h {
            for i in 0..w {
                let (gi, gj) = (start[0] - 1 + i as i64, start[1] - 1 + j as i64);
                points.push([origin[0] + gi as f64 * cell, origin[1] + gj as f64 * cell]);
                at.push(value(i, j));
            }
        }
        let mut triangles = Vec::with_capacity(2 * (w - 1) * (h - 1));
        let id = |i: usize, j: usize| (j * w + i) as u32;
        for j in 0..h - 1 {
            for i in 0..w - 1 {
                triangles.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
                triangles.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        Relief::new(points, triangles, at)
    }

    /// The value at each of its carrier's corners.
    pub fn values(&self) -> &[f64] {
        &self.values
    }

    /// Whether the relief is zero everywhere.
    pub fn is_zero(&self) -> bool {
        self.range == [0.0, 0.0]
    }

    fn tiling(&self) -> &Tiling {
        &self.carrier.0
    }

    /// The corners of triangle `t` and the values there, when the relief is not zero at all of
    /// them.
    fn nonzero_triangle(&self, t: u32) -> Option<([Point; 3], [f64; 3])> {
        let tri = self.tiling().triangles.get(t as usize)?;
        let value = |i: u32| self.values.get(i as usize).copied();
        let v = [value(tri[0])?, value(tri[1])?, value(tri[2])?];
        if v.iter().all(|&x| x == 0.0) {
            return None;
        }
        Some((corners_of(&self.tiling().points, tri)?, v))
    }

    /// The triangle holding `p`: of the relief's triangles whose box may hold it, the one it
    /// lies deepest inside, if no further outside than rounding.
    fn locate(&self, p: Point) -> Option<Located> {
        let mut best: Option<(Located, f64)> = None;
        for &t in self.tiling().buckets.near(p) {
            let Some((corners, values)) = self.nonzero_triangle(t) else { continue };
            let Some(weights) = barycentric(corners, p) else { continue };
            let depth = weights[0].min(weights[1]).min(weights[2]);
            if depth >= -ON_TRIANGLE && best.as_ref().is_none_or(|(_, d)| depth > *d) {
                best = Some((Located { corners, values, weights }, depth));
            }
        }
        best.map(|(found, _)| found)
    }

    /// The value at a point of the plate.
    pub fn at(&self, p: Point) -> f64 {
        let Some(Located { values: v, weights: l, .. }) = self.locate(p) else { return 0.0 };
        l[0] * v[0] + l[1] * v[1] + l[2] * v[2]
    }

    /// The gradient at a point of the plate, in mm per mm: that of the triangle holding it.
    pub fn gradient(&self, p: Point) -> [f64; 2] {
        self.locate(p).and_then(|t| gradient_of(t.corners, t.values)).unwrap_or([0.0, 0.0])
    }

    fn facet(&self, t: u32) -> Option<([Point; 3], [f64; 2])> {
        let (c, v) = self.nonzero_triangle(t)?;
        Some((c, gradient_of(c, v)?))
    }

    /// Each of its triangles' corners and gradient.
    pub fn facets(&self) -> impl Iterator<Item = ([Point; 3], [f64; 2])> + '_ {
        (0..self.tiling().triangles.len() as u32).filter_map(|t| self.facet(t))
    }

    /// The corners and gradient of each of its triangles whose box may meet the box from `lo`
    /// to `hi`.
    pub fn facets_over(&self, lo: Point, hi: Point) -> impl Iterator<Item = ([Point; 3], [f64; 2])> + '_ {
        self.tiling().buckets.within(lo, hi).into_iter().filter_map(|t| self.facet(t))
    }

    /// The fractions of the way from `a` to `b` where the relief may bend, in order, strictly
    /// between the ends: between them it is linear.
    pub fn breaks(&self, a: Point, b: Point) -> Vec<f64> {
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        if self.is_zero() || length.is_nan() || length <= 0.0 {
            return Vec::new();
        }
        let mut ts: Vec<f64> = Vec::new();
        for t in self.tiling().buckets.along(a, b) {
            let Some((c, _)) = self.nonzero_triangle(t) else { continue };
            for (u, v) in sides(c) {
                ts.extend(crossing(a, b, u, v));
            }
        }
        ts.sort_by(f64::total_cmp);
        let mut out: Vec<f64> = Vec::with_capacity(ts.len());
        for t in ts {
            if out.last().is_none_or(|&last| (t - last) * length > SAME_BREAK_MM) {
                out.push(t);
            }
        }
        out
    }

    /// The lowest and highest values over the plate, zero included.
    pub fn range(&self) -> [f64; 2] {
        self.range
    }

    /// The highest value along the segment from `a` to `b`.
    pub fn max_along(&self, a: Point, b: Point) -> f64 {
        let along = |t: f64| self.at([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
        self.breaks(a, b).into_iter().map(along).fold(self.at(a).max(self.at(b)), f64::max)
    }

    /// The lowest and highest values within `radius` of `p`, or over the square about it.
    pub fn range_within(&self, p: Point, radius: f64) -> [f64; 2] {
        self.range_over([p[0] - radius, p[1] - radius], [p[0] + radius, p[1] + radius])
    }

    /// The lowest and highest values over the box from `lo` to `hi`, exactly: a linear function
    /// on each triangle, zero past them, the relief reaches them at the box's corners, where its
    /// sides cross the triangles' edges, or at corners inside it.
    pub fn range_over(&self, lo: Point, hi: Point) -> [f64; 2] {
        if self.is_zero() {
            return [0.0, 0.0];
        }
        if !(lo.iter().chain(&hi).all(|v| v.is_finite())) {
            return self.range();
        }
        let corners = [lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]];
        let mut range = [f64::INFINITY, f64::NEG_INFINITY];
        let mut take = |v: f64| range = [range[0].min(v), range[1].max(v)];
        for (k, &c) in corners.iter().enumerate() {
            let d = corners.get((k + 1) % 4).copied().unwrap_or(c);
            take(self.at(c));
            for t in self.breaks(c, d) {
                take(self.at([c[0] + t * (d[0] - c[0]), c[1] + t * (d[1] - c[1])]));
            }
        }
        let inside = |p: &Point| p[0] >= lo[0] && p[0] <= hi[0] && p[1] >= lo[1] && p[1] <= hi[1];
        for t in self.tiling().buckets.within(lo, hi) {
            let Some((c, v)) = self.nonzero_triangle(t) else { continue };
            for (p, value) in c.iter().zip(v) {
                if inside(p) {
                    take(value);
                }
            }
        }
        range
    }

    /// The steepest slope of the relief, in mm per mm, over its triangles.
    pub fn steepest(&self) -> f64 {
        self.facets().map(|(_, g)| g[0].hypot(g[1])).fold(0.0, f64::max)
    }

    /// The relief as stage files hold it: its triangles where it is not zero, in order, and their
    /// corners and its values there, in the order the triangles first name them.
    fn trimmed(&self) -> Stored {
        let tiling = self.tiling();
        let mut renumber = vec![u32::MAX; tiling.points.len()];
        let mut out = Stored { points: Vec::new(), triangles: Vec::new(), values: Vec::new() };
        for (t, tri) in tiling.triangles.iter().enumerate() {
            let Some((corners, values)) = self.nonzero_triangle(t as u32) else { continue };
            let mut ids = [0u32; 3];
            for (((slot, &i), p), v) in ids.iter_mut().zip(tri).zip(corners).zip(values) {
                let Some(r) = renumber.get_mut(i as usize) else { continue };
                if *r == u32::MAX {
                    *r = out.points.len() as u32;
                    out.points.push(p);
                    out.values.push(v);
                }
                *slot = *r;
            }
            out.triangles.push(ids);
        }
        out
    }

    /// The relief `f(a, b)` of two reliefs, corner by corner, where `f(0, 0) = 0`: on their
    /// carrier when they share one, else on their triangles together, where those of each are
    /// either the other's own or lie apart from them; reliefs whose triangles overlap otherwise
    /// are refused.
    pub fn combine(a: &Relief, b: &Relief, f: impl Fn(f64, f64) -> f64) -> Result<Relief, GeomError> {
        if Arc::ptr_eq(&a.carrier.0, &b.carrier.0) {
            return a.carrier.relief(a.values.iter().zip(b.values.iter()).map(|(&x, &y)| f(x, y)).collect());
        }
        let key = |p: &Point| (p[0].to_bits(), p[1].to_bits());
        let Stored { mut points, mut triangles, values: own } = a.trimmed();
        let mut index: HashMap<(u64, u64), u32> =
            points.iter().enumerate().map(|(i, p)| (key(p), i as u32)).collect();
        let sorted = |t: [u32; 3]| {
            let mut s = t;
            s.sort_unstable();
            s
        };
        let shared: HashSet<[u32; 3]> = triangles.iter().map(|&t| sorted(t)).collect();
        for t in 0..b.tiling().triangles.len() as u32 {
            let Some((corners, _)) = b.nonzero_triangle(t) else { continue };
            if let [Some(x), Some(y), Some(z)] = corners.map(|p| index.get(&key(&p)).copied())
                && shared.contains(&sorted([x, y, z]))
            {
                continue;
            }
            let (lo, hi) = bounds(corners);
            let meets = |s: u32| a.nonzero_triangle(s).is_some_and(|(c, _)| overlap(c, corners));
            if a.tiling().buckets.within(lo, hi).into_iter().any(meets) {
                return Err(GeomError::Engine("reliefs on overlapping triangles of their own".into()));
            }
            let mut ids = [0u32; 3];
            for (slot, p) in ids.iter_mut().zip(corners) {
                *slot = *index.entry(key(&p)).or_insert_with(|| {
                    points.push(p);
                    (points.len() - 1) as u32
                });
            }
            triangles.push(ids);
        }
        // Each relief's value at every corner, the first's as it holds it at its own.
        let values: Vec<f64> = points
            .iter()
            .enumerate()
            .map(|(i, p)| f(own.get(i).copied().unwrap_or_else(|| a.at(*p)), b.at(*p)))
            .collect();
        Relief::new(points, triangles, values)
    }
}
