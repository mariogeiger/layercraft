//! Reliefs: piecewise linear heights over the plate, on triangles every layer of a print shares.
//!
//! A carrier holds triangles and their corners; a warp, a value at each corner; a relief is a
//! warp on its carrier. It is linear on each triangle, zero past them and zero along their
//! outer edges, so it is continuous. A triangle where it is zero at every corner takes no part:
//! every query passes it by, so a relief behaves as the triangles where it is not zero alone.
//! Along a segment it is linear between the points where the segment crosses those triangles'
//! edges: cut there (`Relief::breaks`), the segment carries it exactly, as straight pieces. Its
//! extremes over a box lie at the box's corners, where the box's sides cross those edges, or at
//! corners inside it. A stage file holds its print's carrier once, its corners and triangles,
//! and each layer's warp as its values.

use crate::locate::Buckets;
use crate::orient::orientation;
use crate::{GRID_MM, GeomError, LIMIT_MM, Point, check_value};
use serde::de::{Error as _, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

/// The most corners a carrier holds: a plate 400 mm square, a corner every 0.1 mm.
pub const MAX_POINTS: usize = 1 << 24;
/// How far outside a triangle, as a fraction of it, a point still counts as on it: rounding.
const ON_TRIANGLE: f64 = 1e-9;
/// Breaks closer than this along a segment, in mm, are one: a tenth of the lattice step.
const SAME_BREAK_MM: f64 = GRID_MM / 10.0;

/// Triangles over the plate with their corners, which the warps of a print's layers share.
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

impl PartialEq for Carrier {
    fn eq(&self, other: &Carrier) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
            || (self.0.points == other.0.points && self.0.triangles == other.0.triangles)
    }
}

/// A carrier as stage files hold it.
#[derive(Serialize, Deserialize)]
struct StoredCarrier<P, T> {
    points: P,
    triangles: T,
}

impl Serialize for Carrier {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        StoredCarrier { points: &self.0.points, triangles: &self.0.triangles }.serialize(s)
    }
}

impl<'de> Deserialize<'de> for Carrier {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Carrier, D::Error> {
        let stored = StoredCarrier::<Vec<Point>, Vec<[u32; 3]>>::deserialize(d)?;
        Carrier::new(stored.points, stored.triangles).map_err(D::Error::custom)
    }
}

/// A layer's displacement at each corner of its print's carrier, in mm: flat when it holds no
/// value, as it does when every value would be zero.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Warp {
    values: Arc<[f64]>,
    /// The lowest and highest values, zero included.
    range: [f64; 2],
}

impl Serialize for Warp {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.values.as_ref().serialize(s)
    }
}

/// The values of a warp as stage files hold them, one at each corner.
struct Values;

impl<'de> Visitor<'de> for Values {
    type Value = Vec<f64>;

    fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str(
            "a warp's value at each corner of its stage's carrier (stage files from before 0.16.0, \
             whose bent layers hold triangles of their own, must be made again)",
        )
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Vec<f64>, A::Error> {
        let mut values = Vec::with_capacity(seq.size_hint().unwrap_or(0).min(MAX_POINTS));
        while let Some(v) = seq.next_element()? {
            if values.len() >= MAX_POINTS {
                return Err(A::Error::custom(format!("a warp of more than {MAX_POINTS} values")));
            }
            values.push(v);
        }
        Ok(values)
    }
}

impl<'de> Deserialize<'de> for Warp {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Warp, D::Error> {
        Warp::new(d.deserialize_seq(Values)?).map_err(D::Error::custom)
    }
}

/// A piecewise linear height over the plate: a warp on its carrier.
#[derive(Debug, Clone, Default)]
pub struct Relief {
    carrier: Carrier,
    warp: Warp,
}

/// A triangle of a relief holding a point: its corners, the relief's values there and the
/// point's barycentric coordinates in it.
struct Located {
    corners: [Point; 3],
    values: [f64; 3],
    weights: [f64; 3],
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
    /// not finite or lie past `LIMIT_MM`, more than `MAX_POINTS` corners or twice as many
    /// triangles, more than a triangulation of that many corners has, and triangles naming a
    /// corner that is not there.
    pub fn new(points: Vec<Point>, triangles: Vec<[u32; 3]>) -> Result<Carrier, GeomError> {
        if points.len() > MAX_POINTS || triangles.len() > 2 * MAX_POINTS {
            return Err(GeomError::Engine(format!(
                "a carrier of {} corners and {} triangles is too large",
                points.len(),
                triangles.len()
            )));
        }
        for p in &points {
            check_value(p[0])?;
            check_value(p[1])?;
        }
        if triangles.iter().flatten().any(|&i| i as usize >= points.len()) {
            return Err(GeomError::Engine("a carrier names a corner it does not hold".into()));
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

    /// How many corners it holds.
    pub fn len(&self) -> usize {
        self.0.points.len()
    }

    /// Whether it holds no corner.
    pub fn is_empty(&self) -> bool {
        self.0.points.is_empty()
    }

    /// The relief of `warp` on it, the relief zero for a flat warp. It refuses a warp of one
    /// value more or fewer than the corners, and values other than zero along the triangles'
    /// outer edges or at corners of none.
    pub fn relief(&self, warp: &Warp) -> Result<Relief, GeomError> {
        if !warp.is_zero() {
            let count = self.0.points.len();
            if warp.values.len() != count {
                return Err(GeomError::Engine(format!(
                    "a warp of {} values on a carrier of {count} corners",
                    warp.values.len()
                )));
            }
            if self.0.rim.iter().any(|&i| warp.values.get(i as usize).is_some_and(|&v| v != 0.0)) {
                return Err(GeomError::Engine("a warp is not zero along its triangles' outer edges".into()));
            }
        }
        Ok(Relief { carrier: self.clone(), warp: warp.clone() })
    }

    /// The carrier of its triangles where some of `warps` is not zero, and those warps on it,
    /// each checked as `relief` checks it. Its corners keep their order.
    pub fn trim(&self, warps: &[Warp]) -> Result<(Carrier, Vec<Warp>), GeomError> {
        for warp in warps {
            self.relief(warp)?;
        }
        let tiling = &self.0;
        let bends = |i: u32| warps.iter().any(|w| w.values.get(i as usize).is_some_and(|&v| v != 0.0));
        let kept: Vec<[u32; 3]> =
            tiling.triangles.iter().copied().filter(|t| t.iter().any(|&i| bends(i))).collect();
        let mut renumber = vec![u32::MAX; tiling.points.len()];
        for &i in kept.iter().flatten() {
            if let Some(slot) = renumber.get_mut(i as usize) {
                *slot = 0;
            }
        }
        let mut points = Vec::new();
        for (slot, p) in renumber.iter_mut().zip(&tiling.points).filter(|(slot, _)| **slot == 0) {
            *slot = points.len() as u32;
            points.push(*p);
        }
        let at = |i: u32| renumber.get(i as usize).copied().unwrap_or(0);
        let triangles = kept.iter().map(|t| t.map(at)).collect();
        let carrier = Carrier::new(points, triangles)?;
        let count = carrier.0.points.len();
        let moved = warps
            .iter()
            .map(|w| {
                let mut values = vec![0.0; if w.is_zero() { 0 } else { count }];
                for (&to, &v) in renumber.iter().zip(w.values.iter()) {
                    if let Some(slot) = values.get_mut(to as usize) {
                        *slot = v;
                    }
                }
                Warp::new(values)
            })
            .collect::<Result<Vec<Warp>, GeomError>>()?;
        Ok((carrier, moved))
    }

    /// The carrier of its triangles and `other`'s together, its corners in their places and
    /// with where each of `other`'s lies: a triangle of `other` on the corners of one of its own
    /// is that one. It refuses triangles of `other` that overlap its own otherwise.
    pub fn union(&self, other: &Carrier) -> Result<(Carrier, Vec<u32>), GeomError> {
        let (a, b) = (&self.0, &other.0);
        // A corner is its place: adding zero makes −0 the 0 it is.
        let key = |p: &Point| ((p[0] + 0.0).to_bits(), (p[1] + 0.0).to_bits());
        let (mut points, mut triangles) = (a.points.clone(), a.triangles.clone());
        let mut index: HashMap<(u64, u64), u32> =
            points.iter().enumerate().map(|(i, p)| (key(p), i as u32)).collect();
        let sorted = |t: [u32; 3]| {
            let mut s = t;
            s.sort_unstable();
            s
        };
        let own: HashSet<[u32; 3]> = triangles.iter().map(|&t| sorted(t)).collect();
        for tri in &b.triangles {
            let Some(corners) = corners_of(&b.points, tri) else { continue };
            if let [Some(x), Some(y), Some(z)] = corners.map(|p| index.get(&key(&p)).copied())
                && own.contains(&sorted([x, y, z]))
            {
                continue;
            }
            let (lo, hi) = bounds(corners);
            let meets = |s: u32| {
                a.triangles
                    .get(s as usize)
                    .and_then(|t| corners_of(&a.points, t))
                    .is_some_and(|c| overlap(c, corners))
            };
            if a.buckets.within(lo, hi).into_iter().any(meets) {
                return Err(GeomError::Engine("carriers whose triangles overlap".into()));
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
        let mut places = Vec::with_capacity(b.points.len());
        for p in &b.points {
            places.push(*index.entry(key(p)).or_insert_with(|| {
                points.push(*p);
                (points.len() - 1) as u32
            }));
        }
        Ok((Carrier::new(points, triangles)?, places))
    }
}

impl Warp {
    /// The warp taking `values` at the corners, in their order. It refuses values that are not
    /// finite or lie past `LIMIT_MM`, and more than `MAX_POINTS` of them.
    pub fn new(values: Vec<f64>) -> Result<Warp, GeomError> {
        if values.len() > MAX_POINTS {
            return Err(GeomError::Engine(format!("a warp of {} values is too large", values.len())));
        }
        let mut range = [0.0f64; 2];
        for &v in &values {
            check_value(v)?;
            range = [range[0].min(v), range[1].max(v)];
        }
        if range == [0.0, 0.0] {
            return Ok(Warp::default());
        }
        Ok(Warp { values: values.into(), range })
    }

    /// The value at each corner of its carrier; none when flat.
    pub fn values(&self) -> &[f64] {
        &self.values
    }

    /// Whether it is flat: zero everywhere.
    pub fn is_zero(&self) -> bool {
        self.values.is_empty()
    }

    /// The lowest and highest values, zero included.
    pub fn range(&self) -> [f64; 2] {
        self.range
    }
}

impl Relief {
    /// The relief that is zero everywhere.
    pub fn zero() -> Relief {
        Relief::default()
    }

    /// The relief taking `values` at `points` and linear on `triangles`, index triples into
    /// them, on a carrier of its own: as `Carrier::new`, `Warp::new` and `Carrier::relief`
    /// check them.
    pub fn new(points: Vec<Point>, triangles: Vec<[u32; 3]>, values: Vec<f64>) -> Result<Relief, GeomError> {
        Carrier::new(points, triangles)?.relief(&Warp::new(values)?)
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

    /// The carrier it lies on.
    pub fn carrier(&self) -> &Carrier {
        &self.carrier
    }

    /// Its warp, on its carrier.
    pub fn warp(&self) -> &Warp {
        &self.warp
    }

    /// Whether the relief is zero everywhere.
    pub fn is_zero(&self) -> bool {
        self.warp.is_zero()
    }

    fn tiling(&self) -> &Tiling {
        &self.carrier.0
    }

    /// The corners of triangle `t` and the values there, when the relief is not zero at all of
    /// them.
    fn nonzero_triangle(&self, t: u32) -> Option<([Point; 3], [f64; 3])> {
        let tri = self.tiling().triangles.get(t as usize)?;
        let value = |i: u32| self.warp.values.get(i as usize).copied();
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
        self.warp.range
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
}
