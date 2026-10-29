//! Reliefs: piecewise linear heights over the plate, all on one regular triangulated grid.
//!
//! The grid's vertices lie at `origin + cell·(i, j)`. In grid coordinates `(u, v)`, one cell a
//! unit, three families of lines cut the plane: `u = n`, `v = n` and `u − v = n` for every
//! integer `n`, the last splitting each square by its diagonal from `(i, j)` to `(i + 1, j + 1)`.
//! A relief is linear on each of the triangles they leave. It holds its values on a block of
//! vertices and is zero past it, which it reaches across one cell, so it bends only on the lines
//! through the block widened by a cell (`Lines`). Along a segment a relief is linear between the
//! points where the segment crosses those lines: cut there (`Relief::breaks`), the segment
//! carries it exactly, as straight pieces.

use crate::{GeomError, LIMIT_MM, Point, check_value};
use serde::{Deserialize, Serialize};

/// The most values a relief holds: a plate 400 mm square at a 0.1 mm cell.
pub const MAX_VALUES: usize = 1 << 24;

/// The regular triangulated grid reliefs live on.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Grid {
    /// The vertex `(0, 0)`, in mm.
    pub origin: Point,
    /// The side of a square, in mm.
    pub cell: f64,
}

impl Grid {
    /// The grid coordinates of a point of the plate.
    pub fn coords(&self, p: Point) -> [f64; 2] {
        [(p[0] - self.origin[0]) / self.cell, (p[1] - self.origin[1]) / self.cell]
    }

    /// The vertex `(i, j)` on the plate.
    pub fn vertex(&self, i: i64, j: i64) -> Point {
        [self.origin[0] + i as f64 * self.cell, self.origin[1] + j as f64 * self.cell]
    }

    /// Whether the grid is usable: finite, inside `LIMIT_MM`, its cell at least a micrometre.
    pub fn check(&self) -> Result<(), GeomError> {
        check_value(self.origin[0])?;
        check_value(self.origin[1])?;
        if check_value(self.cell)? < 1e-3 {
            return Err(GeomError::OutOfRange(self.cell));
        }
        Ok(())
    }
}

/// The level of each family of grid lines at grid coordinates `c`: `u`, `v` and `u − v`.
pub fn levels(c: [f64; 2]) -> [f64; 3] {
    [c[0], c[1], c[0] - c[1]]
}

/// Where a segment crosses a grid line: the fraction `t` of the way along it, and the line, by
/// its family (0: `u`, 1: `v`, 2: `u − v`) and level.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Crossing {
    pub t: f64,
    pub family: usize,
    pub level: i64,
}

/// A set of grid lines: in each family, the levels from the first bound to the second.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lines {
    pub levels: [[i64; 2]; 3],
}

impl Lines {
    /// The lines of both sets, and those between.
    pub fn union(self, other: Lines) -> Lines {
        let mut levels = self.levels;
        for (l, o) in levels.iter_mut().zip(other.levels) {
            *l = [l[0].min(o[0]), l[1].max(o[1])];
        }
        Lines { levels }
    }

    /// Where the segment from `a` to `b` (grid coordinates) crosses the lines, in order along
    /// it; crossings at one fraction come by family, then level. A point lies on a line's upper
    /// side when its level is at least the line's, so the segment crosses the lines its ends lie
    /// on different sides of, one end possibly on the line. The result depends only on the two
    /// ends, in that order.
    pub fn crossings(&self, a: [f64; 2], b: [f64; 2]) -> Vec<Crossing> {
        let (la, lb) = (levels(a), levels(b));
        let mut out = Vec::new();
        for (family, ((&from, &to), range)) in la.iter().zip(&lb).zip(&self.levels).enumerate() {
            if !(from.is_finite() && to.is_finite()) || from == to {
                continue;
            }
            // The levels above the lower end, up to the upper one, within the set.
            let (lo, hi) = (from.min(to), from.max(to));
            let first = (lo.floor() as i64).saturating_add(1).max(range[0]);
            let last = (hi.floor() as i64).min(range[1]);
            // A segment crosses at most a plate's width of lines.
            if last.saturating_sub(first) > 1 << 20 {
                continue;
            }
            for level in first..=last {
                let t = ((level as f64 - from) / (to - from)).clamp(0.0, 1.0);
                out.push(Crossing { t, family, level });
            }
        }
        out.sort_by(|x, y| x.t.total_cmp(&y.t).then(x.family.cmp(&y.family)).then(x.level.cmp(&y.level)));
        out
    }
}

/// A piecewise linear height on a grid: its values on a block of vertices, zero past it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Relief {
    pub grid: Grid,
    /// The block's first vertex `(i, j)`.
    pub start: [i64; 2],
    /// The block's columns and rows of vertices.
    pub size: [usize; 2],
    /// The values at the block's vertices, row after row.
    pub values: Vec<f64>,
}

impl Default for Relief {
    /// Zero, on a grid of millimetre squares from the origin.
    fn default() -> Relief {
        Relief::zero(Grid { origin: [0.0, 0.0], cell: 1.0 })
    }
}

impl Relief {
    /// The relief that is zero everywhere.
    pub fn zero(grid: Grid) -> Relief {
        Relief { grid, start: [0, 0], size: [0, 0], values: Vec::new() }
    }

    /// The relief with `value(i, j)` on the block from `start`, `size` vertices, trimmed of its
    /// zero rows and columns.
    pub fn from_fn(grid: Grid, start: [i64; 2], size: [usize; 2], value: impl Fn(i64, i64) -> f64) -> Relief {
        let mut values = Vec::with_capacity(size[0].saturating_mul(size[1]).min(MAX_VALUES));
        for j in 0..size[1] as i64 {
            for i in 0..size[0] as i64 {
                values.push(value(start[0] + i, start[1] + j));
            }
        }
        Relief { grid, start, size, values }.trimmed()
    }

    /// The same relief on the smallest block holding its non-zero values.
    pub fn trimmed(self) -> Relief {
        let [w, _] = self.size;
        let mut lo = [usize::MAX; 2];
        let mut hi = [0usize; 2];
        for (k, v) in self.values.iter().enumerate() {
            if *v != 0.0 && w > 0 {
                let (i, j) = (k % w, k / w);
                lo = [lo[0].min(i), lo[1].min(j)];
                hi = [hi[0].max(i), hi[1].max(j)];
            }
        }
        if lo[0] > hi[0] {
            return Relief::zero(self.grid);
        }
        let size = [hi[0] - lo[0] + 1, hi[1] - lo[1] + 1];
        let mut values = Vec::with_capacity(size[0] * size[1]);
        for j in lo[1]..=hi[1] {
            values.extend(self.values.get(j * w + lo[0]..=j * w + hi[0]).into_iter().flatten());
        }
        Relief {
            grid: self.grid,
            start: [self.start[0] + lo[0] as i64, self.start[1] + lo[1] as i64],
            size,
            values,
        }
    }

    /// Whether the values are sound: the grid usable, as many values as the block has vertices,
    /// no more than `MAX_VALUES`, each finite and inside `LIMIT_MM`, the block on the plate.
    pub fn check(&self) -> Result<(), GeomError> {
        self.grid.check()?;
        let count = self.size[0].checked_mul(self.size[1]).filter(|&n| n <= MAX_VALUES);
        if count != Some(self.values.len()) {
            return Err(GeomError::Engine(format!(
                "a relief of {:?} vertices holds {}",
                self.size,
                self.values.len()
            )));
        }
        let reach = (LIMIT_MM / self.grid.cell) as i64;
        if self.start.iter().any(|s| s.abs() > reach) {
            return Err(GeomError::OutOfRange(self.start[0] as f64 * self.grid.cell));
        }
        self.values.iter().try_for_each(|&v| check_value(v).map(|_| ()))
    }

    /// Whether the relief is zero everywhere.
    pub fn is_zero(&self) -> bool {
        self.values.iter().all(|&v| v == 0.0)
    }

    /// The value at vertex `(i, j)`.
    pub fn value(&self, i: i64, j: i64) -> f64 {
        let (Some(di), Some(dj)) = (i.checked_sub(self.start[0]), j.checked_sub(self.start[1])) else {
            return 0.0;
        };
        let (Ok(di), Ok(dj)) = (usize::try_from(di), usize::try_from(dj)) else { return 0.0 };
        if di >= self.size[0] || dj >= self.size[1] {
            return 0.0;
        }
        self.values.get(dj * self.size[0] + di).copied().unwrap_or(0.0)
    }

    /// The triangle holding grid coordinates `c`, by its square `(i, j)` and whether it is the
    /// lower one (`u − i ≥ v − j`), with the place in its square.
    fn locate(c: [f64; 2]) -> (i64, i64, bool, f64, f64) {
        let (i, j) = (c[0].floor(), c[1].floor());
        let (fu, fv) = (c[0] - i, c[1] - j);
        (i as i64, j as i64, fu >= fv, fu, fv)
    }

    /// The value at a point of the plate.
    pub fn at(&self, p: Point) -> f64 {
        if self.values.is_empty() {
            return 0.0;
        }
        let (i, j, lower, fu, fv) = Relief::locate(self.grid.coords(p));
        let f00 = self.value(i, j);
        let f11 = self.value(i + 1, j + 1);
        if lower {
            let f10 = self.value(i + 1, j);
            f00 + fu * (f10 - f00) + fv * (f11 - f10)
        } else {
            let f01 = self.value(i, j + 1);
            f00 + fv * (f01 - f00) + fu * (f11 - f01)
        }
    }

    /// The gradient at a point of the plate, in mm per mm: that of the triangle holding it.
    pub fn gradient(&self, p: Point) -> [f64; 2] {
        if self.values.is_empty() {
            return [0.0, 0.0];
        }
        let (i, j, lower, _, _) = Relief::locate(self.grid.coords(p));
        self.triangle_gradient(i, j, lower)
    }

    fn triangle_gradient(&self, i: i64, j: i64, lower: bool) -> [f64; 2] {
        let s = self.grid.cell;
        let (f00, f11) = (self.value(i, j), self.value(i + 1, j + 1));
        if lower {
            let f10 = self.value(i + 1, j);
            [(f10 - f00) / s, (f11 - f10) / s]
        } else {
            let f01 = self.value(i, j + 1);
            [(f11 - f01) / s, (f01 - f00) / s]
        }
    }

    /// The lines along which the relief may bend: those through its block widened by a cell;
    /// none when it is zero.
    pub fn lines(&self) -> Option<Lines> {
        if self.is_zero() {
            return None;
        }
        let [[i0, j0], [i1, j1]] = self.block()?;
        let (lo, hi) = (|v: i64| v.saturating_sub(1), |v: i64| v.saturating_add(1));
        Some(Lines {
            levels: [
                [lo(i0), hi(i1)],
                [lo(j0), hi(j1)],
                [lo(i0.saturating_sub(j1)), hi(i1.saturating_sub(j0))],
            ],
        })
    }

    /// The fractions of the way from `a` to `b` where the relief may bend, in order, strictly
    /// between the ends: between them it is linear.
    pub fn breaks(&self, a: Point, b: Point) -> Vec<f64> {
        let Some(lines) = self.lines() else { return Vec::new() };
        let (ca, cb) = (self.grid.coords(a), self.grid.coords(b));
        // Past the box of the `u` and `v` lines the relief is zero: only the part inside bends.
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        for ((&from, &to), range) in ca.iter().zip(&cb).zip(&lines.levels) {
            let (lo, hi) = (range[0] as f64, range[1] as f64);
            if from == to {
                if from < lo || from > hi {
                    return Vec::new();
                }
                continue;
            }
            let (s, e) = ((lo - from) / (to - from), (hi - from) / (to - from));
            t0 = t0.max(s.min(e));
            t1 = t1.min(s.max(e));
        }
        let mut ts: Vec<f64> = lines
            .crossings(ca, cb)
            .into_iter()
            .map(|c| c.t)
            .filter(|&t| t >= t0 && t <= t1 && t > 0.0 && t < 1.0)
            .collect();
        ts.dedup();
        ts
    }

    /// The lowest and highest values over the plate, zero included.
    pub fn range(&self) -> [f64; 2] {
        self.values.iter().fold([0.0f64, 0.0f64], |[lo, hi], &v| [lo.min(v), hi.max(v)])
    }

    /// The highest value along the segment from `a` to `b`.
    pub fn max_along(&self, a: Point, b: Point) -> f64 {
        let along = |t: f64| self.at([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
        self.breaks(a, b).into_iter().map(along).fold(self.at(a).max(self.at(b)), f64::max)
    }

    /// The lowest and highest values within `radius` of `p`.
    pub fn range_within(&self, p: Point, radius: f64) -> [f64; 2] {
        self.range_over([p[0] - radius, p[1] - radius], [p[0] + radius, p[1] + radius])
    }

    /// The lowest and highest values over the box from `lo` to `hi`, exactly: a linear function
    /// on each triangle, the relief reaches them at the box's corners, where its edges cross the
    /// grid's lines, or at the grid's vertices inside it.
    pub fn range_over(&self, lo: Point, hi: Point) -> [f64; 2] {
        let Some([[i0, j0], [i1, j1]]) = self.block().filter(|_| !self.is_zero()) else { return [0.0, 0.0] };
        let (a, b) = (self.grid.coords(lo), self.grid.coords(hi));
        if !(a.iter().chain(&b).all(|v| v.is_finite())) || (b[0] - a[0]) * (b[1] - a[1]) > MAX_VALUES as f64 {
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
        // Inside, past the block widened by a cell the relief is zero: its vertices there count
        // only through the edges' crossings.
        let (u0, u1) = ((a[0].ceil() as i64).max(i0 - 1), (b[0].floor() as i64).min(i1 + 1));
        let (v0, v1) = ((a[1].ceil() as i64).max(j0 - 1), (b[1].floor() as i64).min(j1 + 1));
        for j in v0..=v1 {
            for i in u0..=u1 {
                take(self.value(i, j));
            }
        }
        range
    }

    /// The steepest slope of the relief, in mm per mm, over every triangle it bends on.
    pub fn steepest(&self) -> f64 {
        let Some([[i0, j0], [i1, j1]]) = self.block().filter(|_| !self.is_zero()) else { return 0.0 };
        let mut most = 0.0f64;
        for j in j0.saturating_sub(1)..=j1 {
            for i in i0.saturating_sub(1)..=i1 {
                for lower in [true, false] {
                    let g = self.triangle_gradient(i, j, lower);
                    most = most.max(g[0].hypot(g[1]));
                }
            }
        }
        most
    }

    /// The block's first and last vertices, when it holds any.
    fn block(&self) -> Option<[[i64; 2]; 2]> {
        let last = |start: i64, size: usize| start.checked_add(i64::try_from(size).ok()? - 1);
        (self.size[0] > 0 && self.size[1] > 0).then_some(())?;
        Some([self.start, [last(self.start[0], self.size[0])?, last(self.start[1], self.size[1])?]])
    }

    /// The relief `f(a, b)` of two reliefs on one grid, vertex by vertex, where `f(0, 0) = 0`.
    pub fn combine(a: &Relief, b: &Relief, f: impl Fn(f64, f64) -> f64) -> Result<Relief, GeomError> {
        if a.grid != b.grid {
            return Err(GeomError::Engine("reliefs on different grids".into()));
        }
        let [lo, hi] = match (a.block(), b.block()) {
            (Some([la, ha]), Some([lb, hb])) => {
                [[la[0].min(lb[0]), la[1].min(lb[1])], [ha[0].max(hb[0]), ha[1].max(hb[1])]]
            }
            (Some(x), None) | (None, Some(x)) => x,
            (None, None) => return Ok(Relief::zero(a.grid)),
        };
        let side = |lo: i64, hi: i64| usize::try_from(hi - lo + 1).unwrap_or(0);
        let size = [side(lo[0], hi[0]), side(lo[1], hi[1])];
        if size[0].saturating_mul(size[1]) > MAX_VALUES {
            return Err(GeomError::Engine(format!("a relief of {size:?} vertices is too large")));
        }
        Ok(Relief::from_fn(a.grid, lo, size, |i, j| f(a.value(i, j), b.value(i, j))))
    }
}
