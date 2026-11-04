//! The nozzle's flat face over the leaves' triangles: which corners bind each other, and by how
//! much, so that no bead below stands above the face printing over it, nor the face irons a
//! leaf deeper than allowed.
//!
//! A bead's flat top stands as high as its middle, on its leaf, so the face asks
//! `d_{k−1}(y) ≤ d_k(x) + h` for `|y − x| ≤ ρ`, its reach, `h` the layer height. The leaves are
//! linear on the triangles: for `x` in one with corners `v_i` and weights `α_i` and `y` in one
//! with `w_j` and `β_j`, `d_{k−1}(y) − d_k(x) = Σ α_i β_j (d_{k−1}(w_j) − d_k(v_i))`, and the
//! squared distances between their corners average exactly to
//! `Σ α_i β_j |v_i − w_j|² = |x − y|² + S(x) + S(y)`, each `S` the corners' spread about the
//! point, `Σ α_i |v_i − x|²`, no more than its triangle's (`variance`). So with every pair of
//! corners that can meet within the face bound by `down + t r²/R²`, `R² = ρ² + S_v + S_w` from
//! the largest spreads at the two corners, `t` the thinnest slab and `down` how far `d` may fall
//! straight down, the sum is at most `down + t = h`. Their mean distance is at most the root of
//! that mean square, `R`, so with every such pair on one leaf bound by `(i/R)|w − v|` no leaf
//! rises more than `i` within the face's reach: the face irons no deeper than `i`.

use layercraft_geom::Point;

/// The root mean square distance between the corners of two triangles holding points at most
/// `rho` apart, at most, when their spreads are at most `v` and `w` (`variance`).
pub(crate) fn reach(rho: f64, v: f64, w: f64) -> f64 {
    (rho * rho + v.max(0.0) + w.max(0.0)).sqrt()
}

/// The most `Σ α_i |v_i − x|²` reaches over a triangle: its circumradius squared, or, when it is
/// right or obtuse and its circumcentre outside it, half its longest side squared.
pub(crate) fn variance([a, b, c]: [Point; 3]) -> f64 {
    let d2 = |p: Point, q: Point| (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
    let mut sides = [d2(b, c), d2(c, a), d2(a, b)];
    sides.sort_by(f64::total_cmp);
    let [s0, s1, s2] = sides;
    if s2 >= s0 + s1 {
        return s2 / 4.0;
    }
    // R² = a²b²c² / (16 · area²), 16 area² = 2(a²b² + b²c² + c²a²) − (a⁴ + b⁴ + c⁴).
    let sixteen_area2 = 2.0 * (s0 * s1 + s1 * s2 + s2 * s0) - (s0 * s0 + s1 * s1 + s2 * s2);
    if sixteen_area2 > 0.0 { s0 * s1 * s2 / sixteen_area2 } else { s2 / 4.0 }
}

/// Buckets per side of the square holding a corner's pairs at most: a quarter of the farthest
/// pair, so that a search looks at little more than the disc it needs.
const BUCKETS_PER_REACH: f64 = 4.0;

/// The corners of the triangles, filed in square buckets for the search of those that can meet
/// within the face.
pub(crate) struct Corners<'a> {
    points: &'a [Point],
    /// The longest edge at each corner; 0 at points of no triangle.
    longest: Vec<f64>,
    /// The largest spread of the triangles at each corner (`variance`).
    spread: Vec<f64>,
    rho: f64,
    widest: f64,
    origin: Point,
    size: f64,
    columns: usize,
    rows: usize,
    /// Bucket `k` holds `items[starts[k]..starts[k + 1]]`.
    starts: Vec<usize>,
    items: Vec<u32>,
}

impl<'a> Corners<'a> {
    pub fn new(points: &'a [Point], triangles: &[[u32; 3]], rho: f64) -> Corners<'a> {
        let (mut longest, mut spread) = (vec![0.0f64; points.len()], vec![0.0f64; points.len()]);
        let at = |i: u32| points.get(i as usize).copied().unwrap_or([0.0; 2]);
        for &[a, b, c] in triangles {
            let s = variance([at(a), at(b), at(c)]);
            for (p, q) in [(a, b), (b, c), (c, a)] {
                let l = (at(p)[0] - at(q)[0]).hypot(at(p)[1] - at(q)[1]);
                for v in [p, q] {
                    if let (Some(slot), Some(wide)) =
                        (longest.get_mut(v as usize), spread.get_mut(v as usize))
                    {
                        (*slot, *wide) = (slot.max(l), wide.max(s));
                    }
                }
            }
        }
        let widest = longest.iter().copied().fold(0.0, f64::max);
        // Only the triangles' corners: points of none bind nothing.
        let corners: Vec<u32> =
            (0..points.len() as u32).filter(|&v| longest.get(v as usize).is_some_and(|&l| l > 0.0)).collect();
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in corners.iter().map(|&v| at(v)) {
            (lo, hi) = ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]);
        }
        // No more buckets than corners, about.
        let extent = (hi[0] - lo[0]).max(0.0) * (hi[1] - lo[1]).max(0.0);
        let fine = (rho + 2.0 * widest) / BUCKETS_PER_REACH;
        let size = fine.max((extent / corners.len().max(1) as f64).sqrt()).max(1e-3);
        let side = |e: f64| if e.is_finite() && e >= 0.0 { (e / size) as usize + 1 } else { 1 };
        let (columns, rows) = (side(hi[0] - lo[0]), side(hi[1] - lo[1]));
        let mut out = Corners {
            points,
            longest,
            spread,
            rho,
            widest,
            origin: if lo[0].is_finite() { lo } else { [0.0; 2] },
            size,
            columns,
            rows,
            starts: Vec::new(),
            items: Vec::new(),
        };
        let cell = |out: &Corners, v: u32| {
            let [i, j] = out.cell(at(v));
            j * out.columns + i
        };
        let mut starts = vec![0usize; columns * rows + 1];
        for &v in &corners {
            if let Some(s) = starts.get_mut(cell(&out, v) + 1) {
                *s += 1;
            }
        }
        for k in 0..columns * rows {
            let before = starts.get(k).copied().unwrap_or(0);
            if let Some(s) = starts.get_mut(k + 1) {
                *s += before;
            }
        }
        let mut fill = starts.clone();
        let mut items = vec![0u32; corners.len()];
        for &v in &corners {
            if let Some(slot) = fill.get_mut(cell(&out, v))
                && let Some(item) = items.get_mut(*slot)
            {
                *item = v;
                *slot += 1;
            }
        }
        (out.starts, out.items) = (starts, items);
        out
    }

    /// The bucket column and row of `p`, clamped into the grid.
    fn cell(&self, p: Point) -> [usize; 2] {
        let at = |v: f64, o: f64, n: usize| {
            let k = ((v - o) / self.size).floor();
            if k.is_nan() || k < 0.0 { 0 } else { (k as usize).min(n.saturating_sub(1)) }
        };
        [at(p[0], self.origin[0], self.columns), at(p[1], self.origin[1], self.rows)]
    }

    /// The longest pair of corners that can meet within the face.
    pub fn farthest(&self) -> f64 {
        self.rho + 2.0 * self.widest
    }

    /// The face's reach as the coarsest triangles resolve it: `R` between corners of the largest
    /// spread.
    pub fn coarsest(&self) -> f64 {
        let widest = self.spread.iter().copied().fold(0.0, f64::max);
        reach(self.rho, widest, widest)
    }

    /// The face's reach as the triangles at corners `v` and `w` resolve it, `R` for their pairs.
    pub fn reach(&self, v: u32, w: u32) -> f64 {
        let s = |i: u32| self.spread.get(i as usize).copied().unwrap_or(0.0);
        reach(self.rho, s(v), s(w))
    }

    /// Each corner `w` that can meet `v` within the face, `v` itself included, with their
    /// distance: no farther apart than `ρ` and the longest edges at both.
    pub fn meeting(&self, v: u32, f: impl FnMut(u32, f64)) {
        self.meeting_within(v, f64::INFINITY, f);
    }

    /// Those of `meeting` less than `limit` apart.
    pub fn meeting_within(&self, v: u32, limit: f64, mut f: impl FnMut(u32, f64)) {
        let (Some(&p), Some(&lv)) = (self.points.get(v as usize), self.longest.get(v as usize)) else {
            return;
        };
        if lv <= 0.0 {
            return;
        }
        let radius = (self.rho + lv + self.widest).min(limit);
        let ([i0, j0], [i1, j1]) =
            (self.cell([p[0] - radius, p[1] - radius]), self.cell([p[0] + radius, p[1] + radius]));
        for j in j0..=j1 {
            let (Some(&a), Some(&b)) =
                (self.starts.get(j * self.columns + i0), self.starts.get(j * self.columns + i1 + 1))
            else {
                continue;
            };
            for &w in self.items.get(a..b).unwrap_or(&[]) {
                let (Some(&q), Some(&lw)) = (self.points.get(w as usize), self.longest.get(w as usize))
                else {
                    continue;
                };
                let d = (p[0] - q[0]).hypot(p[1] - q[1]);
                if d <= self.rho + lv + lw && d < limit {
                    f(w, d);
                }
            }
        }
    }
}

/// The bounds from a leaf to the one below under the face, and along a leaf.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Laps {
    /// How far `d` may fall straight down, in mm.
    pub down: f64,
    /// The thinnest slab, in mm.
    pub thin: f64,
    /// The face's reach as the coarsest triangles resolve it, `R`, in mm; 0 without a face.
    pub reach: f64,
    /// The longest pair of corners bound, in mm.
    pub farthest: f64,
    /// How fast `d` may change along a leaf, in mm per mm.
    pub slope: f64,
}

impl Laps {
    /// How far `d` may fall from a corner to one `l` mm off on the leaf below, for a pair whose
    /// triangles resolve the face's reach as `r`.
    pub fn weight(&self, l: f64, r: f64) -> f64 {
        if r > 0.0 { self.down + self.thin * (l / r).powi(2) } else { self.down }
    }

    /// The distance sideways past which a fall across a leaf goes on cheaper along a leaf: where
    /// a pair's `t l²/R²` grows as fast as the slope, or the longest pair.
    fn turn(&self) -> f64 {
        (self.slope * self.reach * self.reach / (2.0 * self.thin)).min(self.farthest)
    }

    /// How much further than straight down `d` falls across one leaf to a point `l` mm off, at
    /// least: along a pair of corners, `t l²/R²`, up to `turn`, and on along a leaf at the slope;
    /// convex in `l`.
    pub fn fall(&self, l: f64) -> f64 {
        if self.reach <= 0.0 {
            return self.slope * l;
        }
        let k = self.turn();
        let pair = |l: f64| self.thin * (l / self.reach).powi(2);
        if l <= k { pair(l) } else { pair(k) + self.slope * (l - k) }
    }

    /// How far sideways one leaf's crossing lets `d` fall by `drop`, at most: `fall` inverted.
    pub fn span(&self, drop: f64) -> f64 {
        let along = |drop: f64| if self.slope > 0.0 { drop / self.slope } else { f64::INFINITY };
        if self.reach <= 0.0 {
            return along(drop);
        }
        let k = self.turn();
        let near = self.fall(k);
        if drop <= near { self.reach * (drop / self.thin).sqrt() } else { k + along(drop - near) }
    }

    /// The least rate, in mm per mm, at which `d` may change sideways along any path: along a
    /// leaf at the slope, or across leaves, each crossing `down + fall(u)` for `u` sideways, least
    /// per mm at `u = R √(down/t)`, `2√(down t)/R`, unless the fall turns before.
    pub fn rate(&self) -> f64 {
        let k = self.turn();
        if self.reach <= 0.0 || k <= 0.0 {
            return self.slope;
        }
        let u = self.reach * (self.down / self.thin).sqrt();
        let across = if u <= k {
            2.0 * (self.down * self.thin).sqrt() / self.reach
        } else {
            (self.down + self.fall(k)) / k
        };
        across.min(self.slope)
    }

    /// How much thicker than a layer slabs parallel to a surface of slope `s` must be for the
    /// face: each pair of corners `l` apart, up to `farthest`, sees the leaf below rise by up to
    /// `s l` and allows `down + t l²/R²`, most short at `l = s R²/(2t)`.
    pub fn parallel(&self, s: f64) -> f64 {
        if self.reach <= 0.0 {
            return 0.0;
        }
        let l = (s * self.reach * self.reach / (2.0 * self.thin)).min(self.farthest);
        (s * l - self.thin * (l / self.reach).powi(2) - self.down).max(0.0)
    }
}
