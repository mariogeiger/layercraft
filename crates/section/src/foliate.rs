//! The leaves the layers lie on: flat where nothing asks otherwise, bent where a gently sloped
//! top surface of the solid can become one of them, so that it is printed without steps.
//!
//! Slab `k` lies between the leaves `k − 1` and `k`; leaf `k` lies at `z_k + d_k(x, y)`, `z_k`
//! the top of the flat layer and `d_k` a field on the grid; the bed and the first leaf stay flat.
//! A patch (`patches`) pins `d` on one leaf `K`, the flat layer nearest the middle of its heights:
//! `d_K = S − z_K` at its pins, and the same on the `parallel_layers − 1` leaves below, which keep
//! its shape. Every other value obeys difference constraints: each slab between `min_layer_height`
//! and `max_layer_height` thick (`d_k − d_{k−1}` within `t_min − h` and `t_max − h`), each leaf
//! leaning no more than `tan(max_slope)` between neighbouring grid vertices, and rising no more
//! than `max_rise` anywhere, what the nozzle reaches below the rest of the head. They make a
//! quasi-distance `δ` between (leaf, vertex) pairs, and the fields obeying them a lattice, closed
//! under max and min: between the least, `Lo(x) = max_p (A_p − δ(x, p))`, and the greatest,
//! `U(x) = min_p (A_p + δ(p, x))`, over pins `p` of value `A_p`, each leaf takes the value nearest
//! flat, `d = max(Lo, min(0, U))`. It meets every pin and keeps every bound, bending no more than
//! the pins demand. Pins are accepted one by one, the largest patch first and in each the pins
//! nearest the leaf's plane first, each only when it keeps the set consistent: `A_p − A_q` within
//! `δ(q, p)` and `A_q − A_p` within `δ(p, q)`, along straight lines, never longer than the grid's
//! paths, so the envelopes `U` and `Lo`, taken along the grid by Dijkstra's algorithm, meet the
//! pins. A patch rising more than `max_rise` is followed in bands, each on a leaf of its own.

use crate::SectionError;
use crate::patches::{Pin, patches};
use layercraft_geom::{Grid, MAX_VALUES, Point, Relief};
use layercraft_mesh::Mesh;
use serde::{Deserialize, Serialize};
use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap};

/// How layers bend to follow the solid's gently sloped top surfaces.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Foliation {
    /// The steepest top surface a layer follows, in degrees, and so the steepest a layer leans;
    /// 0 keeps every layer flat.
    pub max_slope: f64,
    /// The thinnest and thickest a slab may be, in mm.
    pub min_layer_height: f64,
    pub max_layer_height: f64,
    /// The most a layer rises anywhere over its lowest point, in mm.
    pub max_rise: f64,
    /// The layers that keep a followed surface's shape, its own included.
    pub parallel_layers: usize,
    /// The side of the grid's squares, in mm.
    pub cell: f64,
}

/// Fractions of a cell by which the grid is set off the plate's origin, irrational so that its
/// lines pass through no vertex of a mesh drawn on round numbers. The grid depends on nothing
/// else: the bent layers of every print with one cell lie on it, and add up on one plate.
const GENERIC: [f64; 2] = [std::f64::consts::SQRT_2 - 1.0, 0.732_050_807_568_877_2];
/// Room for rounding when pins are compared, in mm.
const SLACK: f64 = 1e-9;
/// The most bands one patch is followed in.
const MAX_BANDS: usize = 8;

/// The quasi-distance the bounds make between (leaf, point) pairs.
struct Metric {
    /// How much more `d` may grow per leaf up, and shrink per leaf down.
    up: f64,
    down: f64,
    slope: f64,
    rise: f64,
}

impl Metric {
    fn vert(&self, from: usize, to: usize) -> f64 {
        if to >= from { (to - from) as f64 * self.up } else { (from - to) as f64 * self.down }
    }

    fn dist(&self, from: (usize, Point), to: (usize, Point)) -> f64 {
        let r = (from.1[0] - to.1[0]).hypot(from.1[1] - to.1[1]);
        self.vert(from.0, to.0) + (self.slope * r).min(self.rise)
    }
}

/// An accepted pin: the leaf, the vertex and its point, and the value of `d` there.
#[derive(Debug, Clone, Copy)]
struct Held {
    leaf: usize,
    vertex: [i64; 2],
    at: Point,
    value: f64,
}

/// The pins accepted so far, filed by place for the consistency test.
struct Accepted<'a> {
    metric: &'a Metric,
    /// Pins farther apart than this, or `leaves` leaves apart, cannot disagree.
    bucket: f64,
    leaves: usize,
    cells: HashMap<[i64; 2], Vec<Held>>,
    all: Vec<Held>,
}

impl Accepted<'_> {
    fn cell(&self, at: Point) -> [i64; 2] {
        [(at[0] / self.bucket).floor() as i64, (at[1] / self.bucket).floor() as i64]
    }

    /// Whether `d = value` at `at` on `leaf` agrees with the flat first leaf and every pin held.
    fn fits(&self, leaf: usize, at: Point, value: f64) -> bool {
        let m = self.metric;
        if value > m.vert(0, leaf) + SLACK || -value > m.vert(leaf, 0) + SLACK {
            return false;
        }
        let [ci, cj] = self.cell(at);
        for dj in -1..=1 {
            for di in -1..=1 {
                for q in self.cells.get(&[ci + di, cj + dj]).into_iter().flatten() {
                    if q.leaf.abs_diff(leaf) > self.leaves {
                        continue;
                    }
                    if value - q.value > m.dist((q.leaf, q.at), (leaf, at)) + SLACK
                        || q.value - value > m.dist((leaf, at), (q.leaf, q.at)) + SLACK
                    {
                        return false;
                    }
                }
            }
        }
        true
    }

    fn hold(&mut self, held: Held) {
        let cell = self.cell(held.at);
        self.cells.entry(cell).or_default().push(held);
        self.all.push(held);
    }
}

fn check(p: &Foliation, layer_height: f64) -> Result<(), SectionError> {
    let bad = |what: &str| Err(SectionError::BadLayering(format!("foliation: {what}")));
    if !(p.max_slope.is_finite() && (0.0..60.0).contains(&p.max_slope)) {
        return bad("the steepest slope must be 0 to 60 degrees");
    }
    if !(p.min_layer_height.is_finite() && p.min_layer_height > 0.0 && p.min_layer_height <= layer_height) {
        return bad("the thinnest slab must be above 0 and at most the layer height");
    }
    if !(p.max_layer_height.is_finite() && p.max_layer_height >= layer_height && p.max_layer_height <= 2.0) {
        return bad("the thickest slab must be at least the layer height and at most 2 mm");
    }
    if !(p.max_rise.is_finite() && (0.0..=100.0).contains(&p.max_rise)) {
        return bad("the rise must be 0 to 100 mm");
    }
    if !(p.cell.is_finite() && (0.05..=5.0).contains(&p.cell)) || p.parallel_layers > 50 {
        return bad("the cell must be 0.05 to 5 mm, and parallel layers at most 50");
    }
    Ok(())
}

/// The densest band of `width` among sorted heights: its lowest and highest heights.
fn densest(heights: &[f64], width: f64) -> Option<[f64; 2]> {
    let mut best: Option<(usize, [f64; 2])> = None;
    let mut j = 0;
    for (i, &lo) in heights.iter().enumerate() {
        while heights.get(j + 1).is_some_and(|&h| h <= lo + width) {
            j += 1;
        }
        j = j.max(i);
        let hi = heights.get(j).copied().unwrap_or(lo);
        if best.is_none_or(|(n, _)| j + 1 - i > n) {
            best = Some((j + 1 - i, [lo, hi]));
        }
    }
    best.map(|(_, band)| band)
}

/// The leaf, past the first, whose plane lies nearest `z`.
fn nearest_leaf(tops: &[f64], z: f64) -> Option<usize> {
    (1..tops.len()).min_by(|&a, &b| {
        let d = |k: usize| tops.get(k).map_or(f64::INFINITY, |t| (t - z).abs());
        d(a).total_cmp(&d(b))
    })
}

/// The heap's entries, ordered by value.
#[derive(PartialEq)]
struct Entry(f64, usize);
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Entry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0).then(self.1.cmp(&other.1))
    }
}

/// `min_p (v_p + slope · path(p, x))` at every vertex of a `width × height` window, the paths
/// along the grid's sides and diagonals.
fn cones(width: usize, height: usize, sources: &[(usize, f64)], step: f64) -> Vec<f64> {
    let mut best = vec![f64::INFINITY; width * height];
    let mut heap = BinaryHeap::new();
    for &(at, v) in sources {
        if let Some(slot) = best.get_mut(at)
            && v < *slot
        {
            *slot = v;
            heap.push(Reverse(Entry(v, at)));
        }
    }
    let diagonal = step * std::f64::consts::SQRT_2;
    while let Some(Reverse(Entry(v, at))) = heap.pop() {
        if best.get(at).is_none_or(|&b| v > b) {
            continue;
        }
        let (i, j) = ((at % width) as i64, (at / width) as i64);
        for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)] {
            let (ni, nj) = (i + di, j + dj);
            if ni < 0 || nj < 0 || ni >= width as i64 || nj >= height as i64 {
                continue;
            }
            let next = (nj as usize) * width + ni as usize;
            let w = v + if di != 0 && dj != 0 { diagonal } else { step };
            if let Some(slot) = best.get_mut(next)
                && w < *slot
            {
                *slot = w;
                heap.push(Reverse(Entry(w, next)));
            }
        }
    }
    best
}

/// The pins held on one leaf, with the envelopes they cast over a window of vertices.
struct Group {
    leaf: usize,
    start: [i64; 2],
    size: [usize; 2],
    lowest: f64,
    highest: f64,
    upper: Vec<f64>,
    lower: Vec<f64>,
}

fn group(leaf: usize, pins: &[Held], slope_per_cell: f64) -> Option<Group> {
    let (mut lo, mut hi) = ([i64::MAX; 2], [i64::MIN; 2]);
    let (mut lowest, mut highest) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in pins {
        lo = [lo[0].min(p.vertex[0]), lo[1].min(p.vertex[1])];
        hi = [hi[0].max(p.vertex[0]), hi[1].max(p.vertex[1])];
        lowest = lowest.min(p.value);
        highest = highest.max(p.value);
    }
    // Past `|A| / slope` from every pin, the cones cross zero and no longer matter.
    let reach = (lowest.abs().max(highest.abs()) / slope_per_cell).ceil() as i64 + 1;
    let start = [lo[0] - reach, lo[1] - reach];
    let size = [(hi[0] - lo[0] + 2 * reach + 1) as usize, (hi[1] - lo[1] + 2 * reach + 1) as usize];
    if pins.is_empty() || size[0].saturating_mul(size[1]) > MAX_VALUES {
        return None;
    }
    let index = |v: [i64; 2]| (v[1] - start[1]) as usize * size[0] + (v[0] - start[0]) as usize;
    let up: Vec<(usize, f64)> = pins.iter().map(|p| (index(p.vertex), p.value)).collect();
    let down: Vec<(usize, f64)> = pins.iter().map(|p| (index(p.vertex), -p.value)).collect();
    let upper = cones(size[0], size[1], &up, slope_per_cell);
    let lower = cones(size[0], size[1], &down, slope_per_cell).into_iter().map(|v| -v).collect();
    Some(Group { leaf, start, size, lowest, highest, upper, lower })
}

/// Each leaf's displacement from its plane, for slabs with tops `tops` and every slab but the
/// first `layer_height` high, on a grid over the mesh; all zero when `max_slope` is.
pub fn foliate(
    mesh: &Mesh,
    tops: &[f64],
    layer_height: f64,
    p: &Foliation,
) -> Result<Vec<Relief>, SectionError> {
    check(p, layer_height)?;
    let s = p.cell;
    let grid = Grid { origin: [s * GENERIC[0], s * GENERIC[1]], cell: s };
    let flat = vec![Relief::zero(grid); tops.len()];
    let slope = p.max_slope.to_radians().tan();
    if slope < 1e-3 {
        return Ok(flat);
    }
    let metric = Metric {
        up: p.max_layer_height - layer_height,
        down: layer_height - p.min_layer_height,
        slope,
        rise: p.max_rise,
    };
    // Bands this wide put every pin within half the rise of its leaf's plane, the plane half a
    // layer from the band's middle at most, the band's edges half a layer past it at most: two
    // pins then never differ by more than the rise, and differ by more than their distance's
    // share of it only within `bucket` of each other.
    let band = (p.max_rise - 2.0 * layer_height).max(0.0);
    let bound = band / 2.0 + layer_height;
    let least = metric.up.min(metric.down);
    let mut accepted = Accepted {
        metric: &metric,
        bucket: (2.0 * bound / slope).max(s),
        leaves: if least > 1e-6 { (2.0 * bound / least).ceil() as usize } else { usize::MAX },
        cells: HashMap::new(),
        all: Vec::new(),
    };
    let parallel = p.parallel_layers.max(1);
    for patch in patches(mesh, &grid, p.max_slope.to_radians()) {
        let mut remaining: Vec<Pin> = patch.pins;
        for _ in 0..MAX_BANDS {
            let mut heights: Vec<f64> = remaining.iter().filter(|q| q.exact).map(|q| q.height).collect();
            heights.sort_by(f64::total_cmp);
            let Some([low, high]) = densest(&heights, band) else { break };
            let Some(leaf) = nearest_leaf(tops, (low + high) / 2.0) else { break };
            let z = tops.get(leaf).copied().unwrap_or(0.0);
            let (mut inside, mut rest): (Vec<Pin>, Vec<Pin>) = remaining
                .into_iter()
                .partition(|q| q.height >= low - layer_height && q.height <= high + layer_height);
            // Nearest the plane first; under the patch before beside it.
            inside.sort_by(|a, b| {
                (a.height - z).abs().total_cmp(&(b.height - z).abs()).then(b.exact.cmp(&a.exact))
            });
            let mut took = false;
            for q in inside {
                let (at, value) = (grid.vertex(q.vertex[0], q.vertex[1]), q.height - z);
                if value.abs() > bound {
                    if q.exact {
                        rest.push(q);
                    }
                    continue;
                }
                let copies = (0..parallel).filter_map(|j| leaf.checked_sub(j)).filter(|&k| k >= 1);
                if copies.clone().all(|k| accepted.fits(k, at, value)) {
                    for k in copies {
                        accepted.hold(Held { leaf: k, vertex: q.vertex, at, value });
                    }
                    took = true;
                } else if q.exact && (q.height < low || q.height > high) {
                    rest.push(q);
                }
            }
            remaining = rest;
            if !took || remaining.is_empty() {
                break;
            }
        }
    }
    leaves(&grid, &metric, tops.len(), &accepted.all)
}

/// The leaves the held pins make: `d = max(Lo, min(0, U))` on each.
fn leaves(grid: &Grid, metric: &Metric, count: usize, held: &[Held]) -> Result<Vec<Relief>, SectionError> {
    let mut by_leaf: BTreeMap<usize, Vec<Held>> = BTreeMap::new();
    for h in held {
        by_leaf.entry(h.leaf).or_default().push(*h);
    }
    let step = metric.slope * grid.cell;
    let groups: Vec<Group> = by_leaf.iter().filter_map(|(&leaf, pins)| group(leaf, pins, step)).collect();
    let mut out = Vec::with_capacity(count);
    for k in 0..count {
        // The groups whose envelopes reach this leaf on either side of zero.
        let near: Vec<&Group> = groups
            .iter()
            .filter(|g| {
                k >= 1
                    && (g.lowest + metric.vert(g.leaf, k) < 0.0 || g.highest - metric.vert(k, g.leaf) > 0.0)
            })
            .collect();
        let Some(first) = near.first() else {
            out.push(Relief::zero(*grid));
            continue;
        };
        let (mut lo, mut hi) = (first.start, first.start);
        for g in &near {
            lo = [lo[0].min(g.start[0]), lo[1].min(g.start[1])];
            hi = [hi[0].max(g.start[0] + g.size[0] as i64), hi[1].max(g.start[1] + g.size[1] as i64)];
        }
        let size = [(hi[0] - lo[0]) as usize, (hi[1] - lo[1]) as usize];
        if size[0].saturating_mul(size[1]) > MAX_VALUES {
            return Err(SectionError::BadLayering(format!("a leaf of {size:?} vertices is too large")));
        }
        let mut upper = vec![metric.vert(0, k); size[0] * size[1]];
        let mut lower = vec![-metric.vert(k, 0); size[0] * size[1]];
        for g in &near {
            let (rise, fall) = (metric.vert(g.leaf, k), metric.vert(k, g.leaf));
            for gj in 0..g.size[1] {
                for gi in 0..g.size[0] {
                    let (i, j) = ((g.start[0] - lo[0]) as usize + gi, (g.start[1] - lo[1]) as usize + gj);
                    let (src, dst) = (gj * g.size[0] + gi, j * size[0] + i);
                    if let (Some(u), Some(slot)) = (g.upper.get(src), upper.get_mut(dst)) {
                        *slot = slot.min(u + rise);
                    }
                    if let (Some(l), Some(slot)) = (g.lower.get(src), lower.get_mut(dst)) {
                        *slot = slot.max(l - fall);
                    }
                }
            }
        }
        out.push(Relief::from_fn(*grid, lo, size, |i, j| {
            let at = (j - lo[1]) as usize * size[0] + (i - lo[0]) as usize;
            let (u, l) = (upper.get(at).copied().unwrap_or(0.0), lower.get(at).copied().unwrap_or(0.0));
            l.max(u.min(0.0))
        }));
    }
    Ok(out)
}
