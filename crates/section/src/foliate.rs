//! The leaves the layers lie on: flat where nothing asks otherwise, bent where a gently sloped
//! top surface of the solid can become one of them, so that it is printed without steps.
//!
//! Slab `k` lies between the leaves `k − 1` and `k`; leaf `k` lies at `z_k + d_k(x, y)`, `z_k`
//! the top of the flat layer and `d_k` a field on the grid; the bed and the first leaf stay flat.
//! A patch (`patches`) pins `d` on one leaf `K`, the flat layer nearest the middle of its heights,
//! and on the `parallel_layers − 1` leaves below, which keep its shape, each slab as thick as the
//! face lets parallel slabs be on the patch's slope. Every other value obeys difference
//! constraints: each slab `min_layer_height` to `max_layer_height` thick, each leaf leaning no
//! more than `tan(max_slope)` between neighbouring vertices and rising no more than `max_rise`,
//! and no bead below above the nozzle's face, `d_{k−1}(y) ≤ d_k(x) + h` for `|y − x| ≤ face_reach`
//! (`face`). They make a quasi-distance `δ` between (leaf, vertex) pairs, and the fields obeying
//! them a lattice, closed under max and min: between the least, `Lo(x) = max_p (A_p − δ(x, p))`,
//! and the greatest, `U(x) = min_p (A_p + δ(p, x))`, over pins `p` of value `A_p`, each leaf takes
//! the value nearest flat, `d = max(Lo, min(0, U))`, meeting every pin and bending no more than
//! they demand. Pins are accepted largest patch first, nearest the plane first, each only when it
//! agrees with those held, `A_p − A_q ≤ δ(q, p)` by a bound never above the grid's paths, so the
//! envelopes, cones along the grid by Dijkstra's algorithm closed across leaves, meet the pins.
//! A patch rising more than `max_rise` is followed in bands, each on a leaf of its own.

use crate::SectionError;
use crate::face::{Laps, laps, lean};
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
    /// How far from where a leaf is printed the leaf below lies no higher, in mm: the radius of
    /// the nozzle's flat face, and half the widest bead, whose flat top stands across its width
    /// as high as its middle.
    pub face_reach: f64,
    /// How deep the face may iron the beads of the leaf it prints, in mm: no leaf leans more than
    /// lets one rise by that over the face's reach (`face::lean`).
    pub ironing: f64,
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
    /// The edges to the leaf below, under the nozzle's face.
    laps: Laps,
    /// The most leaves a path crosses.
    leaves: usize,
    /// Whether a path may gain by turning: crossing a leaf more each way, for `up + down`, lets
    /// `d` fall sideways at most `slope · R − t` further, `fall` being convex.
    turns: bool,
}

impl Metric {
    fn vert(&self, from: usize, to: usize) -> f64 {
        if to >= from { (to - from) as f64 * self.up } else { (from - to) as f64 * self.down }
    }

    /// The most `d` may change from `from` to `to`, at least: a path crossing `m` leaves down and
    /// `m − n` up costs `down` and `up` each and, sideways, `m · fall(r / m)` across the leaves,
    /// `fall` convex and no steeper than the slope, or the slope along a leaf, or the rise.
    fn dist(&self, from: (usize, Point), to: (usize, Point)) -> f64 {
        let r = (from.1[0] - to.1[0]).hypot(from.1[1] - to.1[1]);
        let (falls, rises) = (from.0.saturating_sub(to.0), to.0.saturating_sub(from.0));
        let sideways = |m: usize| {
            let along = if m == 0 { self.slope * r } else { m as f64 * self.laps.fall(r / m as f64) };
            along.min(self.rise)
        };
        if !self.turns {
            return falls as f64 * self.down + rises as f64 * self.up + sideways(falls);
        }
        let mut best = f64::INFINITY;
        for extra in 0..=self.leaves {
            let crossing = (falls + extra) as f64 * self.down + (rises + extra) as f64 * self.up;
            if crossing >= best {
                break;
            }
            best = best.min(crossing + sideways(falls + extra));
        }
        best
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
        // Sideways `d` changes no slower than this: pins that differ by less over their
        // distance agree, whatever leaves they lie on.
        let rate = m.laps.rate();
        for dj in -1..=1 {
            for di in -1..=1 {
                for q in self.cells.get(&[ci + di, cj + dj]).into_iter().flatten() {
                    let (dx, dy, dv) = (q.at[0] - at[0], q.at[1] - at[1], q.value - value);
                    if q.leaf.abs_diff(leaf) > self.leaves || dv * dv <= rate * rate * (dx * dx + dy * dy) {
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
    if !(p.face_reach.is_finite() && (0.0..=10.0).contains(&p.face_reach)) {
        return bad("the nozzle's face must reach 0 to 10 mm");
    }
    if !(p.ironing.is_finite() && p.ironing >= 0.0) {
        return bad("the ironing must be finite and not negative");
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
    let slope = p.max_slope.to_radians().tan().min(p.ironing / lean(s, p.face_reach));
    if slope < 1e-3 {
        return Ok(flat);
    }
    let (up, down) = (p.max_layer_height - layer_height, layer_height - p.min_layer_height);
    let laps = laps(s, p.face_reach, slope, down, p.min_layer_height);
    let turns = up + down < slope * laps.reach - p.min_layer_height;
    let metric = Metric { up, down, slope, rise: p.max_rise, laps, leaves: tops.len(), turns };
    // Bands this wide put every pin within half the rise of its leaf's plane, the plane half a
    // layer from the band's middle at most, the band's edges half a layer past it at most: two
    // pins then never differ by more than the rise, and differ by more than their distance's
    // share of it only within `bucket` of each other.
    let band = (p.max_rise - 2.0 * layer_height).max(0.0);
    let bound = band / 2.0 + layer_height;
    let least = metric.up.min(metric.down);
    let leaves_apart = if least > 1e-6 { (2.0 * bound / least).ceil() as usize } else { tops.len() };
    let mut accepted = Accepted {
        metric: &metric,
        // Sideways, `d` changes no slower than this, whichever leaves it crosses.
        bucket: (2.0 * bound / metric.laps.rate()).max(s),
        leaves: leaves_apart,
        cells: HashMap::new(),
        all: Vec::new(),
    };
    let parallel = p.parallel_layers.max(1);
    for patch in patches(mesh, &grid, slope.atan()) {
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
                // Each copy a slab lower, as thick as the face lets parallel slabs be on the pin's
                // slope; none where no slab can be that thick.
                let thicker = (q.slope * metric.laps.reach - layer_height).max(0.0);
                let depth = if thicker <= metric.up + SLACK { parallel } else { 1 };
                let copies = (0..depth)
                    .filter_map(|j| Some((leaf.checked_sub(j)?, value - j as f64 * thicker)))
                    .filter(|&(k, _)| k >= 1);
                // Every value within the bound keeps every leaf within the rise.
                if copies.clone().all(|(k, v)| v.abs() <= bound && accepted.fits(k, at, v)) {
                    for (k, value) in copies {
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

/// The vertices the leaves may bend over: around every pin, as far as its envelopes stay on the
/// side of zero they bend the leaves to.
struct Window {
    start: [i64; 2],
    size: [usize; 2],
}

impl Window {
    fn around(held: &[Held], metric: &Metric, cell: f64) -> Option<Window> {
        let first = held.first()?;
        let (mut lo, mut hi) = (first.vertex, first.vertex);
        let mut most = 0.0f64;
        for p in held {
            lo = [lo[0].min(p.vertex[0]), lo[1].min(p.vertex[1])];
            hi = [hi[0].max(p.vertex[0]), hi[1].max(p.vertex[1])];
            most = most.max(p.value.abs());
        }
        // Past `|A|` over the least rate `d` changes sideways at, the envelopes cross zero.
        let cells = (most / metric.laps.rate() / cell).ceil();
        if cells.is_nan() || cells >= MAX_VALUES as f64 {
            return None;
        }
        let reach = cells as i64 + 1;
        let start = [lo[0].checked_sub(reach)?, lo[1].checked_sub(reach)?];
        let side = |a: i64, b: i64| usize::try_from(b.checked_sub(a)?.checked_add(2 * reach + 1)?).ok();
        let size = [side(lo[0], hi[0])?, side(lo[1], hi[1])?];
        (size[0].saturating_mul(size[1]) <= MAX_VALUES).then_some(Window { start, size })
    }

    fn index(&self, v: [i64; 2]) -> usize {
        (v[1] - self.start[1]) as usize * self.size[0] + (v[0] - self.start[0]) as usize
    }
}

/// An envelope on the window where it bends a leaf: below zero, the rest past caring; `None`
/// when nowhere.
fn bending(values: Vec<f64>) -> Option<Vec<f64>> {
    let mut any = false;
    let values: Vec<f64> = values
        .into_iter()
        .map(|v| {
            any |= v < 0.0;
            if v < 0.0 { v } else { f64::INFINITY }
        })
        .collect();
    any.then_some(values)
}

/// Lowers an envelope to another where that one is lower; whether anywhere.
fn lower(slot: &mut Option<Vec<f64>>, other: Option<Vec<f64>>) -> bool {
    let Some(other) = other else { return false };
    match slot {
        Some(values) => {
            let mut changed = false;
            for (x, y) in values.iter_mut().zip(other) {
                if y < *x {
                    *x = y;
                    changed = true;
                }
            }
            changed
        }
        None => {
            *slot = Some(other);
            true
        }
    }
}

/// One leaf on from an envelope, by `shift` alone, where the slabs thicken.
fn shifted(f: &Option<Vec<f64>>, shift: f64) -> Option<Vec<f64>> {
    f.as_ref().and_then(|f| bending(f.iter().map(|v| v + shift).collect()))
}

/// One leaf on from an envelope where the slabs thin or the face laps: by each lap's weight to
/// its offset, then no steeper than the slope. The laps are symmetric, their weights depending
/// on the offset's length alone, so one leaf on is the same either way.
fn lapped(f: &Option<Vec<f64>>, w: &Window, metric: &Metric, cell: f64) -> Option<Vec<f64>> {
    let f = f.as_ref()?;
    let [width, height] = w.size;
    let mut next = vec![f64::INFINITY; f.len()];
    for j in 0..height as i64 {
        for i in 0..width as i64 {
            let mut best = f64::INFINITY;
            for &([di, dj], weight) in &metric.laps.edges {
                let (ni, nj) = (i - di, j - dj);
                if ni >= 0 && nj >= 0 && ni < width as i64 && nj < height as i64 {
                    let v = f.get(nj as usize * width + ni as usize).copied().unwrap_or(f64::INFINITY);
                    best = best.min(v + weight);
                }
            }
            if let Some(slot) = next.get_mut(j as usize * width + i as usize) {
                *slot = best;
            }
        }
    }
    let sources: Vec<(usize, f64)> =
        next.iter().enumerate().filter(|(_, v)| v.is_finite() && **v < 0.0).map(|(k, &v)| (k, v)).collect();
    bending(cones(width, height, &sources, metric.slope * cell))
}

/// Closes envelopes under the bounds between leaves, each leaf's own to begin with: from a leaf
/// to the next by `up`, to the one before by the laps, pass after pass until neither lowers
/// anything, a leaf casting each way only once lowered since it last did. A path that turns
/// costs a slab's thickening and thinning, which in practice the slope beats: the second pass
/// up then lowers nothing.
fn close(env: &mut [Option<Vec<f64>>], w: &Window, metric: &Metric, cell: f64) -> Result<(), SectionError> {
    let n = env.len();
    // Whether each leaf is still to cast up, and down.
    let mut due = vec![(true, true); n];
    let mark = |due: &mut Vec<(bool, bool)>, k: usize| {
        if let Some(d) = due.get_mut(k) {
            *d = (true, true);
        }
    };
    for _ in 0..=n {
        for k in 1..n {
            if due.get(k - 1).is_some_and(|d| d.0) {
                let next = env.get(k - 1).and_then(|f| shifted(f, metric.up));
                if env.get_mut(k).is_some_and(|slot| lower(slot, next)) {
                    mark(&mut due, k);
                }
            }
        }
        due.iter_mut().for_each(|d| d.0 = false);
        for k in (0..n.saturating_sub(1)).rev() {
            if due.get(k + 1).is_some_and(|d| d.1) {
                let next = env.get(k + 1).and_then(|f| lapped(f, w, metric, cell));
                if env.get_mut(k).is_some_and(|slot| lower(slot, next)) {
                    mark(&mut due, k);
                }
            }
        }
        due.iter_mut().for_each(|d| d.1 = false);
        if !due.iter().any(|d| d.0) {
            return Ok(());
        }
    }
    Err(SectionError::BadLayering("foliation: the bounds between leaves do not settle".into()))
}

/// The leaves the held pins make: `d = max(Lo, min(0, U))` on each. The greatest field `U`
/// below the pins is their cones closed under the bounds between leaves; the least, `Lo = −N`,
/// the same of the pins negated on the leaves in reverse, every bound reversed: `N` comes down
/// the leaves where they thicken and up where they thin or the face laps.
fn leaves(grid: &Grid, metric: &Metric, count: usize, held: &[Held]) -> Result<Vec<Relief>, SectionError> {
    let Some(w) = Window::around(held, metric, grid.cell) else {
        return if held.is_empty() {
            Ok(vec![Relief::zero(*grid); count])
        } else {
            Err(SectionError::BadLayering("the leaves bend over too many vertices".into()))
        };
    };
    let mut by_leaf: BTreeMap<usize, Vec<(usize, f64)>> = BTreeMap::new();
    for h in held {
        by_leaf.entry(h.leaf).or_default().push((w.index(h.vertex), h.value));
    }
    let step = metric.slope * grid.cell;
    let own = |k: usize, sign: f64| -> Option<Vec<f64>> {
        let pins = by_leaf.get(&k)?;
        let sources: Vec<(usize, f64)> = pins.iter().map(|&(at, v)| (at, sign * v)).collect();
        bending(cones(w.size[0], w.size[1], &sources, step))
    };
    let mut upper: Vec<Option<Vec<f64>>> = (0..count).map(|k| own(k, 1.0)).collect();
    close(&mut upper, &w, metric, grid.cell)?;
    let mut lower: Vec<Option<Vec<f64>>> = (0..count).rev().map(|k| own(k, -1.0)).collect();
    close(&mut lower, &w, metric, grid.cell)?;
    lower.reverse();
    let mut out = vec![Relief::zero(*grid); count];
    for k in 1..count {
        let (u, n) = (upper.get(k).and_then(Option::as_ref), lower.get(k).and_then(Option::as_ref));
        if u.is_none() && n.is_none() {
            continue;
        }
        let at = |values: Option<&Vec<f64>>, i: usize| values.and_then(|v| v.get(i)).copied();
        let relief = Relief::from_fn(*grid, w.start, w.size, |i, j| {
            let index = (j - w.start[1]) as usize * w.size[0] + (i - w.start[0]) as usize;
            let up = at(u, index).unwrap_or(f64::INFINITY);
            let lo = -at(n, index).unwrap_or(f64::INFINITY);
            lo.max(up.min(0.0))
        });
        if let Some(slot) = out.get_mut(k) {
            *slot = relief;
        }
    }
    Ok(out)
}
