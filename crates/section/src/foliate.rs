//! The leaves the layers lie on: flat where nothing asks otherwise, bent where a gently sloped
//! top surface of the solid can become one of them, so that it is printed without steps.
//!
//! Slab `k` lies between the leaves `k − 1` and `k`; leaf `k` lies at `z_k + d_k(x, y)`, `z_k`
//! the top of the flat layer and `d_k` linear on triangles shared by every leaf, whose edges hold
//! every facet's edges, as far past the patches as a leaf may bend (`triangulation`); the bed
//! and the first leaf stay flat. A patch pins `d` at the corners on its facets, on one leaf `K`,
//! the flat layer nearest the middle of its heights, and on the `parallel_layers − 1` leaves
//! below, which keep its shape, each slab as much thicker as the face asks of parallel slabs on
//! the facet's slope: the leaf then meets the patch exactly, creases included. Every other value
//! obeys difference constraints: each slab `min_layer_height` to `max_layer_height` thick, each
//! leaf changing no faster than `tan(max_slope)` between neighbouring corners and rising no more
//! than `max_rise`, and no bead below above the nozzle's face nor the face ironing deeper than
//! allowed (`face`). They make a quasi-distance `δ` between (leaf, corner) pairs, and the fields
//! obeying them a lattice, closed under max and min: between the least,
//! `Lo(x) = max_p (A_p − δ(x, p))`, and the greatest, `U(x) = min_p (A_p + δ(p, x))`, over pins
//! `p` of value `A_p`, each leaf takes the value nearest flat, `d = max(Lo, min(0, U))`
//! (`envelope`). Pins are accepted largest patch first, nearest the plane first, each only when
//! it agrees with those held, `A_p − A_q ≤ δ(q, p)` by a bound never above `δ`. A patch rising
//! more than `max_rise` is followed in bands, each on a leaf of its own.

use crate::SectionError;
use crate::envelope::{Bounds, Graph};
use crate::face::{Corners, Laps, reach};
use crate::patches::{Facet, patches};
use crate::triangulation::{Triangulation, hull, triangulation};
use layercraft_geom::{GRID_MM, Point};
use layercraft_mesh::Mesh;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// How layers bend to follow the solid's gently sloped top surfaces.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Foliation {
    /// The steepest top surface a layer follows, in degrees, and so the steepest a layer leans
    /// between neighbouring corners; 0 keeps every layer flat.
    pub max_slope: f64,
    /// The thinnest and thickest a slab may be, in mm.
    pub min_layer_height: f64,
    pub max_layer_height: f64,
    /// The most a layer rises anywhere over its lowest point, in mm.
    pub max_rise: f64,
    /// The layers that keep a followed surface's shape, its own included.
    pub parallel_layers: usize,
    /// How wide the triangles the leaves bend on are, about, in mm.
    pub cell: f64,
    /// How far from where a leaf is printed the leaf below lies no higher, in mm: the radius of
    /// the nozzle's flat face, and half the widest bead, whose flat top stands across its width
    /// as high as its middle.
    pub face_reach: f64,
    /// How deep the face may iron the beads of the leaf it prints, in mm: no leaf rises more
    /// than that within the face's reach.
    pub ironing: f64,
}

/// Room for rounding when pins are compared, in mm.
const SLACK: f64 = 1e-9;
/// The most bands one patch is followed in.
const MAX_BANDS: usize = 8;
/// A pin this close to its leaf's plane, in mm, asks no bend: half the lattice step.
const ON_PLANE_MM: f64 = GRID_MM / 2.0;

/// The leaves: the triangles they are linear on, and each leaf's value at their corners, `None`
/// where it is flat.
#[derive(Debug, Clone, Default)]
pub(crate) struct Leaves {
    pub triangulation: Triangulation,
    pub values: Vec<Option<Vec<f64>>>,
}

/// The quasi-distance the bounds make between (leaf, point) pairs, at least.
struct Metric {
    /// How much more `d` may grow per leaf up, and shrink per leaf down.
    up: f64,
    down: f64,
    slope: f64,
    rise: f64,
    laps: Laps,
    /// The most leaves a path crosses.
    leaves: usize,
    /// Whether a path may gain by turning: crossing a leaf more each way, for `up + down`, lets
    /// `d` fall sideways further, `fall` being convex.
    turns: bool,
}

impl Metric {
    /// How far from a pin of value at most `bound` a leaf may still be bent by it: past that,
    /// every path from the pin crossing `m` leaves costs more, `m` times `down` and the fall
    /// over a share of the distance, or along a leaf the slope.
    fn extent(&self, bound: f64) -> f64 {
        let mut far = if self.slope > 0.0 { bound / self.slope } else { f64::INFINITY };
        for m in 1..=self.leaves {
            let left = bound - m as f64 * self.down;
            if left <= 0.0 {
                break;
            }
            far = far.max(m as f64 * self.laps.span(left / m as f64));
        }
        far
    }

    fn vert(&self, from: usize, to: usize) -> f64 {
        if to >= from { (to - from) as f64 * self.up } else { (from - to) as f64 * self.down }
    }

    /// The most `d` may change from `from` to `to`, at least: a path crossing `m` leaves down and
    /// `m − n` up costs `down` and `up` each and, sideways, `m · fall(r / m)` across the leaves,
    /// `fall` convex, or the slope along a leaf, or the rise.
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

/// An accepted pin: the leaf, the corner and its point, and the value of `d` there.
#[derive(Debug, Clone, Copy)]
struct Held {
    leaf: usize,
    corner: u32,
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

/// A corner a patch asks a leaf through: its height there, and how steep its facet is.
#[derive(Debug, Clone, Copy)]
struct Pin {
    corner: u32,
    height: f64,
    slope: f64,
}

/// Each leaf's displacement from its plane, for slabs with tops `tops` and every slab but the
/// first `layer_height` high; all flat when `max_slope` is zero.
pub(crate) fn foliate(
    mesh: &Mesh,
    tops: &[f64],
    layer_height: f64,
    p: &Foliation,
) -> Result<Leaves, SectionError> {
    check(p, layer_height)?;
    let flat = Leaves { triangulation: Triangulation::default(), values: vec![None; tops.len()] };
    let steepest = p.max_slope.to_radians().tan();
    if steepest < 1e-3 {
        return Ok(flat);
    }
    let (up, down) = (p.max_layer_height - layer_height, layer_height - p.min_layer_height);
    let thin = p.min_layer_height;
    // Bands this wide put every pin within half the rise of its leaf's plane, the plane half a
    // layer from the band's middle at most, the band's edges half a layer past it at most: two
    // pins then never differ by more than the rise, and differ by more than their distance's
    // share of it only within `bucket` of each other.
    let band = (p.max_rise - 2.0 * layer_height).max(0.0);
    let bound = band / 2.0 + layer_height;
    // The bounds when the triangles resolve the face's reach as `resolved` and pairs of their
    // corners reach `farthest`; no leaf steeper than the ironing allows over that reach.
    let metric = |resolved: f64, farthest: f64| -> Metric {
        let slope = if resolved > 0.0 { steepest.min(p.ironing / resolved) } else { steepest };
        let laps = Laps { down, thin, reach: resolved, farthest, slope };
        let turns = up + down < slope * farthest - laps.fall(farthest);
        Metric { up, down, slope, rise: p.max_rise, laps, leaves: tops.len(), turns }
    };
    let face = |spread: f64| if p.face_reach > 0.0 { reach(p.face_reach, spread, spread) } else { 0.0 };
    // Top surfaces all lying flat on a leaf's plane ask no leaf to bend.
    let all = patches(mesh, steepest)?;
    let asks = |f: &Facet| {
        let z = f.anchor[2];
        f.slope() > 0.0
            || nearest_leaf(tops, z)
                .and_then(|k| tops.get(k))
                .is_none_or(|&top| (z - top).abs() > ON_PLANE_MM)
    };
    if !all.iter().flat_map(|p| &p.facets).any(asks) {
        return Ok(flat);
    }
    // The triangles reach as far past the patches as any leaf can bend, by the bounds as loose
    // as triangles of edges up to two cells allow (`triangulate_evenly` keeps them within √2):
    // their spread a third of that edge squared at most, an equilateral's.
    let longest = 2.0 * p.cell;
    let margin = metric(face(longest * longest / 3.0), p.face_reach + 2.0 * longest).extent(bound) + p.cell;
    if !margin.is_finite() {
        return Ok(flat);
    }
    let shadow: Vec<Point> =
        mesh.bodies.iter().flat_map(|b| b.vertices.iter().map(|v| [v[0], v[1]])).collect();
    let t = triangulation(&all, p.cell, margin, &hull(&shadow))?;
    let inner = t.triangles.get(..t.inner).unwrap_or(&[]);
    if inner.is_empty() {
        return Ok(flat);
    }
    let at = |i: u32| t.points.get(i as usize).copied().unwrap_or([0.0; 2]);
    let corners = Corners::new(&t.points, inner, p.face_reach);
    let metric = metric(if p.face_reach > 0.0 { corners.coarsest() } else { 0.0 }, corners.farthest());
    let slope = metric.slope;
    if slope < 1e-3 {
        return Ok(flat);
    }
    let least = metric.up.min(metric.down);
    let leaves_apart = if least > 1e-6 { (2.0 * bound / least).ceil() as usize } else { tops.len() };
    let mut accepted = Accepted {
        metric: &metric,
        bucket: (2.0 * bound / metric.laps.rate()).max(p.cell),
        leaves: leaves_apart,
        cells: HashMap::new(),
        all: Vec::new(),
    };
    let parallel = p.parallel_layers.max(1);
    // The corners on each patch's facets.
    let mut on_patch: HashMap<usize, Vec<(u32, usize)>> = HashMap::new();
    for (corner, on) in t.on.iter().enumerate() {
        for &(pk, f) in on {
            on_patch.entry(pk).or_default().push((corner as u32, f));
        }
    }
    for (k, patch) in all.iter().enumerate() {
        // The corners on the patch's facets, each at the highest of their heights there.
        let mut pins: HashMap<u32, Pin> = HashMap::new();
        for &(corner, f) in on_patch.get(&k).into_iter().flatten() {
            let Some(facet) = patch.facets.get(f) else { continue };
            if facet.slope() > slope + SLACK {
                continue;
            }
            let (height, steep) = (facet.height(at(corner)), facet.slope());
            let pin = pins.entry(corner).or_insert(Pin { corner, height, slope: steep });
            if height > pin.height {
                pin.height = height;
            }
            pin.slope = pin.slope.max(steep);
        }
        let mut remaining: Vec<Pin> = pins.into_values().collect();
        remaining.sort_by_key(|q| q.corner);
        for _ in 0..MAX_BANDS {
            let mut heights: Vec<f64> = remaining.iter().map(|q| q.height).collect();
            heights.sort_by(f64::total_cmp);
            let Some([low, high]) = densest(&heights, band) else { break };
            let Some(leaf) = nearest_leaf(tops, (low + high) / 2.0) else { break };
            let z = tops.get(leaf).copied().unwrap_or(0.0);
            let (mut inside, mut rest): (Vec<Pin>, Vec<Pin>) = remaining
                .into_iter()
                .partition(|q| q.height >= low - layer_height && q.height <= high + layer_height);
            // Nearest the plane first.
            inside.sort_by(|a, b| {
                (a.height - z).abs().total_cmp(&(b.height - z).abs()).then(a.corner.cmp(&b.corner))
            });
            let mut took = false;
            for q in inside {
                let (point, mut value) = (at(q.corner), q.height - z);
                if value.abs() <= ON_PLANE_MM {
                    value = 0.0;
                }
                if value.abs() > bound {
                    rest.push(q);
                    continue;
                }
                // Each copy a slab lower, as much thicker than a layer as the face asks of
                // parallel slabs on the pin's slope; none where no slab can be that thick.
                let thicker = metric.laps.parallel(q.slope);
                let depth = if thicker <= metric.up + SLACK { parallel } else { 1 };
                let copies = (0..depth)
                    .filter_map(|j| Some((leaf.checked_sub(j)?, value - j as f64 * thicker)))
                    .filter(|&(k, _)| k >= 1);
                // Every value within the bound keeps every leaf within the rise.
                if copies.clone().all(|(k, v)| v.abs() <= bound && accepted.fits(k, point, v)) {
                    for (k, value) in copies {
                        accepted.hold(Held { leaf: k, corner: q.corner, at: point, value });
                    }
                    took = true;
                } else if q.height < low || q.height > high {
                    rest.push(q);
                }
            }
            remaining = rest;
            if !took || remaining.is_empty() {
                break;
            }
        }
    }
    let graph = Graph::new(&t.points, inner, &corners, slope, (p.face_reach > 0.0).then_some(p.ironing));
    let bounds = Bounds { graph: &graph, corners: &corners, laps: metric.laps, up, count: tops.len() };
    let pins: Vec<(usize, u32, f64)> = accepted.all.iter().map(|h| (h.leaf, h.corner, h.value)).collect();
    let values = bounds.leaves(t.points.len(), &pins)?;
    Ok(Leaves { triangulation: t, values })
}
