//! The envelopes of the pins over the leaves' triangles, and the leaves they make.
//!
//! Along one leaf, `d` may change between neighbouring corners by `s` times their distance: the
//! triangles' edges and the corners facing each other across an edge, the second making the
//! paths' lengths nearly straight; and, where those paths are not straight enough to keep a
//! leaf from rising more than the ironing allows within the face's reach (`face`), by the
//! ironing's depth over `R` per mm between the corners that can meet under the face. Between
//! leaves, the slabs' thickness and the face's pairs bound it (`face::Laps`). The greatest field below the pins is
//! their cones along a leaf, Dijkstra's algorithm on those bounds, closed across the leaves: to
//! the next leaf by the slabs' thickening, to the one before by the face's pairs, pass after
//! pass until nothing lowers; the least is the same of the pins negated, the leaves in reverse.
//! Each leaf then takes the value nearest flat between them, `d = max(Lo, min(0, U))`.

use crate::SectionError;
use crate::face::{Corners, Laps};
use layercraft_geom::Point;
use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap};

/// The bounds along one leaf: each corner's neighbours, and how far `d` may change to each.
pub(crate) struct Graph {
    starts: Vec<usize>,
    edges: Vec<(u32, f64)>,
}

/// The costs a search from one corner reached, reset between searches.
struct Ball {
    best: Vec<f64>,
    reached: Vec<u32>,
}

/// The heap's entries, ordered by value.
#[derive(PartialEq)]
struct Entry(f64, u32);
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

impl Graph {
    /// The bounds of a leaf changing by `slope` per mm along the triangles' edges and across
    /// them, and, when the face irons no deeper than `ironing`, by that over `R` per mm between
    /// the corners that meet under the face wherever those paths would let it change more.
    pub fn new(
        points: &[Point],
        triangles: &[[u32; 3]],
        corners: &Corners,
        slope: f64,
        ironing: Option<f64>,
    ) -> Graph {
        let n = points.len();
        let at = |i: u32| points.get(i as usize).copied().unwrap_or([0.0; 2]);
        let length = |a: u32, b: u32| (at(a)[0] - at(b)[0]).hypot(at(a)[1] - at(b)[1]);
        // Each edge's far corners in its triangles.
        let mut facing: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
        for &[a, b, c] in triangles {
            for (p, q, far) in [(a, b, c), (b, c, a), (c, a, b)] {
                facing.entry((p.min(q), p.max(q))).or_default().push(far);
            }
        }
        // Each corner's neighbours, a pair bound twice held to the tighter bound.
        let mut lists: Vec<Vec<(u32, f64)>> = vec![Vec::new(); n];
        let link = |lists: &mut Vec<Vec<(u32, f64)>>, a: u32, b: u32, w: f64| {
            for (x, y) in [(a, b), (b, a)] {
                let Some(list) = lists.get_mut(x as usize) else { continue };
                match list.iter_mut().find(|(z, _)| *z == y) {
                    Some(pair) => pair.1 = pair.1.min(w),
                    None => list.push((y, w)),
                }
            }
        };
        for (&(p, q), far) in &facing {
            link(&mut lists, p, q, slope * length(p, q));
            if let [a, b] = far.as_slice() {
                link(&mut lists, *a, *b, slope * length(*a, *b));
            }
        }
        // The pairs under the face that the paths along the leaf do not bind tightly enough.
        let along = Graph::packed(&lists);
        let mut ball = Ball { best: vec![f64::INFINITY; n], reached: Vec::new() };
        let mut extra: Vec<(u32, u32, f64)> = Vec::new();
        if let Some(depth) = ironing.filter(|&i| i.is_finite() && i > 0.0 && slope > 0.0) {
            for v in 0..n as u32 {
                let mut pairs: Vec<(u32, f64)> = Vec::new();
                corners.meeting(v, |w, d| {
                    if w > v {
                        pairs.push((w, depth * d / corners.reach(v, w)));
                    }
                });
                let Some(limit) = pairs.iter().map(|p| p.1).reduce(f64::max) else { continue };
                along.ball(v, limit, &mut ball);
                let reached = &ball.best;
                extra.extend(
                    pairs
                        .into_iter()
                        .filter(|(w, cost)| reached.get(*w as usize).is_none_or(|c| c > cost))
                        .map(|(w, cost)| (v, w, cost)),
                );
            }
        }
        for (v, w, cost) in extra {
            link(&mut lists, v, w, cost);
        }
        Graph::packed(&lists)
    }

    fn packed(lists: &[Vec<(u32, f64)>]) -> Graph {
        let mut starts = Vec::with_capacity(lists.len() + 1);
        let mut edges = Vec::with_capacity(lists.iter().map(Vec::len).sum());
        for list in lists {
            starts.push(edges.len());
            edges.extend_from_slice(list);
        }
        starts.push(edges.len());
        Graph { starts, edges }
    }

    fn neighbours(&self, v: u32) -> &[(u32, f64)] {
        match (self.starts.get(v as usize), self.starts.get(v as usize + 1)) {
            (Some(&a), Some(&b)) => self.edges.get(a..b).unwrap_or(&[]),
            _ => &[],
        }
    }

    /// The cost of the cheapest path from `v` to each corner it reaches for at most `limit`, in
    /// `ball.best`, the corners reached listed in `ball.reached`.
    fn ball(&self, v: u32, limit: f64, ball: &mut Ball) {
        for &u in &ball.reached {
            if let Some(slot) = ball.best.get_mut(u as usize) {
                *slot = f64::INFINITY;
            }
        }
        ball.reached.clear();
        let mut heap = BinaryHeap::new();
        if let Some(slot) = ball.best.get_mut(v as usize) {
            *slot = 0.0;
            ball.reached.push(v);
            heap.push(Reverse(Entry(0.0, v)));
        }
        while let Some(Reverse(Entry(c, u))) = heap.pop() {
            if ball.best.get(u as usize).is_none_or(|&b| c > b) {
                continue;
            }
            for &(w, cost) in self.neighbours(u) {
                let next = c + cost;
                if let Some(slot) = ball.best.get_mut(w as usize)
                    && next <= limit
                    && next < *slot
                {
                    if !slot.is_finite() {
                        ball.reached.push(w);
                    }
                    *slot = next;
                    heap.push(Reverse(Entry(next, w)));
                }
            }
        }
    }

    /// `min_p (v_p + path(p, x))` at every corner, the paths along the leaf's bounds, where
    /// below zero; `None` where nowhere.
    fn cones(&self, n: usize, sources: impl IntoIterator<Item = (u32, f64)>) -> Option<Vec<f64>> {
        let mut best = vec![f64::INFINITY; n];
        let mut heap = BinaryHeap::new();
        for (at, v) in sources {
            if let Some(slot) = best.get_mut(at as usize)
                && v < *slot
                && v < 0.0
            {
                *slot = v;
                heap.push(Reverse(Entry(v, at)));
            }
        }
        if heap.is_empty() {
            return None;
        }
        while let Some(Reverse(Entry(v, at))) = heap.pop() {
            if best.get(at as usize).is_none_or(|&b| v > b) {
                continue;
            }
            for &(next, cost) in self.neighbours(at) {
                let w = v + cost;
                // Past zero the envelope bends no leaf.
                if w >= 0.0 {
                    continue;
                }
                if let Some(slot) = best.get_mut(next as usize)
                    && w < *slot
                {
                    *slot = w;
                    heap.push(Reverse(Entry(w, next)));
                }
            }
        }
        Some(best)
    }
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

/// What the closure across leaves works with.
pub(crate) struct Bounds<'a> {
    pub graph: &'a Graph,
    pub corners: &'a Corners<'a>,
    pub laps: Laps,
    /// How much more `d` may grow per leaf up.
    pub up: f64,
    pub count: usize,
}

impl Bounds<'_> {
    /// One leaf on from an envelope, by `up` alone, where the slabs thicken.
    fn shifted(&self, f: &Option<Vec<f64>>) -> Option<Vec<f64>> {
        let f = f.as_ref()?;
        self.graph.cones(f.len(), f.iter().enumerate().map(|(k, &v)| (k as u32, v + self.up)))
    }

    /// One leaf on from an envelope where the slabs thin or the face laps: by each pair's
    /// weight, then along the leaf. The pairs are symmetric, so one leaf on is the same either
    /// way.
    fn lapped(&self, f: &Option<Vec<f64>>) -> Option<Vec<f64>> {
        let f = f.as_ref()?;
        let mut next = vec![f64::INFINITY; f.len()];
        for (v, &value) in f.iter().enumerate() {
            // Past zero the envelope bends no leaf: from here a pair's weight must stay under
            // `−value`, `t d²/R²` under `−value − down`, `R` at most the coarsest.
            let left = -value - self.laps.down;
            if !(value.is_finite() && left > 0.0) {
                continue;
            }
            if self.laps.reach > 0.0 {
                let limit = self.laps.reach * (left / self.laps.thin).sqrt();
                self.corners.meeting_within(v as u32, limit, |w, d| {
                    if let Some(slot) = next.get_mut(w as usize) {
                        *slot = slot.min(value + self.laps.weight(d, self.corners.reach(v as u32, w)));
                    }
                });
            } else if let Some(slot) = next.get_mut(v) {
                *slot = slot.min(value + self.laps.down);
            }
        }
        self.graph.cones(next.len(), next.iter().enumerate().map(|(k, &v)| (k as u32, v)))
    }

    /// Closes envelopes under the bounds between leaves, each leaf's own to begin with: from a
    /// leaf to the next by `up`, to the one before by the laps, pass after pass until neither
    /// lowers anything, a leaf casting each way only once lowered since it last did.
    fn close(&self, env: &mut [Option<Vec<f64>>]) -> Result<(), SectionError> {
        let n = env.len();
        let mut due = vec![(true, true); n];
        let mark = |due: &mut Vec<(bool, bool)>, k: usize| {
            if let Some(d) = due.get_mut(k) {
                *d = (true, true);
            }
        };
        for _ in 0..=n {
            for k in 1..n {
                if due.get(k - 1).is_some_and(|d| d.0) {
                    let next = env.get(k - 1).and_then(|f| self.shifted(f));
                    if env.get_mut(k).is_some_and(|slot| lower(slot, next)) {
                        mark(&mut due, k);
                    }
                }
            }
            due.iter_mut().for_each(|d| d.0 = false);
            for k in (0..n.saturating_sub(1)).rev() {
                if due.get(k + 1).is_some_and(|d| d.1) {
                    let next = env.get(k + 1).and_then(|f| self.lapped(f));
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

    /// Each leaf's displacement at every corner for pins `(leaf, corner, value)`:
    /// `max(Lo, min(0, U))`; `None` for a leaf left flat.
    pub fn leaves(
        &self,
        n: usize,
        pins: &[(usize, u32, f64)],
    ) -> Result<Vec<Option<Vec<f64>>>, SectionError> {
        let mut by_leaf: BTreeMap<usize, Vec<(u32, f64)>> = BTreeMap::new();
        for &(leaf, corner, value) in pins {
            by_leaf.entry(leaf).or_default().push((corner, value));
        }
        let own = |k: usize, sign: f64| -> Option<Vec<f64>> {
            let pins = by_leaf.get(&k)?;
            self.graph.cones(n, pins.iter().map(|&(at, v)| (at, sign * v)))
        };
        let mut upper: Vec<Option<Vec<f64>>> = (0..self.count).map(|k| own(k, 1.0)).collect();
        self.close(&mut upper)?;
        let mut lower: Vec<Option<Vec<f64>>> = (0..self.count).rev().map(|k| own(k, -1.0)).collect();
        self.close(&mut lower)?;
        lower.reverse();
        let mut out = Vec::with_capacity(self.count);
        for k in 0..self.count {
            let (u, l) = (upper.get(k).and_then(Option::as_ref), lower.get(k).and_then(Option::as_ref));
            if k == 0 || (u.is_none() && l.is_none()) {
                out.push(None);
                continue;
            }
            let at = |values: Option<&Vec<f64>>, i: usize| values.and_then(|v| v.get(i)).copied();
            let values: Vec<f64> = (0..n)
                .map(|i| {
                    let up = at(u, i).unwrap_or(f64::INFINITY);
                    let lo = -at(l, i).unwrap_or(f64::INFINITY);
                    lo.max(up.min(0.0))
                })
                .collect();
            out.push(Some(values));
        }
        Ok(out)
    }
}
