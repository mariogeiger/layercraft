//! The order of paths that leaves the nozzle the least travel between them: a travelling
//! salesman from where the nozzle is to wherever the last path ends, each open path taken from
//! either end and a loop from its start.
//!
//! A move costs first whether it leaves what hides it (`Cover`), and so retracts and lifts, then
//! its length. The tour is a cycle through the paths and the nozzle, whose second end is free:
//! the move from the last path to it costs nothing. Nearest first lays the cycle out; two
//! changes then shorten it until neither can: reversing a run of it (2-opt), and moving a run of
//! up to `MAX_RUN` paths elsewhere, either way round (Or-opt). Pair each move a change adds with
//! a move it drops at an end they share: the savings of the pairs, summed round the change from
//! the right pair on, stay positive all the way (Lin and Kernighan). So a change that shortens
//! the tour has an end whose new move is cheaper than the move it replaces there, or, for a run
//! it takes out, than what taking the run out saves. Each end therefore looks only at the ends
//! nearer than that (`Near`), and, where cheaper means hidden, at those a hidden move from it
//! reaches: every change that shortens the tour is found, for the cost of each end's neighbours.

use crate::Path;
use crate::link::Cover;
use crate::near::Near;
use layercraft_geom::Point;
use std::collections::{HashMap, VecDeque};
use std::hash::BuildHasherDefault;

/// The most rounds of looks at every path; a round that changes the tour is followed by
/// another, to find what the change made worth doing.
const MAX_ROUNDS: usize = 32;
/// The most paths Or-opt moves at once.
const MAX_RUN: usize = 3;
/// A tour shorter by less than this, in mm, is no shorter: the rounding of its sums.
const GAIN: f64 = 1e-9;
/// No end: the move to it costs nothing.
const NOWHERE: usize = usize::MAX;

/// Three moves, each between two ends: what a change drops or adds, a 2-opt's third from
/// `NOWHERE` to `NOWHERE`.
type Moves = [(usize, usize); 3];

fn distance(a: Point, b: Point) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// Hashes a pair of ends packed in one number by Fibonacci hashing, a multiplication by 2⁶⁴ over
/// the golden ratio, which spreads near numbers far apart: the pairs are many and small, and come
/// from the tour, not from outside.
#[derive(Default)]
struct Pairs(u64);

impl std::hash::Hasher for Pairs {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_u64(u64::from(b));
        }
    }
    fn write_u64(&mut self, n: u64) {
        self.0 = (self.0.rotate_left(5) ^ n).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
}

/// What moves cost: how many leave what hides them, then how far they go; or what a change
/// saves of both.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Cost {
    leaves: i64,
    length: f64,
}

impl Cost {
    const NONE: Cost = Cost { leaves: 0, length: 0.0 };

    fn plus(self, o: Cost) -> Cost {
        Cost { leaves: self.leaves + o.leaves, length: self.length + o.length }
    }
    fn minus(self, o: Cost) -> Cost {
        Cost { leaves: self.leaves - o.leaves, length: self.length - o.length }
    }
    /// Whether it is less than `o`: fewer leave, or as many and it is shorter.
    fn below(self, o: Cost) -> bool {
        self.leaves < o.leaves || (self.leaves == o.leaves && self.length < o.length)
    }
    /// Whether, saved, it shortens the tour by more than rounding.
    fn saves(self) -> bool {
        self.leaves > 0 || (self.leaves == 0 && self.length > GAIN)
    }
}

/// A change of the cycle, by positions.
#[derive(Clone, Copy, Debug)]
enum Change {
    /// Reverses the run after `x` up to `y`, dropping the moves after each.
    Reverse { x: usize, y: usize },
    /// Moves the run from `s` to `t` between `k` and the position after it, turned or not.
    Shift { s: usize, t: usize, k: usize, turned: bool },
}

/// The tour as a cycle of nodes, each a path or the nozzle, each run from one of its ends to the
/// other: node `v`'s ends are numbered `2v` and `2v + 1`.
struct Cycle<'a> {
    /// Each node's two ends, one point for a loop or a path whose ends meet; the nozzle's first
    /// end is free.
    ends: Vec<[Option<Point>; 2]>,
    /// The node at each position.
    order: Vec<usize>,
    /// Each node's position.
    pos: Vec<usize>,
    /// Whether each node is run from its second end to its first.
    turned: Vec<bool>,
    cover: &'a Cover<'a>,
    /// Whether the move between two ends stays hidden, once asked.
    hidden: HashMap<u64, bool, BuildHasherDefault<Pairs>>,
    /// How far a hidden move from each end reaches, once asked.
    reach: Vec<Option<f64>>,
    /// The ends a hidden move goes to from each end it may take anywhere, once asked: asking
    /// each pair once keeps the pairs remembered few.
    far: Vec<Option<Vec<usize>>>,
}

impl Cycle<'_> {
    fn len(&self) -> usize {
        self.order.len()
    }

    fn node(&self, i: usize) -> usize {
        self.order.get(i % self.len().max(1)).copied().unwrap_or(0)
    }

    fn pos(&self, v: usize) -> usize {
        self.pos.get(v).copied().unwrap_or(0)
    }

    fn turned(&self, v: usize) -> bool {
        self.turned.get(v).copied().unwrap_or(false)
    }

    fn point(&self, e: usize) -> Option<Point> {
        self.ends.get(e / 2).and_then(|ends| ends.get(e % 2)).copied().flatten()
    }

    /// Whether node `v`'s two ends are one point.
    fn single(&self, v: usize) -> bool {
        self.ends.get(v).is_some_and(|[a, b]| a == b)
    }

    /// The end the node at position `i` is entered by, and the end it is left by.
    fn a(&self, i: usize) -> usize {
        let v = self.node(i);
        2 * v + usize::from(self.turned(v))
    }
    fn b(&self, i: usize) -> usize {
        let v = self.node(i);
        2 * v + 1 - usize::from(self.turned(v))
    }

    fn length(&self, p: usize, q: usize) -> f64 {
        match (self.point(p), self.point(q)) {
            (Some(a), Some(b)) => distance(a, b),
            _ => 0.0,
        }
    }

    fn cost(&mut self, p: usize, q: usize) -> Cost {
        let (Some(a), Some(b)) = (self.point(p), self.point(q)) else { return Cost::NONE };
        let one = |e: usize| if self.single(e / 2) { e & !1 } else { e };
        let (p, q) = (one(p), one(q));
        let cover = self.cover;
        let key = (p.min(q) as u64) << 32 | p.max(q) as u64;
        let hidden = *self.hidden.entry(key).or_insert_with(|| cover.stays(a, b));
        Cost { leaves: i64::from(!hidden), length: distance(a, b) }
    }

    /// The ends a hidden move from end `p` goes to, its own node's aside, taken out of `far`.
    fn far_from(&mut self, p: usize) -> Vec<usize> {
        if let Some(far) = self.far.get_mut(p).and_then(Option::take) {
            return far;
        }
        let Some(a) = self.point(p) else { return Vec::new() };
        (0..self.ends.len() * 2)
            .filter(|&q| q / 2 != p / 2 && (q % 2 == 0 || !self.single(q / 2)))
            .filter(|&q| self.point(q).is_some_and(|b| self.cover.stays(a, b)))
            .collect()
    }

    /// Whether the move between ends `p` and `q` costs less than `bound`, asking whether it
    /// leaves only where its length does not tell.
    fn cheaper(&mut self, p: usize, q: usize, bound: Cost) -> bool {
        let length = self.length(p, q);
        match bound.leaves {
            l if l < 0 => false,
            0 => length < bound.length && self.cost(p, q).leaves == 0,
            1 if length < bound.length => true,
            1 => self.cost(p, q).leaves == 0,
            _ => true,
        }
    }

    fn reach(&mut self, e: usize) -> f64 {
        if let Some(Some(r)) = self.reach.get(e) {
            return *r;
        }
        let r = self.point(e).map_or(0.0, |p| self.cover.reach(p));
        if let Some(slot) = self.reach.get_mut(e) {
            *slot = Some(r);
        }
        r
    }

    /// The moves a change drops and those it adds, as pairs of ends.
    fn moves(&self, change: Change) -> (Moves, Moves) {
        let n = self.len();
        match change {
            Change::Reverse { x, y } => {
                let (bx, ax, by, ay) = (self.b(x), self.a(x + 1), self.b(y), self.a(y + 1));
                ([(bx, ax), (by, ay), (NOWHERE, NOWHERE)], [(bx, by), (ax, ay), (NOWHERE, NOWHERE)])
            }
            Change::Shift { s, t, k, turned } => {
                let (u, a, b, v) = (self.b(s + n - 1), self.a(s), self.b(t), self.a(t + 1));
                let (x, y) = (self.b(k), self.a(k + 1));
                let added = if turned { [(u, v), (x, b), (a, y)] } else { [(u, v), (x, a), (b, y)] };
                ([(u, a), (b, v), (x, y)], added)
            }
        }
    }

    /// Whether a change is one: two moves dropped, or a short run put between two other paths.
    fn valid(&self, change: Change) -> bool {
        let n = self.len();
        match change {
            Change::Reverse { x, y } => x % n != y % n,
            Change::Shift { s, t, k, .. } => {
                let m = (t + n - s) % n + 1;
                m <= MAX_RUN && m + 2 <= n && (k + n + 1 - s) % n > m
            }
        }
    }

    /// What a change saves, where it shortens the tour by more than `best` does: whether the
    /// moves it adds leave is asked only while it still can.
    fn saving(&mut self, change: Change, best: Option<Cost>) -> Option<Cost> {
        let (dropped, added) = self.moves(change);
        let length = dropped.iter().map(|&(p, q)| self.length(p, q)).sum::<f64>()
            - added.iter().map(|&(p, q)| self.length(p, q)).sum::<f64>();
        let leaving: i64 = dropped.iter().map(|&(p, q)| self.cost(p, q).leaves).sum();
        let better = |saving: Cost| saving.saves() && best.is_none_or(|b| b.below(saving));
        let mut saving = Cost { leaves: leaving, length };
        for &(p, q) in &added {
            if !better(saving) {
                return None;
            }
            saving.leaves -= self.cost(p, q).leaves;
        }
        better(saving).then_some(saving)
    }

    fn set(&mut self, at: usize, v: usize) {
        if let Some(slot) = self.order.get_mut(at) {
            *slot = v;
        }
        if let Some(p) = self.pos.get_mut(v) {
            *p = at;
        }
    }

    fn flip(&mut self, v: usize) {
        if let Some(t) = self.turned.get_mut(v) {
            *t = !*t;
        }
    }

    fn apply(&mut self, change: Change) {
        let n = self.len();
        match change {
            Change::Reverse { x, y } => {
                // The run after `x` up to `y`, or the rest of the cycle where shorter: the same
                // cycle either way, run the other way round.
                let (from, length) = ((x + 1) % n, (y + n - x) % n);
                let (from, length) = if 2 * length > n { ((y + 1) % n, n - length) } else { (from, length) };
                for i in 0..length / 2 {
                    let (p, q) = ((from + i) % n, (from + length - 1 - i) % n);
                    let (v, w) = (self.node(p), self.node(q));
                    self.set(p, w);
                    self.set(q, v);
                }
                for i in 0..length {
                    let v = self.node(from + i);
                    self.flip(v);
                }
            }
            Change::Shift { s, t, k, turned } => {
                // What lies between the run and its new place shifts over, the shorter way round.
                let m = (t + n - s) % n + 1;
                let run: Vec<usize> = (0..m).map(|i| self.node(s + i)).collect();
                let (after, before) = ((k + n - t) % n, (s + n - 1 - k) % n);
                let start = if after <= before {
                    for i in 0..after {
                        let v = self.node(t + 1 + i);
                        self.set((s + i) % n, v);
                    }
                    (s + after) % n
                } else {
                    for i in (0..before).rev() {
                        let v = self.node(k + 1 + i);
                        self.set((k + 1 + m + i) % n, v);
                    }
                    (k + 1) % n
                };
                for (i, &v) in run.iter().enumerate() {
                    let at = if turned { (start + m - 1 - i) % n } else { (start + i) % n };
                    self.set(at, v);
                    if turned {
                        self.flip(v);
                    }
                }
            }
        }
    }

    /// The nodes within a run's length of the moves a change drops: those whose moves, or the
    /// runs from them, it changes.
    fn touched(&self, change: Change) -> Vec<usize> {
        let n = self.len();
        let at = match change {
            Change::Reverse { x, y } => vec![x, y],
            Change::Shift { s, t, k, .. } => vec![s + n - 1, t, k],
        };
        let around = move |k: usize| (0..2 * MAX_RUN).map(move |d| k + MAX_RUN * n + 1 + d - MAX_RUN);
        at.into_iter().flat_map(around).map(|i| self.node(i)).collect()
    }

    /// The changes that put a new move between end `p`, entered (`p_out` false) or left by the
    /// node at position `i`, and end `q`, likewise of the node at position `j`.
    fn changes(&self, (i, p_out): (usize, bool), (j, q_out): (usize, bool)) -> [Option<Change>; 7] {
        let n = self.len();
        let back = |x: usize, by: usize| (x + n - by % n) % n;
        let ahead = |x: usize, by: usize| (x + by) % n;
        let mut all = [None; 7];
        let shifts = |m: usize| -> [Change; 2] {
            let shift = |s, t, k, turned| Change::Shift { s, t, k, turned };
            match (p_out, q_out) {
                // `p` ends the run, turned to `q`'s end; or `p`'s move takes the run ending at `q`, turned.
                (true, true) => [shift(back(i, m - 1), i, j, true), shift(back(j, m - 1), j, i, true)],
                (true, false) => {
                    [shift(back(i, m - 1), i, back(j, 1), false), shift(j, ahead(j, m - 1), i, false)]
                }
                (false, false) => {
                    [shift(i, ahead(i, m - 1), back(j, 1), true), shift(j, ahead(j, m - 1), back(i, 1), true)]
                }
                (false, true) => {
                    [shift(i, ahead(i, m - 1), j, false), shift(back(j, m - 1), j, back(i, 1), false)]
                }
            }
        };
        if let Some(first) = all.first_mut() {
            *first = match (p_out, q_out) {
                (true, true) => Some(Change::Reverse { x: i, y: j }),
                (false, false) => Some(Change::Reverse { x: back(i, 1), y: back(j, 1) }),
                _ => None,
            };
        }
        for m in 1..=MAX_RUN {
            for (slot, change) in all.iter_mut().skip(2 * m - 1).zip(shifts(m)) {
                *slot = Some(change);
            }
        }
        all
    }

    /// The change that saves the most of those putting a new move at an end of node `v`, in
    /// place of its move there.
    fn best(&mut self, v: usize, near: &Near, free: usize, found: &mut Vec<(f64, usize)>) -> Option<Change> {
        let n = self.len();
        let i = self.pos(v);
        let roles = if self.single(v) {
            [(2 * v, false), (2 * v, true)]
        } else {
            [(self.a(i), false), (self.b(i), true)]
        };
        let mut best: Option<(Cost, Change)> = None;
        for (p, p_out) in roles {
            let Some(point) = self.point(p) else { continue };
            // A new move from `p` gains where it is cheaper than the move it replaces there, or
            // than what taking out a run that `p` ends saves.
            let replaced = if p_out {
                self.cost(self.b(i), self.a(i + 1))
            } else {
                self.cost(self.b(i + n - 1), self.a(i))
            };
            let mut runs = [replaced; MAX_RUN];
            for (m, run) in (1..=MAX_RUN.min(n.saturating_sub(2))).zip(runs.iter_mut()) {
                let (s, t) = if p_out { ((i + n + 1 - m) % n, i) } else { (i, (i + m - 1) % n) };
                let (u, a, b, w) = (self.b(s + n - 1), self.a(s), self.b(t), self.a(t + 1));
                let out = self.cost(u, a).plus(self.cost(b, w)).minus(self.cost(u, w));
                if replaced.below(out) {
                    *run = out;
                }
            }
            let bound = runs.iter().fold(replaced, |b, &r| if b.below(r) { r } else { b });
            // Within the bound's length, any end; beyond it, where cheaper means hidden, those
            // a hidden move from `p` reaches.
            let (radius, anywhere) = match bound.leaves {
                0 if bound.length > 0.0 => (bound.length, false),
                1 => {
                    let reach = self.reach(p);
                    if reach.is_finite() { (bound.length.max(reach), false) } else { (bound.length, true) }
                }
                l if l > 1 => (f64::INFINITY, false),
                _ => continue,
            };
            near.within(point, radius, found);
            if anywhere {
                let far = self.far_from(p);
                let beyond = far.iter().map(|&q| (self.length(p, q), q)).filter(|&(d, _)| d > radius);
                found.extend(beyond.map(|(d, q)| (d * d, q)));
                if let Some(slot) = self.far.get_mut(p) {
                    *slot = Some(far);
                }
            }
            for q in std::iter::once(free).chain(found.iter().map(|&(_, q)| q)) {
                let u = q / 2;
                if u == v || !self.cheaper(p, q, bound) {
                    continue;
                }
                let j = self.pos(u);
                // Which way round `q` is met: entered, left, or both, its ends one point.
                let leaves_u = q % 2 == 1 - usize::from(self.turned(u));
                let q_roles: &[bool] =
                    if self.single(u) { &[false, true] } else { std::slice::from_ref(&leaves_u) };
                for &q_out in q_roles {
                    for (k, change) in self.changes((i, p_out), (j, q_out)).into_iter().enumerate() {
                        // The 2-opt and the run put by `p` pair the new move with the one it
                        // replaces; the run `p` ends, with that or with what taking it out saves.
                        let own =
                            if k % 2 == 1 { runs.get(k / 2).copied().unwrap_or(replaced) } else { replaced };
                        let Some(change) = change.filter(|&c| self.valid(c)) else { continue };
                        if !self.cheaper(p, q, own) {
                            continue;
                        }
                        if let Some(saving) = self.saving(change, best.map(|(b, _)| b)) {
                            best = Some((saving, change));
                        }
                    }
                }
            }
        }
        best.map(|(_, change)| change)
    }

    /// Shortens the cycle until no change does, or `MAX_ROUNDS` rounds of looks at each node.
    fn shorten(&mut self, free: usize) {
        let n = self.len();
        let near = Near::new((0..2 * n).filter_map(|e| {
            let point = self.point(e)?;
            (e % 2 == 0 || !self.single(e / 2)).then_some((point, e))
        }));
        let mut found = Vec::new();
        let mut queue: VecDeque<usize> = VecDeque::new();
        let mut queued = vec![false; n];
        let (mut looks, mut changed) = (0, true);
        while changed && looks < MAX_ROUNDS * n {
            changed = false;
            queue.extend((0..n).map(|i| self.node(i)));
            queued.iter_mut().for_each(|q| *q = true);
            while let Some(v) = queue.pop_front() {
                if let Some(q) = queued.get_mut(v) {
                    *q = false;
                }
                looks += 1;
                if looks > MAX_ROUNDS * n {
                    break;
                }
                let Some(change) = self.best(v, &near, free, &mut found) else { continue };
                let touched = self.touched(change);
                self.apply(change);
                changed = true;
                for w in touched {
                    if let Some(q) = queued.get_mut(w).filter(|q| !**q) {
                        *q = true;
                        queue.push_back(w);
                    }
                }
            }
        }
    }
}

/// The ends of a path, its start twice for a loop; none where it has no finite point.
fn ends(path: &Path) -> Option<[Point; 2]> {
    let (&a, &b) = (path.points.first()?, path.points.last()?);
    let b = if path.closed { a } else { b };
    (a.iter().chain(&b).all(|c| c.is_finite())).then_some([a, b])
}

/// Each node in turn the nearest to where the last one ended, from its nearer end, the nozzle
/// first: the nodes and whether each is run from its second end.
fn nearest_first(ends: &[[Option<Point>; 2]], nozzle: usize, mut at: Point) -> Vec<(usize, bool)> {
    let mut near =
        Near::new(ends.iter().enumerate().filter(|&(v, _)| v != nozzle).flat_map(|(v, [a, b])| {
            let second = b.filter(|b| Some(*b) != *a);
            [a.map(|a| (a, 2 * v)), second.map(|b| (b, 2 * v + 1))].into_iter().flatten()
        }));
    let mut order = vec![(nozzle, false)];
    let mut laid = vec![false; ends.len()];
    if let Some(l) = laid.get_mut(nozzle) {
        *l = true;
    }
    while let Some(e) = near.nearest(at) {
        let (v, turned) = (e / 2, e % 2 == 1);
        near.take(2 * v);
        near.take(2 * v + 1);
        if let Some(l) = laid.get_mut(v) {
            *l = true;
        }
        if let Some(Some(end)) = ends.get(v).and_then(|ends| ends.get(usize::from(!turned))) {
            at = *end;
        }
        order.push((v, turned));
    }
    order.extend(laid.iter().enumerate().filter(|(_, l)| !**l).map(|(v, _)| (v, false)));
    order
}

/// Orders `paths` for the least travel from `at` through them all, a move that `cover` hides
/// costing less than any that leaves it, and leaves `at` where the last one ends.
pub(crate) fn tour(paths: Vec<Path>, at: &mut Point, out: &mut Vec<Path>, cover: &Cover) {
    let (paths, pointless): (Vec<Path>, Vec<Path>) = paths.into_iter().partition(|p| ends(p).is_some());
    let mut ends: Vec<[Option<Point>; 2]> =
        paths.iter().map(|p| ends(p).map_or([None, None], |[a, b]| [Some(a), Some(b)])).collect();
    let nozzle = ends.len();
    ends.push([None, Some(*at)]);
    let order = nearest_first(&ends, nozzle, *at);
    let count = ends.len();
    let mut cycle = Cycle {
        ends,
        order: order.iter().map(|&(v, _)| v).collect(),
        pos: vec![0; count],
        turned: vec![false; count],
        cover,
        hidden: HashMap::default(),
        reach: vec![None; 2 * count],
        far: vec![None; 2 * count],
    };
    for (i, &(v, turned)) in order.iter().enumerate() {
        cycle.set(i, v);
        if turned {
            cycle.flip(v);
        }
    }
    cycle.shorten(2 * nozzle);
    // Round the cycle from the nozzle, the way it leaves the nozzle.
    let (from, back) = (cycle.pos(nozzle), cycle.turned(nozzle));
    let mut slots: Vec<Option<Path>> = paths.into_iter().map(Some).collect();
    for k in 1..count {
        let v = cycle.node(if back { from + count - k } else { from + k });
        if let Some(mut path) = slots.get_mut(v).and_then(Option::take) {
            if cycle.turned(v) != back {
                path.reverse();
            }
            if let Some(end) = path.end() {
                *at = end;
            }
            out.push(path);
        }
    }
    out.extend(pointless);
}
