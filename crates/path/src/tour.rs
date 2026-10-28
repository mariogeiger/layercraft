//! The order of paths that leaves the nozzle the least travel between them: a travelling
//! salesman from where the nozzle is to wherever the last path ends, each open path taken from
//! either end and a loop from its start.
//!
//! A move costs first whether it leaves what hides it (`stays`), and so retracts and lifts, then
//! its length. Nearest first lays out a tour; two moves then shorten it until neither can.
//! Reversing a run of the tour (2-opt) lays each open path in it the other way round; moving a
//! run of up to `MAX_RUN` paths to another place, either way round (Or-opt), joins its
//! neighbours. A move costs the same both ways, so either changes the tour's cost only at the
//! ends of the run, and is weighed in constant time.

use crate::Path;
use layercraft_geom::Point;
use std::collections::HashMap;

/// The most passes over a tour; each one that shortens it is followed by another.
const MAX_PASSES: usize = 32;
/// The most paths Or-opt moves at once.
const MAX_RUN: usize = 3;
/// A tour shorter by less than this, in mm, is no shorter: the rounding of its sums.
const GAIN: f64 = 1e-9;

fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// What a run of moves costs: how many leave what hides them, then how far they go.
#[derive(Clone, Copy, PartialEq)]
struct Cost {
    leaves: i64,
    length: f64,
}

impl Cost {
    fn plus(self, o: Cost) -> Cost {
        Cost { leaves: self.leaves + o.leaves, length: self.length + o.length }
    }
    fn below(self, o: Cost) -> bool {
        self.leaves < o.leaves || (self.leaves == o.leaves && self.length < o.length - GAIN)
    }
}

/// The cost of each move, remembered: whether it stays hidden is costly to find out.
struct Moves<'a> {
    stays: &'a dyn Fn(Point, Point) -> bool,
    known: HashMap<[u64; 4], bool>,
}

impl Moves<'_> {
    fn cost(&mut self, a: Point, b: Point) -> Cost {
        let length = distance(a, b);
        let key = |p: Point, q: Point| [p[0].to_bits(), p[1].to_bits(), q[0].to_bits(), q[1].to_bits()];
        let k = if (a[0], a[1]) <= (b[0], b[1]) { key(a, b) } else { key(b, a) };
        let stays = match self.known.get(&k) {
            Some(&v) => v,
            None => {
                let v = (self.stays)(a, b);
                self.known.insert(k, v);
                v
            }
        };
        Cost { leaves: i64::from(!stays), length }
    }
}

/// Orders `paths` for the least travel from `at` through them all, a move that `stays` hidden
/// costing less than any that leaves, and leaves `at` where the last one ends.
pub(crate) fn tour(
    paths: Vec<Path>,
    at: &mut Point,
    out: &mut Vec<Path>,
    stays: &dyn Fn(Point, Point) -> bool,
) {
    let mut order = nearest_first(paths, *at);
    let mut moves = Moves { stays, known: HashMap::new() };
    shorten(&mut order, *at, &mut moves);
    if let Some(end) = order.last().and_then(Path::end) {
        *at = end;
    }
    out.extend(order);
}

/// Each path in turn the nearest to where the last one ended, from its nearer end.
fn nearest_first(mut paths: Vec<Path>, mut at: Point) -> Vec<Path> {
    let mut order = Vec::with_capacity(paths.len());
    while !paths.is_empty() {
        let mut best = (f64::MAX, 0, false);
        for (i, path) in paths.iter().enumerate() {
            let (Some(&a), Some(&b)) = (path.points.first(), path.points.last()) else { continue };
            let (da, db) = (distance(at, a), if path.closed { f64::MAX } else { distance(at, b) });
            if da < best.0 {
                best = (da, i, false);
            }
            if db < best.0 {
                best = (db, i, true);
            }
        }
        let mut path = paths.swap_remove(best.1);
        if best.2 {
            path.reverse();
        }
        if let Some(end) = path.end() {
            at = end;
        }
        order.push(path);
    }
    order
}

/// Shortens the tour `order` from `from` by 2-opt and Or-opt moves until neither does.
/// A move's cost is sought only where the tour could gain: it is shorter, or a move it drops
/// leaves.
fn shorten(order: &mut Vec<Path>, from: Point, moves: &mut Moves) {
    let n = order.len();
    if n < 2 {
        return;
    }
    let start = |p: &Path| p.points.first().copied().unwrap_or(from);
    let end = |p: &Path| p.end().unwrap_or(from);
    let zero = Cost { leaves: 0, length: 0.0 };
    for _ in 0..MAX_PASSES {
        let mut shorter = false;
        // 2-opt: the run `i..=j` reversed joins the point before it to `j`'s end, and `i`'s
        // start to the path after it.
        for i in 0..n {
            for j in i + 1..n {
                let (Some(pi), Some(pj)) = (order.get(i), order.get(j)) else { continue };
                let before = if i == 0 { from } else { order.get(i - 1).map_or(from, end) };
                let next = order.get(j + 1).map(start);
                let (si, ej) = (start(pi), end(pj));
                let was_length = distance(before, si) + next.map_or(0.0, |s| distance(ej, s));
                let now_length = distance(before, ej) + next.map_or(0.0, |s| distance(si, s));
                let was = moves.cost(before, si).plus(next.map_or(zero, |s| moves.cost(ej, s)));
                if !(now_length < was_length - GAIN || was.leaves > 0) {
                    continue;
                }
                let now = moves.cost(before, ej).plus(next.map_or(zero, |s| moves.cost(si, s)));
                if now.below(was)
                    && let Some(run) = order.get_mut(i..=j)
                {
                    run.reverse();
                    run.iter_mut().for_each(Path::reverse);
                    shorter = true;
                }
            }
        }
        // Or-opt: the run `k..k + len` taken out joins its neighbours; put in before `t`, either
        // way round, it parts `t` from the path before it.
        for len in 1..=MAX_RUN {
            for k in 0..n.saturating_sub(len - 1) {
                let (Some(first), Some(last)) = (order.get(k), order.get(k + len - 1)) else { continue };
                let (sk, ek) = (start(first), end(last));
                let before = if k == 0 { from } else { order.get(k - 1).map_or(from, end) };
                let next = order.get(k + len).map(start);
                let out_was = moves.cost(before, sk).plus(next.map_or(zero, |s| moves.cost(ek, s)));
                let out_now = next.map_or(zero, |s| moves.cost(before, s));
                let mut best: Option<(Cost, usize, bool)> = None;
                for t in (0..=n).filter(|&t| t < k || t > k + len) {
                    let p = if t == 0 { from } else { order.get(t - 1).map_or(from, end) };
                    let q = order.get(t).map(start);
                    let was = out_was.plus(q.map_or(zero, |q| moves.cost(p, q)));
                    for (a, b, turned) in [(sk, ek, false), (ek, sk, true)] {
                        let length = out_now.length + distance(p, a) + q.map_or(0.0, |q| distance(b, q));
                        if !(length < was.length - GAIN || was.leaves > 0) {
                            continue;
                        }
                        let now = out_now.plus(moves.cost(p, a)).plus(q.map_or(zero, |q| moves.cost(b, q)));
                        if now.below(was) && best.is_none_or(|(c, _, _)| now.below(c)) {
                            best = Some((now, t, turned));
                        }
                    }
                }
                if let Some((_, t, turned)) = best {
                    let mut run: Vec<Path> = order.drain(k..k + len).collect();
                    if turned {
                        run.reverse();
                        run.iter_mut().for_each(Path::reverse);
                    }
                    let at = if t > k { t - len } else { t };
                    order.splice(at..at, run);
                    shorter = true;
                }
            }
        }
        if !shorter {
            break;
        }
    }
}
