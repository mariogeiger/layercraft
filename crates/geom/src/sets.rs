//! Disjoint sets and the chains of a graph, which several algorithms walk the same way.
//!
//! `DisjointSets` joins indices into classes, each named by its least member, so that the classes
//! come out the same whatever order they were joined in. `chains` cuts a graph into the paths
//! between its ends and junctions, the vertices where other than two edges meet, then its loops.

use std::collections::HashSet;

/// Indices joined into classes.
#[derive(Debug, Clone)]
pub struct DisjointSets {
    parent: Vec<usize>,
}

impl DisjointSets {
    /// `n` indices, each a class of its own.
    pub fn new(n: usize) -> DisjointSets {
        DisjointSets { parent: (0..n).collect() }
    }

    /// The least member of `i`'s class, the path there halved.
    pub fn root(&mut self, mut i: usize) -> usize {
        for _ in 0..self.parent.len() {
            let up = self.parent.get(i).copied().unwrap_or(i);
            if up == i {
                break;
            }
            let next = self.parent.get(up).copied().unwrap_or(up);
            if let Some(slot) = self.parent.get_mut(i) {
                *slot = next;
            }
            i = next;
        }
        i
    }

    /// Joins the classes of `a` and `b`.
    pub fn unite(&mut self, a: usize, b: usize) {
        let (x, y) = (self.root(a), self.root(b));
        if let Some(slot) = self.parent.get_mut(x.max(y)) {
            *slot = x.min(y);
        }
    }

    /// The classes, in the order of their least members, each with its members in order.
    pub fn classes(&mut self) -> Vec<Vec<usize>> {
        let mut number: Vec<Option<usize>> = vec![None; self.parent.len()];
        let mut out: Vec<Vec<usize>> = Vec::new();
        for i in 0..self.parent.len() {
            let root = self.root(i);
            let Some(slot) = number.get_mut(root) else { continue };
            let k = *slot.get_or_insert_with(|| {
                out.push(Vec::new());
                out.len() - 1
            });
            if let Some(class) = out.get_mut(k) {
                class.push(i);
            }
        }
        out
    }
}

/// The graph whose vertex `i` has the neighbours `neighbours[i]` cut into chains, each as its
/// vertices in order and whether it closes: from each vertex of other than two edges, in order,
/// along each of its edges not yet walked, to the next such vertex; then from each vertex left,
/// round its loop back to it.
pub fn chains(neighbours: &[Vec<usize>]) -> Vec<(Vec<usize>, bool)> {
    let degree = |i: usize| neighbours.get(i).map_or(0, Vec::len);
    let mut used: HashSet<(usize, usize)> = HashSet::new();
    let mut out = Vec::new();
    let walk = |start: usize, next: usize, used: &mut HashSet<(usize, usize)>| {
        let mut ids = vec![start];
        let (mut from, mut at) = (start, next);
        loop {
            used.insert((from.min(at), from.max(at)));
            ids.push(at);
            if degree(at) != 2 || at == start {
                break;
            }
            let Some(&to) = neighbours.get(at).and_then(|l| l.iter().find(|&&m| m != from)) else { break };
            (from, at) = (at, to);
        }
        ids
    };
    for loops in [false, true] {
        for start in 0..neighbours.len() {
            if !loops && degree(start) == 2 {
                continue;
            }
            for &next in neighbours.get(start).into_iter().flatten() {
                if used.contains(&(start.min(next), start.max(next))) {
                    continue;
                }
                let mut ids = walk(start, next, &mut used);
                let closed = loops && ids.len() > 2 && ids.last() == ids.first();
                if closed {
                    ids.pop();
                }
                out.push((ids, closed));
            }
        }
    }
    out
}
