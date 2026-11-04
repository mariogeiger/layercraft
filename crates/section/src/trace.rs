//! Walking the leaves' triangles along a segment of the plate, every step decided exactly.
//!
//! The triangles meet edge to edge, counterclockwise, and cover the plate under the mesh. A
//! segment from `a` to `b` starts in the triangle holding `a`, inside it, on one of its edges or
//! at one of its corners, and leaves each triangle it passes through by the edge running from a
//! corner on its right to one on its left, going round counterclockwise, or by a corner it
//! meets; across an edge it enters the triangle beyond, at a corner the one whose angle there
//! holds its direction, or it runs along an edge. Every decision is the side of a line a point
//! lies on, taken exactly on the points as they are (`orientation`): the points where the
//! segment passes from a triangle to the next come in their true order however close they lie,
//! the same for every triangle of the mesh that has the segment as an edge.

use layercraft_geom::{Buckets, GeomError, Point, orientation};
use std::cmp::Ordering::{self, Equal, Greater, Less};
use std::collections::HashMap;

/// Where a point lies on the triangles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Spot {
    /// At a corner.
    Corner(u32),
    /// Inside the edge from the first corner to the second, counterclockwise in a triangle.
    Edge(u32, u32),
    /// Inside a triangle.
    Inside(u32),
    /// On none.
    Outside,
}

/// A point where a segment passes from one triangle to another.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum Event {
    /// Through a corner.
    Corner(u32),
    /// Across the inside of an edge, named by its corners, the lower first.
    Crossing(u32, u32),
}

/// A piece of a segment between two of its events, or its ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Piece {
    /// Inside a triangle.
    Inside(u32),
    /// Along an edge, from its first corner toward its second.
    Along(u32, u32),
}

/// A segment's way over the triangles: its events from its start on, and the pieces before,
/// between and after them.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct Trace {
    pub events: Vec<Event>,
    pub pieces: Vec<Piece>,
}

/// Where the walk stands: inside a triangle or along an edge, going on toward `b`.
#[derive(Debug, Clone, Copy)]
enum Step {
    Inside(u32),
    Along(u32, u32),
}

/// The triangles, with what the walk needs to find its way among them.
pub(crate) struct Net<'a> {
    points: &'a [Point],
    triangles: &'a [[u32; 3]],
    /// The triangle holding each edge, directed counterclockwise round it.
    left: HashMap<(u32, u32), u32>,
    /// The triangles at corner `v`: `around[starts[v]..starts[v + 1]]`.
    starts: Vec<usize>,
    around: Vec<u32>,
    /// Buckets over runs of triangles of about one size, each with the first of its run.
    buckets: Vec<(u32, Buckets)>,
}

/// Whether `q − p` and `s − r`, parallel, point the same way: exact, for the sign of a
/// difference of doubles is.
fn same_way(p: Point, q: Point, r: Point, s: Point) -> bool {
    let (d, e) = if p[0] != q[0] { (q[0] - p[0], s[0] - r[0]) } else { (q[1] - p[1], s[1] - r[1]) };
    e != 0.0 && (d > 0.0) == (e > 0.0)
}

impl<'a> Net<'a> {
    /// The net of `triangles` over `points`, the first `split` of them buckets apart from the
    /// rest; refused when a triangle is not counterclockwise or an edge is used twice one way.
    pub fn new(points: &'a [Point], triangles: &'a [[u32; 3]], split: usize) -> Result<Net<'a>, GeomError> {
        let refuse = |what: &str| Err(GeomError::Engine(format!("the leaves' triangles {what}")));
        let mut left = HashMap::with_capacity(3 * triangles.len());
        let mut starts = vec![0usize; points.len() + 1];
        for (k, &[a, b, c]) in triangles.iter().enumerate() {
            let (Some(&pa), Some(&pb), Some(&pc)) =
                (points.get(a as usize), points.get(b as usize), points.get(c as usize))
            else {
                return refuse("name a corner they do not hold");
            };
            if orientation(pa, pb, pc) != Greater {
                return refuse("hold one that is not counterclockwise");
            }
            for edge in [(a, b), (b, c), (c, a)] {
                if left.insert(edge, k as u32).is_some() {
                    return refuse("use an edge twice the same way");
                }
            }
            for v in [a, b, c] {
                if let Some(s) = starts.get_mut(v as usize + 1) {
                    *s += 1;
                }
            }
        }
        for v in 0..points.len() {
            let before = starts.get(v).copied().unwrap_or(0);
            if let Some(s) = starts.get_mut(v + 1) {
                *s += before;
            }
        }
        let mut fill = starts.clone();
        let mut around = vec![0u32; starts.last().copied().unwrap_or(0)];
        for (k, t) in triangles.iter().enumerate() {
            for &v in t {
                if let Some(slot) = fill.get_mut(v as usize)
                    && let Some(item) = around.get_mut(*slot)
                {
                    *item = k as u32;
                    *slot += 1;
                }
            }
        }
        let corners = |t: usize| -> Option<[Point; 3]> {
            let [a, b, c] = *triangles.get(t)?;
            Some([*points.get(a as usize)?, *points.get(b as usize)?, *points.get(c as usize)?])
        };
        let split = split.min(triangles.len());
        let buckets = vec![
            (0, Buckets::over(split, corners)),
            (split as u32, Buckets::over(triangles.len() - split, |t| corners(t + split))),
        ];
        Ok(Net { points, triangles, left, starts, around, buckets })
    }

    pub fn point(&self, v: u32) -> Option<Point> {
        self.points.get(v as usize).copied()
    }

    pub fn corners(&self, t: u32) -> Option<[u32; 3]> {
        self.triangles.get(t as usize).copied()
    }

    /// The corners of triangle `t` and where they lie.
    pub fn triangle(&self, t: u32) -> Option<([u32; 3], [Point; 3])> {
        let c = self.corners(t)?;
        Some((c, [self.point(c[0])?, self.point(c[1])?, self.point(c[2])?]))
    }

    /// The triangle holding the edge from `u` to `v` counterclockwise.
    pub fn left_of(&self, u: u32, v: u32) -> Option<u32> {
        self.left.get(&(u, v)).copied()
    }

    /// The triangles across each edge of `t`.
    pub fn neighbours(&self, t: u32) -> [Option<u32>; 3] {
        match self.corners(t) {
            Some([a, b, c]) => [self.left_of(b, a), self.left_of(c, b), self.left_of(a, c)],
            None => [None; 3],
        }
    }

    fn star(&self, v: u32) -> &[u32] {
        match (self.starts.get(v as usize), self.starts.get(v as usize + 1)) {
            (Some(&a), Some(&b)) => self.around.get(a..b).unwrap_or(&[]),
            _ => &[],
        }
    }

    /// Where `p` lies on the triangles, exactly.
    pub fn locate(&self, p: Point) -> Spot {
        for (first, buckets) in &self.buckets {
            for &k in buckets.near(p) {
                let Some(([a, b, c], [pa, pb, pc])) = self.triangle(first + k) else { continue };
                let sides = [orientation(pa, pb, p), orientation(pb, pc, p), orientation(pc, pa, p)];
                match sides {
                    _ if sides.contains(&Less) => continue,
                    [Greater, Greater, Greater] => return Spot::Inside(first + k),
                    [Equal, Greater, Greater] => return Spot::Edge(a, b),
                    [Greater, Equal, Greater] => return Spot::Edge(b, c),
                    [Greater, Greater, Equal] => return Spot::Edge(c, a),
                    // On two edges' lines, at the corner they share.
                    [Equal, Equal, _] => return Spot::Corner(b),
                    [_, Equal, Equal] => return Spot::Corner(c),
                    _ => return Spot::Corner(a),
                }
            }
        }
        Spot::Outside
    }

    /// The segment from `a` to `b` walked over the triangles; `None` when it leaves them, or
    /// when `a` and `b` are one point.
    pub fn trace(&self, a: Point, b: Point) -> Option<Trace> {
        if a == b {
            return None;
        }
        let mut step = match self.locate(a) {
            Spot::Inside(t) => Step::Inside(t),
            Spot::Edge(u, v) => self.off_edge(u, v, a, b)?,
            Spot::Corner(w) => self.off_corner(w, b)?,
            Spot::Outside => return None,
        };
        let mut trace = Trace::default();
        // A straight segment enters each triangle and passes each corner at most once.
        for _ in 0..=self.triangles.len() + self.points.len() {
            step = match step {
                Step::Inside(t) => {
                    trace.pieces.push(Piece::Inside(t));
                    let ([p, q, r], [pp, pq, pr]) = self.triangle(t)?;
                    if [orientation(pp, pq, b), orientation(pq, pr, b), orientation(pr, pp, b)]
                        .iter()
                        .all(|&s| s != Less)
                    {
                        return Some(trace);
                    }
                    let sides =
                        [(p, orientation(a, b, pp)), (q, orientation(a, b, pq)), (r, orientation(a, b, pr))];
                    let (event, next) = self.exit(sides, b)?;
                    trace.events.push(event);
                    next
                }
                Step::Along(u, v) => {
                    trace.pieces.push(Piece::Along(u, v));
                    let (pu, pv) = (self.point(u)?, self.point(v)?);
                    // On the edge's line: `b` ends the walk unless it lies past `v`.
                    if b == pv || !same_way(pu, pv, pv, b) {
                        return Some(trace);
                    }
                    trace.events.push(Event::Corner(v));
                    self.off_corner(v, b)?
                }
            };
        }
        None
    }

    /// How the segment leaves a triangle it passes through toward `b`, from the side of its
    /// line each corner lies on, counterclockwise: across the edge from a corner on its right
    /// to one on its left, or through a corner on it whose next two corners lie left then right.
    fn exit(&self, sides: [(u32, Ordering); 3], b: Point) -> Option<(Event, Step)> {
        let [x, y, z] = sides;
        for ((u, su), (v, sv), (_, sw)) in [(x, y, z), (y, z, x), (z, x, y)] {
            if su == Less && sv == Greater {
                return Some((Event::Crossing(u.min(v), u.max(v)), Step::Inside(self.left_of(v, u)?)));
            }
            if su == Equal && sv == Greater && sw == Less {
                return Some((Event::Corner(u), self.off_corner(u, b)?));
            }
        }
        None
    }

    /// Where the segment goes from a point inside the edge from `u` to `v`, toward `b`.
    fn off_edge(&self, u: u32, v: u32, at: Point, b: Point) -> Option<Step> {
        let (pu, pv) = (self.point(u)?, self.point(v)?);
        match orientation(pu, pv, b) {
            Greater => Some(Step::Inside(self.left_of(u, v)?)),
            Less => Some(Step::Inside(self.left_of(v, u)?)),
            Equal if same_way(pu, pv, at, b) => Some(Step::Along(u, v)),
            Equal => Some(Step::Along(v, u)),
        }
    }

    /// Where the segment goes from corner `w`, toward `b`: into the triangle whose angle at `w`
    /// holds its direction, or along an edge.
    fn off_corner(&self, w: u32, b: Point) -> Option<Step> {
        let pw = self.point(w)?;
        for &t in self.star(w) {
            let Some(([p, q, r], _)) = self.triangle(t) else { continue };
            let (x, y) = if p == w {
                (q, r)
            } else if q == w {
                (r, p)
            } else {
                (p, q)
            };
            let (px, py) = (self.point(x)?, self.point(y)?);
            let (sx, sy) = (orientation(pw, px, b), orientation(pw, py, b));
            if sx == Greater && sy == Less {
                return Some(Step::Inside(t));
            }
            if sx == Equal && same_way(pw, px, pw, b) {
                return Some(Step::Along(w, x));
            }
            if sy == Equal && same_way(pw, py, pw, b) {
                return Some(Step::Along(w, y));
            }
        }
        None
    }
}
