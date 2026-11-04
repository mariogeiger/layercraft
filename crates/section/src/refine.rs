//! The mesh cut along the leaves' triangles, so that every leaf is linear on each triangle of
//! the result: a leaf then cuts each one along a straight segment, as a plane does, and the
//! planar cut applies unchanged.
//!
//! Only triangles of the mesh over the leaves' bent triangles and within the heights the bent
//! leaves reach are cut. Each of their edges is walked over the leaves' triangles exactly
//! (`trace`) and cut where it passes a corner or crosses an edge, each point made once, from the
//! edge's lower-numbered end, for every triangle sharing it, so the result stays closed. A
//! triangle of the mesh, its edges so cut, is the union of its parts over the leaves'
//! triangles: over one, the runs of its boundary that lie in it, joined counterclockwise along
//! that triangle's edges through its corners between, or the whole triangle when it lies inside;
//! the triangles inside are found from those its boundary passes through, neighbour to
//! neighbour. Each part is fanned from its centre. A steep triangle, whose shadow is a segment,
//! and one whose parts do not add up, which only a gap in the leaves' triangles can cause and
//! which is counted, are fanned from their centre through the points of their edges, as are
//! the triangles beside them that share a cut edge. Each vertex keeps its place on the leaves'
//! triangles, so that a leaf's value there is the sum of its values at their corners.

use crate::trace::{Event, Net, Piece, Spot, Trace};
use layercraft_geom::{GRID_MM, Point, orientation};
use layercraft_mesh::{Body, Mesh, Vertex};
use std::cmp::Ordering::{Equal, Greater, Less};
use std::collections::{HashMap, HashSet, VecDeque};

/// Where a vertex lies on the leaves' triangles: a leaf's value there is the sum of its values
/// at `corners` by `weights`; nowhere, all weights zero, the leaves are flat.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Place {
    corners: [u32; 3],
    weights: [f64; 3],
}

impl Place {
    fn at(corner: u32) -> Place {
        Place { corners: [corner; 3], weights: [1.0, 0.0, 0.0] }
    }

    fn between(u: u32, v: u32, s: f64) -> Place {
        let s = if s.is_finite() { s.clamp(0.0, 1.0) } else { 0.5 };
        Place { corners: [u, v, v], weights: [1.0 - s, s, 0.0] }
    }

    fn inside(corners: [u32; 3], l: [f64; 3]) -> Place {
        let l = l.map(|x| if x.is_finite() { x.max(0.0) } else { 0.0 });
        let sum = l[0] + l[1] + l[2];
        if sum > 0.0 { Place { corners, weights: l.map(|x| x / sum) } } else { Place::at(corners[0]) }
    }

    /// The corners the place is weighed from.
    #[cfg(test)]
    pub fn named(&self) -> impl Iterator<Item = u32> + '_ {
        self.corners.iter().zip(self.weights).filter(|&(_, w)| w != 0.0).map(|(&c, _)| c)
    }

    /// The value here of a field linear on the triangles, from its values at their corners.
    pub fn value(&self, values: &[f64]) -> f64 {
        self.corners
            .iter()
            .zip(self.weights)
            .filter(|&(_, w)| w != 0.0)
            .map(|(&c, w)| w * values.get(c as usize).copied().unwrap_or(0.0))
            .sum()
    }
}

/// The refined mesh: its bodies, in the mesh's order, each vertex's place, and how many of the
/// mesh's triangles were fanned whole where they should have been cut.
#[derive(Debug, Clone, Default)]
pub(crate) struct Refined {
    pub mesh: Mesh,
    pub places: Vec<Vec<Place>>,
    pub unsplit: usize,
}

/// A vertex of a part: one of the body's, or a corner of the leaves' triangles strictly inside
/// the mesh's triangle, made for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Node {
    Mesh(u32),
    Inner(u32),
}

/// An edge's walk, and the vertex made at each of its events.
#[derive(Debug, Clone)]
struct Cut {
    trace: Trace,
    ids: Vec<u32>,
}

fn flat(v: Vertex) -> Point {
    [v[0], v[1]]
}

fn lerp(a: Vertex, b: Vertex, t: f64) -> Vertex {
    [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])]
}

/// How far along from `a` to `b` the point of their line nearest `p` lies, from 0 to 1.
fn along(a: Point, b: Point, p: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l2 = d[0] * d[0] + d[1] * d[1];
    if l2 > 0.0 { (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0.0, 1.0) } else { 0.0 }
}

/// How far along each segment two crossing segments cross, from 0 to 1.
fn crossing(a: Point, b: Point, u: Point, v: Point) -> (f64, f64) {
    let (d, e, w) = ([b[0] - a[0], b[1] - a[1]], [v[0] - u[0], v[1] - u[1]], [u[0] - a[0], u[1] - a[1]]);
    let det = d[0] * e[1] - d[1] * e[0];
    let (t, s) = ((w[0] * e[1] - w[1] * e[0]) / det, (w[0] * d[1] - w[1] * d[0]) / det);
    if t.is_finite() && s.is_finite() { (t.clamp(0.0, 1.0), s.clamp(0.0, 1.0)) } else { (0.5, 0.5) }
}

/// The barycentric coordinates of `p` in the triangle, when it has an area.
fn barycentric([a, b, c]: [Point; 3], p: Point) -> Option<[f64; 3]> {
    let det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
    let l1 = ((p[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (p[1] - a[1])) / det;
    let l2 = ((b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1])) / det;
    (l1.is_finite() && l2.is_finite()).then_some([1.0 - l1 - l2, l1, l2])
}

/// The place of the point `p`, lying at `spot`.
fn place(net: &Net, spot: Spot, p: Point) -> Place {
    match spot {
        Spot::Corner(w) => Place::at(w),
        Spot::Edge(u, v) => match (net.point(u), net.point(v)) {
            (Some(pu), Some(pv)) => Place::between(u, v, along(pu, pv, p)),
            _ => Place::default(),
        },
        Spot::Inside(t) => match net.triangle(t) {
            Some((c, at)) => barycentric(at, p).map_or(Place::at(c[0]), |l| Place::inside(c, l)),
            None => Place::default(),
        },
        Spot::Outside => Place::default(),
    }
}

/// Whether the triangle's box meets `area` on the plate and `heights` up.
fn meets([a, b, c]: [Vertex; 3], area: [Point; 2], heights: [f64; 2]) -> bool {
    let lo = [a[0].min(b[0]).min(c[0]), a[1].min(b[1]).min(c[1]), a[2].min(b[2]).min(c[2])];
    let hi = [a[0].max(b[0]).max(c[0]), a[1].max(b[1]).max(c[1]), a[2].max(b[2]).max(c[2])];
    let [[x0, y0], [x1, y1]] = area;
    hi[2] >= heights[0] && lo[2] <= heights[1] && hi[0] >= x0 && lo[0] <= x1 && hi[1] >= y0 && lo[1] <= y1
}

/// The mesh with its triangles over `area`, a box of the plate, and within `heights` cut along
/// the triangles of `net`.
pub(crate) fn refine(mesh: &Mesh, net: &Net, area: [Point; 2], heights: [f64; 2]) -> Refined {
    let mut refined = Refined {
        mesh: Mesh { bodies: Vec::new(), materials: mesh.materials.clone() },
        ..Refined::default()
    };
    for body in &mesh.bodies {
        let mut cutter = Cutter::new(body, net);
        let (body, places) = cutter.run(body, area, heights);
        refined.mesh.bodies.push(body);
        refined.places.push(places);
        refined.unsplit += cutter.unsplit;
    }
    refined
}

/// The cutting of one body: its vertices so far, with their places, and its edges' cuts.
struct Cutter<'a> {
    net: &'a Net<'a>,
    vertices: Vec<Vertex>,
    places: Vec<Place>,
    /// Where each of the body's own vertices lies, for the corners of the triangles cut.
    spots: Vec<Spot>,
    edges: HashMap<(u32, u32), Option<Cut>>,
    triangles: Vec<[u32; 3]>,
    unsplit: usize,
}

impl<'a> Cutter<'a> {
    fn new(body: &Body, net: &'a Net<'a>) -> Cutter<'a> {
        Cutter {
            net,
            vertices: body.vertices.clone(),
            places: vec![Place::default(); body.vertices.len()],
            spots: vec![Spot::Outside; body.vertices.len()],
            edges: HashMap::new(),
            triangles: Vec::with_capacity(body.triangles.len()),
            unsplit: 0,
        }
    }

    fn run(&mut self, body: &Body, area: [Point; 2], heights: [f64; 2]) -> (Body, Vec<Place>) {
        let chosen: Vec<bool> =
            body.triangles.iter().map(|t| body.corners(t).is_some_and(|c| meets(c, area, heights))).collect();
        let mut located = vec![false; body.vertices.len()];
        for (tri, _) in body.triangles.iter().zip(&chosen).filter(|(_, c)| **c) {
            for &i in tri {
                let Some(seen @ false) = located.get_mut(i as usize) else { continue };
                *seen = true;
                let Some(p) = body.vertices.get(i as usize).map(|&v| flat(v)) else { continue };
                let spot = self.net.locate(p);
                if let (Some(s), Some(pl)) = (self.spots.get_mut(i as usize), self.places.get_mut(i as usize))
                {
                    (*s, *pl) = (spot, place(self.net, spot, p));
                }
            }
            for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let key = (a.min(b), a.max(b));
                if !self.edges.contains_key(&key) {
                    let cut = self.cut(key.0, key.1);
                    self.edges.insert(key, cut);
                }
            }
        }
        for (tri, &chosen) in body.triangles.iter().zip(&chosen) {
            if chosen && !matches!(self.turn(*tri), Some(Equal) | None) {
                if self.split(*tri) {
                    continue;
                }
                self.unsplit += 1;
            }
            self.fan(*tri);
        }
        let out = Body {
            name: body.name.clone(),
            vertices: std::mem::take(&mut self.vertices),
            triangles: std::mem::take(&mut self.triangles),
            material: body.material,
        };
        (out, std::mem::take(&mut self.places))
    }

    fn add(&mut self, at: Vertex, place: Place) -> Option<u32> {
        let id = u32::try_from(self.vertices.len()).ok().filter(|&id| id < u32::MAX)?;
        self.vertices.push(at);
        self.places.push(place);
        Some(id)
    }

    fn vertex(&self, id: u32) -> Option<Vertex> {
        self.vertices.get(id as usize).copied()
    }

    /// The side the triangle's shadow turns: counterclockwise, clockwise, or none, steep: its
    /// shadow no wider than the lattice's step, a segment as far as the regions can tell.
    fn turn(&self, [a, b, c]: [u32; 3]) -> Option<std::cmp::Ordering> {
        let (p, q, r) = (flat(self.vertex(a)?), flat(self.vertex(b)?), flat(self.vertex(c)?));
        let twice = (q[0] - p[0]) * (r[1] - p[1]) - (r[0] - p[0]) * (q[1] - p[1]);
        let longest = [(p, q), (q, r), (r, p)]
            .iter()
            .map(|(u, v)| (u[0] - v[0]).hypot(u[1] - v[1]))
            .fold(0.0, f64::max);
        if twice.abs() <= GRID_MM * longest {
            return Some(Equal);
        }
        Some(orientation(p, q, r))
    }

    /// The edge from `a` to `b`, `a` the lower, walked and cut; `None` when steep, or when the
    /// walk leaves the leaves' triangles.
    fn cut(&mut self, a: u32, b: u32) -> Option<Cut> {
        let (va, vb) = (self.vertex(a)?, self.vertex(b)?);
        let (pa, pb) = (flat(va), flat(vb));
        let trace = self.net.trace(pa, pb)?;
        let mut ids = Vec::with_capacity(trace.events.len());
        for event in &trace.events {
            let (at, place) = match *event {
                Event::Corner(w) => {
                    let pw = self.net.point(w)?;
                    let s = along(pa, pb, pw);
                    ([pw[0], pw[1], va[2] + s * (vb[2] - va[2])], Place::at(w))
                }
                Event::Crossing(u, v) => {
                    let (t, s) = crossing(pa, pb, self.net.point(u)?, self.net.point(v)?);
                    (lerp(va, vb, t), Place::between(u, v, s))
                }
            };
            ids.push(self.add(at, place)?);
        }
        Some(Cut { trace, ids })
    }

    /// The cut points of the edge from `c` to `d` in that direction, with their events.
    fn points(&self, c: u32, d: u32) -> Vec<(u32, Event)> {
        let Some(Some(cut)) = self.edges.get(&(c.min(d), c.max(d))) else { return Vec::new() };
        let mut out: Vec<(u32, Event)> =
            cut.ids.iter().copied().zip(cut.trace.events.iter().copied()).collect();
        if c > d {
            out.reverse();
        }
        out
    }

    /// The triangle fanned from its centre through the cut points of its edges, or kept whole
    /// when it has none.
    fn fan(&mut self, tri: [u32; 3]) {
        let [a, b, c] = tri;
        let mut ring = Vec::with_capacity(3);
        for (p, q) in [(a, b), (b, c), (c, a)] {
            ring.push(p);
            ring.extend(self.points(p, q).into_iter().map(|(id, _)| id));
        }
        if ring.len() == 3 {
            self.triangles.push(tri);
            return;
        }
        let at: Vec<Vertex> = ring.iter().filter_map(|&id| self.vertex(id)).collect();
        let n = at.len() as f64;
        let centre = at.iter().fold([0.0; 3], |s, v| [s[0] + v[0] / n, s[1] + v[1] / n, s[2] + v[2] / n]);
        let spot = self.net.locate(flat(centre));
        let Some(id) = self.add(centre, place(self.net, spot, flat(centre))) else {
            self.triangles.push(tri);
            return;
        };
        for (k, &p) in ring.iter().enumerate() {
            let q = ring.get(k + 1).or(ring.first()).copied().unwrap_or(p);
            self.triangles.push([id, p, q]);
        }
    }

    /// The triangle cut into its parts over the leaves' triangles; whether its parts add up.
    fn split(&mut self, tri: [u32; 3]) -> bool {
        let [a, b, c] = tri;
        let (m, reversed) = match self.turn(tri) {
            Some(Greater) => ([a, b, c], false),
            Some(Less) => ([a, c, b], true),
            _ => return false,
        };
        let Some(parts) = self.parts(m) else { return false };
        let saved = (self.vertices.len(), self.triangles.len());
        if self.lay(m, parts, reversed).is_none() {
            self.vertices.truncate(saved.0);
            self.places.truncate(saved.0);
            self.triangles.truncate(saved.1);
            return false;
        }
        true
    }

    /// The parts of the counterclockwise triangle `m` over the leaves' triangles, each with
    /// the triangle it lies over and its vertices counterclockwise; `None` when they do not add
    /// up.
    fn parts(&self, m: [u32; 3]) -> Option<Vec<(u32, Vec<Node>)>> {
        let net = self.net;
        let [m0, m1, m2] = m;
        let corners = [flat(self.vertex(m0)?), flat(self.vertex(m1)?), flat(self.vertex(m2)?)];
        // The boundary counterclockwise: each point, where it lies, and the triangle holding
        // the piece from it to the next.
        let (mut nodes, mut spots, mut holders) = (Vec::new(), Vec::new(), Vec::new());
        for (c, d) in [(m0, m1), (m1, m2), (m2, m0)] {
            let Some(Some(cut)) = self.edges.get(&(c.min(d), c.max(d))) else { return None };
            let n = cut.trace.events.len();
            if cut.trace.pieces.len() != n + 1 || cut.ids.len() != n {
                return None;
            }
            nodes.push(c);
            spots.push(*self.spots.get(c as usize)?);
            for j in 0..=n {
                let piece = if c < d { cut.trace.pieces.get(j)? } else { cut.trace.pieces.get(n - j)? };
                holders.push(match *piece {
                    Piece::Inside(t) => t,
                    Piece::Along(u, v) if c < d => net.left_of(u, v)?,
                    Piece::Along(u, v) => net.left_of(v, u)?,
                });
                if j < n {
                    let k = if c < d { j } else { n - 1 - j };
                    nodes.push(*cut.ids.get(k)?);
                    spots.push(match *cut.trace.events.get(k)? {
                        Event::Corner(w) => Spot::Corner(w),
                        Event::Crossing(u, v) => Spot::Edge(u, v),
                    });
                }
            }
        }
        let count = nodes.len();
        let Some(start) = (0..count).find(|&i| holders.get(i) != holders.get((i + count - 1) % count)) else {
            // The whole boundary over one triangle: the triangle of the mesh lies inside it.
            return Some(vec![(*holders.first()?, nodes.into_iter().map(Node::Mesh).collect())]);
        };
        // The runs of boundary over each triangle, `(first piece, pieces)`, counterclockwise.
        let mut runs: Vec<(u32, Vec<(usize, usize)>)> = Vec::new();
        let mut index: HashMap<u32, usize> = HashMap::new();
        let mut j = 0;
        while j < count {
            let first = (start + j) % count;
            let t = *holders.get(first)?;
            let mut len = 1;
            while j + len < count && holders.get((first + len) % count) == Some(&t) {
                len += 1;
            }
            let k = *index.entry(t).or_insert_with(|| {
                runs.push((t, Vec::new()));
                runs.len() - 1
            });
            runs.get_mut(k)?.1.push((first, len));
            j += len;
        }
        // The corners of the leaves' triangles on the boundary, and the side of each edge of
        // `m` the others lie on.
        let on_boundary: HashMap<u32, u32> = nodes
            .iter()
            .zip(&spots)
            .filter_map(|(&id, s)| if let Spot::Corner(w) = s { Some((*w, id)) } else { None })
            .collect();
        let [p0, p1, p2] = corners;
        let sides = |w: u32| -> Option<[std::cmp::Ordering; 3]> {
            let p = net.point(w)?;
            Some([orientation(p0, p1, p), orientation(p1, p2, p), orientation(p2, p0, p)])
        };
        let node = |w: u32| -> Option<Node> {
            if let Some(&id) = on_boundary.get(&w) {
                return Some(Node::Mesh(id));
            }
            (sides(w)? == [Greater; 3]).then_some(Node::Inner(w))
        };
        let mut parts = Vec::with_capacity(runs.len());
        for (t, list) in &runs {
            let (tc, _) = net.triangle(*t)?;
            // Where a point of the boundary lies round `t`: corner `k` at `2k`, inside the edge
            // from corner `k` at `2k + 1`.
            let round = |spot: Spot| -> Option<usize> {
                let [a, b, c] = tc;
                match spot {
                    Spot::Corner(w) => [a, b, c].iter().position(|&x| x == w).map(|k| 2 * k),
                    Spot::Edge(u, v) => [(a, b), (b, c), (c, a)]
                        .iter()
                        .position(|&(x, y)| (x, y) == (u, v) || (y, x) == (u, v))
                        .map(|k| 2 * k + 1),
                    _ => None,
                }
            };
            let mut part = Vec::new();
            for (r, &(first, len)) in list.iter().enumerate() {
                for i in 0..=len {
                    part.push(Node::Mesh(*nodes.get((first + i) % count)?));
                }
                let next = list.get((r + 1) % list.len())?.0;
                let (from, to) = (round(*spots.get((first + len) % count)?)?, round(*spots.get(next)?)?);
                // Round `t` counterclockwise from where the boundary leaves it to where it
                // comes back, through the corners between.
                if from != to {
                    let mut p = (from + 1) % 6;
                    while p != to {
                        if p % 2 == 0 {
                            part.push(node(*tc.get(p / 2)?)?);
                        }
                        p = (p + 1) % 6;
                    }
                }
            }
            part.dedup();
            if part.len() > 1 && part.first() == part.last() {
                part.pop();
            }
            parts.push((*t, part));
        }
        // The triangles inside, from the boundary's in, neighbour to neighbour.
        let mut seen: HashSet<u32> = runs.iter().map(|(t, _)| *t).collect();
        let mut queue: VecDeque<u32> = runs.iter().map(|(t, _)| *t).collect();
        while let Some(t) = queue.pop_front() {
            for next in net.neighbours(t).into_iter().flatten() {
                if !seen.insert(next) {
                    continue;
                }
                let (tc, _) = net.triangle(next)?;
                let mut inside = true;
                for &w in &tc {
                    inside &= !sides(w)?.contains(&Less);
                }
                if inside {
                    parts.push((next, tc.iter().map(|&w| node(w)).collect::<Option<Vec<Node>>>()?));
                    queue.push_back(next);
                }
            }
        }
        Some(parts)
    }

    /// The parts of the counterclockwise triangle `m` laid as triangles, each fanned from its
    /// centre, turned back clockwise when `m` was.
    fn lay(&mut self, m: [u32; 3], parts: Vec<(u32, Vec<Node>)>, reversed: bool) -> Option<()> {
        let at = [self.vertex(m[0])?, self.vertex(m[1])?, self.vertex(m[2])?];
        let shadow = at.map(flat);
        let mut inner: HashMap<u32, u32> = HashMap::new();
        let push = |this: &mut Self, [p, q, r]: [u32; 3]| {
            this.triangles.push(if reversed { [p, r, q] } else { [p, q, r] });
        };
        for (t, part) in parts {
            let mut ids = Vec::with_capacity(part.len());
            for node in part {
                ids.push(match node {
                    Node::Mesh(id) => id,
                    Node::Inner(w) => match inner.get(&w) {
                        Some(&id) => id,
                        None => {
                            let pw = self.net.point(w)?;
                            let l = barycentric(shadow, pw)?.map(|x| x.clamp(0.0, 1.0));
                            let sum = l[0] + l[1] + l[2];
                            let z = (l[0] * at[0][2] + l[1] * at[1][2] + l[2] * at[2][2]) / sum;
                            let id = self.add(
                                [pw[0], pw[1], if z.is_finite() { z } else { at[0][2] }],
                                Place::at(w),
                            )?;
                            inner.insert(w, id);
                            id
                        }
                    },
                });
            }
            match ids.as_slice() {
                [p, q, r] => push(self, [*p, *q, *r]),
                _ if ids.len() > 3 => {
                    let points: Vec<Vertex> = ids.iter().map(|&id| self.vertex(id)).collect::<Option<_>>()?;
                    let n = points.len() as f64;
                    let centre = points
                        .iter()
                        .fold([0.0; 3], |s, v| [s[0] + v[0] / n, s[1] + v[1] / n, s[2] + v[2] / n]);
                    let place = match self.net.triangle(t) {
                        Some((c, corners)) => barycentric(corners, flat(centre))
                            .map_or(Place::at(c[0]), |l| Place::inside(c, l)),
                        None => return None,
                    };
                    let id = self.add(centre, place)?;
                    for (k, &p) in ids.iter().enumerate() {
                        let q = ids.get(k + 1).or(ids.first()).copied()?;
                        push(self, [id, p, q]);
                    }
                }
                _ => {}
            }
        }
        Some(())
    }
}
