//! The mesh cut along the vertical planes through a set of grid lines, so that every field
//! bending only on those lines is linear on each triangle of the result: a leaf then cuts each
//! one along a straight segment, as a plane does, and the planar cut applies unchanged.
//!
//! Only triangles that a box of the plate and a band of heights touch are cut. A triangle's
//! edge is cut where it crosses each line, a point computed once from its lower-numbered end and
//! shared by every triangle on that edge, so the result stays closed. Inside, the triangle is
//! split by one line after another into convex cells; a new corner there is where a line meets
//! a cut made before, a vertex of the grid. Each point lies on a line's upper side when its level
//! is at least the line's: a triangle's corner by its coordinates, a point on its edge by its
//! place along that edge, a grid vertex by its exact indices, so every decision is the same
//! wherever it is taken. A cell is then fanned from its centre into triangles. A cell the rule
//! cannot split, its corners crossing a line more than twice, which only rounding at a grid
//! vertex lying on a mesh edge can cause, is left whole and counted.

use layercraft_geom::{Crossing, Grid, Lines, levels};
use layercraft_mesh::{Body, Mesh, Vertex};
use std::collections::HashMap;

/// Where a point of a triangle being split lies.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Kind {
    Corner,
    /// On the triangle's edge `edge`, the `rank`-th point along it from its lower-numbered end,
    /// where it crosses `crossing`'s line.
    Edge {
        edge: usize,
        rank: usize,
        family: usize,
        level: i64,
    },
    /// The grid vertex `(i, j)`.
    Node(i64, i64),
}

/// What an edge of a cell runs along: an edge of the triangle, or a grid line.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Support {
    Edge,
    Line(usize, i64),
}

/// A point of the triangle being split: its vertex in the body, place and kind.
#[derive(Debug, Clone, Copy)]
struct Point3 {
    id: u32,
    pos: Vertex,
    grid: [f64; 2],
    kind: Kind,
}

/// A cell: its points, as indices into the triangle's points, and what each edge runs along,
/// from that point to the next.
#[derive(Debug, Clone, Default)]
struct Cell {
    points: Vec<usize>,
    supports: Vec<Support>,
}

/// One edge of the triangle being split: its ends' corners, lower-numbered first, and the
/// crossings along it from there.
struct Side<'a> {
    from: usize,
    to: usize,
    crossings: &'a [(Crossing, u32)],
}

/// The side of a line a point lies on: 1 above, 0 on, -1 below.
type Class = i8;

fn lerp(a: Vertex, b: Vertex, t: f64) -> Vertex {
    [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])]
}

/// The grid vertex where two lines of different families meet.
fn node(a: (usize, i64), b: (usize, i64)) -> Option<(i64, i64)> {
    let (a, b) = if a.0 < b.0 { (a, b) } else { (b, a) };
    match (a.0, b.0) {
        (0, 1) => Some((a.1, b.1)),
        (0, 2) => Some((a.1, a.1 - b.1)),
        (1, 2) => Some((a.1 + b.1, a.1)),
        _ => None,
    }
}

/// The vertices of the refined body: the mesh's, then the new ones.
struct Out {
    vertices: Vec<Vertex>,
}

impl Out {
    /// A new vertex at `pos`, while indices last.
    fn add(&mut self, pos: Vertex) -> Option<u32> {
        let id = u32::try_from(self.vertices.len()).ok().filter(|&id| id < u32::MAX)?;
        self.vertices.push(pos);
        Some(id)
    }

    fn get(&self, id: u32) -> Option<Vertex> {
        self.vertices.get(id as usize).copied()
    }
}

/// The splitting of one triangle.
struct Split<'a> {
    points: Vec<Point3>,
    sides: [Side<'a>; 3],
    nodes: HashMap<(i64, i64), usize>,
}

impl Split<'_> {
    fn class(&self, at: usize, family: usize, level: i64) -> Class {
        let sign = |v: f64| if v >= level as f64 { 1 } else { -1 };
        let Some(p) = self.points.get(at) else { return 0 };
        match p.kind {
            Kind::Corner => levels(p.grid).get(family).map_or(0, |&v| sign(v)),
            Kind::Node(i, j) => {
                let v = [i, j, i - j].get(family).copied().unwrap_or(0);
                (v.cmp(&level)) as i8
            }
            Kind::Edge { edge, rank, family: f, level: l } => {
                if f == family {
                    return l.cmp(&level) as i8;
                }
                let Some(side) = self.sides.get(edge) else { return 0 };
                let corner = |k: usize| self.class(k, family, level);
                match side.crossings.iter().position(|(c, _)| c.family == family && c.level == level) {
                    Some(r) if rank < r => corner(side.from),
                    Some(_) => corner(side.to),
                    None => corner(side.from),
                }
            }
        }
    }

    /// The point where the cell's edge from `a` to `b`, along a grid line, meets the line
    /// `(family, level)`: a grid vertex, made once.
    fn meet(
        &mut self,
        a: usize,
        b: usize,
        along: Support,
        family: usize,
        level: i64,
        out: &mut Out,
    ) -> Option<usize> {
        let Support::Line(f, l) = along else { return None };
        let (i, j) = node((f, l), (family, level))?;
        if let Some(&k) = self.nodes.get(&(i, j)) {
            return Some(k);
        }
        let (pa, pb) = (self.points.get(a)?, self.points.get(b)?);
        let (la, lb) = (levels(pa.grid).get(family).copied()?, levels(pb.grid).get(family).copied()?);
        let t = if lb != la { ((level as f64 - la) / (lb - la)).clamp(0.0, 1.0) } else { 0.5 };
        let pos = lerp(pa.pos, pb.pos, t);
        let id = out.add(pos)?;
        self.points.push(Point3 { id, pos, grid: [i as f64, j as f64], kind: Kind::Node(i, j) });
        let k = self.points.len() - 1;
        self.nodes.insert((i, j), k);
        Some(k)
    }

    /// The cell split by the line `(family, level)` into its parts above and below; `None` when
    /// the rule cannot split it.
    #[allow(clippy::type_complexity)]
    fn split(
        &mut self,
        cell: &Cell,
        family: usize,
        level: i64,
        out: &mut Out,
    ) -> Option<(Option<Cell>, Option<Cell>)> {
        let m = cell.points.len();
        let classes: Vec<Class> = cell.points.iter().map(|&p| self.class(p, family, level)).collect();
        if !classes.contains(&-1) {
            return Some((Some(cell.clone()), None));
        }
        if !classes.contains(&1) {
            return Some((None, Some(cell.clone())));
        }
        // Rotated to start above the line: the run above, a crossing, the run below, a crossing.
        let shift = classes.iter().position(|&c| c == 1)?;
        let at = |k: usize| (k + shift) % m;
        let class = |k: usize| classes.get(at(k)).copied().unwrap_or(0);
        let first_below = (0..m).find(|&k| class(k) == -1)?;
        let last_below = (0..m).rev().find(|&k| class(k) == -1)?;
        if (first_below..=last_below).any(|k| class(k) == 1) {
            return None;
        }
        let last_above = (0..first_below).rev().find(|&k| class(k) == 1)?;
        let next_above = (last_below + 1..m).find(|&k| class(k) == 1).unwrap_or(m);
        let point = |k: usize| cell.points.get(at(k)).copied();
        let support = |k: usize| cell.supports.get(at(k)).copied();
        // A crossing is a point on the line between the runs, or a new one on the edge between.
        let mut cut = |from: usize, to: usize, this: &mut Self| -> Option<usize> {
            match to - from {
                1 => {
                    let along = support(from)?;
                    if along == Support::Edge {
                        return None;
                    }
                    this.meet(point(from)?, point(to % m)?, along, family, level, out)
                }
                2 => point(from + 1),
                _ => None,
            }
        };
        let down = cut(last_above, first_below, self)?;
        let up = cut(last_below, next_above, self)?;
        let line = Support::Line(family, level);
        let mut above = Cell::default();
        above.points.push(up);
        above.supports.push(support(next_above - 1)?);
        for k in next_above..m + last_above + 1 {
            above.points.push(point(k % m)?);
            above.supports.push(support(k % m)?);
        }
        if let Some(s) = above.supports.last_mut() {
            *s = support(last_above)?;
        }
        above.points.push(down);
        above.supports.push(line);
        let mut below = Cell::default();
        below.points.push(down);
        below.supports.push(support(first_below - 1)?);
        for k in first_below..=last_below {
            below.points.push(point(k)?);
            below.supports.push(support(k)?);
        }
        below.points.push(up);
        below.supports.push(line);
        Some((Some(above), Some(below)))
    }
}

/// The mesh with every triangle whose box meets the lines' box and whose heights meet
/// `heights` cut along the lines, and the number of cells left whole.
pub fn refine(mesh: &Mesh, grid: &Grid, lines: &Lines, heights: [f64; 2]) -> (Mesh, usize) {
    let mut failed = 0;
    let bodies = mesh.bodies.iter().map(|b| refine_body(b, grid, lines, heights, &mut failed)).collect();
    (Mesh { bodies }, failed)
}

fn refine_body(body: &Body, grid: &Grid, lines: &Lines, heights: [f64; 2], failed: &mut usize) -> Body {
    let coords: Vec<[f64; 2]> = body.vertices.iter().map(|v| grid.coords([v[0], v[1]])).collect();
    let [lu, lv, _] = lines.levels;
    let touched = |tri: &[u32; 3]| -> bool {
        let (Some(c), Some(v)) =
            (tri.iter().map(|&i| coords.get(i as usize)).collect::<Option<Vec<_>>>(), body.corners(tri))
        else {
            return false;
        };
        let range = |f: &dyn Fn(usize) -> f64| {
            (0..3).fold((f64::MAX, f64::MIN), |(lo, hi), k| (lo.min(f(k)), hi.max(f(k))))
        };
        let (u0, u1) = range(&|k| c.get(k).map_or(0.0, |p| p[0]));
        let (v0, v1) = range(&|k| c.get(k).map_or(0.0, |p| p[1]));
        let (z0, z1) = range(&|k| v.get(k).map_or(0.0, |p| p[2]));
        u1 >= lu[0] as f64
            && u0 <= lu[1] as f64
            && v1 >= lv[0] as f64
            && v0 <= lv[1] as f64
            && z1 >= heights[0]
            && z0 <= heights[1]
    };
    let cut: Vec<bool> = body.triangles.iter().map(touched).collect();
    let mut out = Out { vertices: body.vertices.clone() };
    // Each cut edge's crossings, from its lower-numbered end, with their vertices.
    let mut edges: HashMap<(u32, u32), Vec<(Crossing, u32)>> = HashMap::new();
    for (tri, _) in body.triangles.iter().zip(&cut).filter(|(_, c)| **c) {
        for k in 0..3 {
            let (Some(&a), Some(&b)) = (tri.get(k), tri.get((k + 1) % 3)) else { continue };
            let key = (a.min(b), a.max(b));
            if edges.contains_key(&key) {
                continue;
            }
            let (Some(&ca), Some(&cb), Some(&pa), Some(&pb)) = (
                coords.get(key.0 as usize),
                coords.get(key.1 as usize),
                body.vertices.get(key.0 as usize),
                body.vertices.get(key.1 as usize),
            ) else {
                continue;
            };
            let mut list = Vec::new();
            for c in lines.crossings(ca, cb) {
                let Some(id) = out.add(lerp(pa, pb, c.t)) else { break };
                list.push((c, id));
            }
            edges.insert(key, list);
        }
    }
    let mut triangles = Vec::with_capacity(body.triangles.len() * 2);
    for (tri, &inside) in body.triangles.iter().zip(&cut) {
        let key = |k: usize| -> Option<(u32, u32)> {
            let (a, b) = (*tri.get(k)?, *tri.get((k + 1) % 3)?);
            Some((a.min(b), a.max(b)))
        };
        let split_edges =
            (0..3).any(|k| key(k).is_some_and(|e| edges.get(&e).is_some_and(|l| !l.is_empty())));
        if !inside && !split_edges {
            triangles.push(*tri);
            continue;
        }
        let Some(corners) = body.corners(tri) else { continue };
        let empty: &[(Crossing, u32)] = &[];
        let side = |k: usize| -> Side<'_> {
            let (a, b) = (tri.get(k).copied().unwrap_or(0), tri.get((k + 1) % 3).copied().unwrap_or(0));
            let crossings = key(k).and_then(|e| edges.get(&e)).map_or(empty, |l| l.as_slice());
            if a <= b {
                Side { from: k, to: (k + 1) % 3, crossings }
            } else {
                Side { from: (k + 1) % 3, to: k, crossings }
            }
        };
        let mut split =
            Split { points: Vec::new(), sides: [side(0), side(1), side(2)], nodes: HashMap::new() };
        for (&id, pos) in tri.iter().zip(corners) {
            let grid = coords.get(id as usize).copied().unwrap_or([0.0; 2]);
            split.points.push(Point3 { id, pos, grid, kind: Kind::Corner });
        }
        // The boundary: each corner, then the crossings of the edge it starts, in its direction.
        let mut boundary = Cell::default();
        for k in 0..3 {
            boundary.points.push(k);
            boundary.supports.push(Support::Edge);
            let Some(s) = split.sides.get(k) else { continue };
            let ranks: Vec<usize> = if s.from == k {
                (0..s.crossings.len()).collect()
            } else {
                (0..s.crossings.len()).rev().collect()
            };
            for rank in ranks {
                let Some(&(c, id)) = s.crossings.get(rank) else { continue };
                let pos = out.get(id).unwrap_or([0.0; 3]);
                let grid = grid.coords([pos[0], pos[1]]);
                split.points.push(Point3 {
                    id,
                    pos,
                    grid,
                    kind: Kind::Edge { edge: k, rank, family: c.family, level: c.level },
                });
                boundary.points.push(split.points.len() - 1);
                boundary.supports.push(Support::Edge);
            }
        }
        let mut cells = vec![boundary];
        if inside {
            for (family, range) in lines.levels.iter().enumerate() {
                let corner_levels: Vec<f64> =
                    split.points.iter().take(3).filter_map(|p| levels(p.grid).get(family).copied()).collect();
                let lo = corner_levels.iter().copied().fold(f64::MAX, f64::min);
                let hi = corner_levels.iter().copied().fold(f64::MIN, f64::max);
                let first = (lo.floor() as i64).saturating_add(1).max(range[0]);
                let last = (hi.floor() as i64).min(range[1]);
                if last.saturating_sub(first) > 1 << 16 {
                    continue;
                }
                for level in first..=last {
                    let mut next = Vec::with_capacity(cells.len() + 1);
                    for cell in cells {
                        match split.split(&cell, family, level, &mut out) {
                            Some((above, below)) => next.extend(above.into_iter().chain(below)),
                            None => {
                                *failed += 1;
                                next.push(cell);
                            }
                        }
                    }
                    cells = next;
                }
            }
        }
        for cell in cells {
            let ids: Vec<u32> =
                cell.points.iter().filter_map(|&k| split.points.get(k).map(|p| p.id)).collect();
            match ids.as_slice() {
                [a, b, c] => triangles.push([*a, *b, *c]),
                _ if ids.len() > 3 => {
                    let n = cell.points.len() as f64;
                    let centre =
                        cell.points.iter().filter_map(|&k| split.points.get(k)).fold([0.0; 3], |c, p| {
                            [c[0] + p.pos[0] / n, c[1] + p.pos[1] / n, c[2] + p.pos[2] / n]
                        });
                    let Some(id) = out.add(centre) else { continue };
                    for (k, &a) in ids.iter().enumerate() {
                        let b = ids.get((k + 1) % ids.len()).copied().unwrap_or(a);
                        triangles.push([id, a, b]);
                    }
                }
                _ => {}
            }
        }
    }
    Body { name: body.name.clone(), vertices: out.vertices, triangles }
}
