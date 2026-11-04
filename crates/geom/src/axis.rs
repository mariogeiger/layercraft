//! The chordal axis of a region: the centre lines of its strips, with their thickness.
//!
//! A constrained Delaunay triangulation cuts the region into triangles: the contours' edges
//! stay, and every other edge, a chord, joins two points of the boundary. Where two triangles'
//! four corners lie on one circle, as along an arc, the chord between them is a tie that the
//! lattice's rounding breaks, as the plate lies: such triangles make one cell. Across a strip the
//! chords between cells run from side to side, and their midpoints lie on its centre line:
//! exactly between parallel sides, to second order where the sides turn or part. A cell with two
//! chords joins their midpoints; one with more, a junction, joins them at the mean of its
//! corners; one with a single chord, a strip's end, stops at that chord's midpoint (Prasad's
//! chordal axis transform, without the end's centroid, which lies off the centre line). At each
//! point the strip is twice as thick as the point is far from the boundary, found along the
//! boundary from the corners of the point's chord or cell. A branch from a junction to an end
//! within the junction's disc only reaches into a corner, and goes; each end left runs on
//! straight to the boundary, as thick as its strip. A shape no longer than twice as thick as it
//! is is no strip: its centre line is its chord along its own axis (`Moments`), the part's own
//! frame choosing among the lines a symmetric shape leaves open.

use crate::ops::{from_lattice, region_to_lattice};
use crate::{
    Contour, GeomError, Moments, Point, RESOLUTION_MM, Region, check_points, check_value, edges, encloses,
    segment_distance,
};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::IntOverlayOptions;
use i_overlay::core::simplify::Simplify;
use i_triangle::int::unchecked::IntUncheckedTriangulatable;
use std::collections::{HashMap, HashSet};

/// A centre line of a region and how thick the region is along it.
#[derive(Debug, Clone, PartialEq)]
pub struct Centreline {
    pub points: Vec<Point>,
    /// The region's thickness at each point.
    pub thickness: Vec<f64>,
    /// Whether it runs round a loop, its last point back to its first.
    pub closed: bool,
}

/// The most pieces one edge is cut into, and the most edges walked from one corner.
const MAX_PIECES: f64 = 1e6;
const MAX_WALK: usize = 100_000;

/// The vertices of a closed contour that keep it within `tolerance` of itself, by Douglas and
/// Peucker: between two kept vertices, the farthest one from their chord is kept when it strays
/// more than `tolerance`.
fn simplified(contour: &Contour, tolerance: f64) -> Contour {
    let n = contour.len();
    let at = |i: usize| contour.get(i % n.max(1)).copied().unwrap_or([0.0, 0.0]);
    if n <= 3 || tolerance <= 0.0 {
        return contour.clone();
    }
    // Split at the vertex farthest from the vertices' mean and the one farthest from that, which
    // the contour places, not where its list starts; then halve each side while it strays.
    let mean = contour.iter().fold([0.0, 0.0], |m, p| [m[0] + p[0] / n as f64, m[1] + p[1] / n as f64]);
    let farthest = |from: Point| {
        let d = |i: usize| (at(i)[0] - from[0]).hypot(at(i)[1] - from[1]);
        (0..n).max_by(|&a, &b| d(a).total_cmp(&d(b)))
    };
    let Some(first) = farthest(mean) else { return contour.clone() };
    let Some(second) = farthest(at(first)).filter(|&i| i != first) else { return contour.clone() };
    let (lo, hi) = (first.min(second), first.max(second));
    let mut keep = vec![false; n];
    let mut spans = vec![(lo, hi), (hi, lo + n)];
    while let Some((a, b)) = spans.pop() {
        if let Some(k) = keep.get_mut(a % n) {
            *k = true;
        }
        let worst =
            (a + 1..b).map(|i| (i, segment_distance(at(i), at(a), at(b)))).max_by(|x, y| x.1.total_cmp(&y.1));
        if let Some((i, _)) = worst.filter(|&(_, d)| d > tolerance) {
            spans.push((a, i));
            spans.push((i, b));
        }
    }
    (0..n).filter(|&i| keep.get(i).copied().unwrap_or(false)).map(at).collect()
}

/// The contours of `region` within `tolerance` of themselves, with points along every edge at
/// most `step` apart.
fn sampled(region: &Region, step: f64, tolerance: f64) -> Region {
    let sample = |contour: &Contour| -> Contour {
        let contour = simplified(contour, tolerance);
        let mut out = Vec::with_capacity(contour.len());
        for (a, b) in edges(&contour) {
            let pieces = ((a[0] - b[0]).hypot(a[1] - b[1]) / step).ceil().clamp(1.0, MAX_PIECES) as usize;
            out.extend((0..pieces).map(|k| {
                let t = k as f64 / pieces as f64;
                [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
            }));
        }
        out
    };
    region.iter().map(|shape| shape.iter().map(sample).collect()).collect()
}

/// The boundary of the triangulation: its points and each point's neighbours along it.
struct Boundary {
    points: Vec<Point>,
    next: HashMap<usize, Vec<usize>>,
}

impl Boundary {
    fn at(&self, i: usize) -> Point {
        self.points.get(i).copied().unwrap_or([0.0, 0.0])
    }

    /// The distance from `p` to the boundary, walked from each corner `from` as far as twice its
    /// distance to `p`, beyond which no nearer point of a smooth boundary lies.
    fn distance(&self, p: Point, from: &[usize]) -> f64 {
        let mut best = f64::INFINITY;
        for &s in from {
            let q = self.at(s);
            let budget = 2.0 * (q[0] - p[0]).hypot(q[1] - p[1]);
            for &first in self.next.get(&s).into_iter().flatten() {
                let (mut prev, mut cur, mut walked) = (s, first, 0.0);
                for _ in 0..MAX_WALK {
                    let (a, b) = (self.at(prev), self.at(cur));
                    best = best.min(segment_distance(p, a, b));
                    walked += (b[0] - a[0]).hypot(b[1] - a[1]);
                    let next = self.next.get(&cur).and_then(|l| l.iter().find(|&&m| m != prev)).copied();
                    match next {
                        Some(n) if walked <= budget && cur != s => (prev, cur) = (cur, n),
                        _ => break,
                    }
                }
            }
        }
        best
    }

    /// How far a ray from `p` along the unit `d` runs before it meets the boundary, if within
    /// `reach`: the edges walked from each corner `from` as far as twice its distance to `p`
    /// and `reach` together, beyond which no edge within `reach` of `p` lies.
    fn hit(&self, p: Point, d: [f64; 2], from: &[usize], reach: f64) -> Option<f64> {
        let cross = |a: [f64; 2], b: [f64; 2]| a[0] * b[1] - a[1] * b[0];
        let mut best: Option<f64> = None;
        for &s in from {
            let q = self.at(s);
            let budget = 2.0 * ((q[0] - p[0]).hypot(q[1] - p[1]) + reach);
            for &first in self.next.get(&s).into_iter().flatten() {
                let (mut prev, mut cur, mut walked) = (s, first, 0.0);
                for _ in 0..MAX_WALK {
                    let (a, b) = (self.at(prev), self.at(cur));
                    // p + u·d = a + v·(b − a), with u ≥ 0 and v in [0, 1].
                    let (e, f) = ([b[0] - a[0], b[1] - a[1]], [a[0] - p[0], a[1] - p[1]]);
                    let den = cross(d, e);
                    if den.abs() > 0.0 {
                        let (u, v) = (cross(f, e) / den, cross(f, d) / den);
                        if u >= 0.0 && u <= reach && (0.0..=1.0).contains(&v) && best.is_none_or(|x| u < x) {
                            best = Some(u);
                        }
                    }
                    walked += e[0].hypot(e[1]);
                    let next = self.next.get(&cur).and_then(|l| l.iter().find(|&&m| m != prev)).copied();
                    match next {
                        Some(n) if walked <= budget && cur != s => (prev, cur) = (cur, n),
                        _ => break,
                    }
                }
            }
        }
        best
    }
}

/// A point of the axis and the corners its distance to the boundary is sought from.
struct Node {
    at: Point,
    corners: Vec<usize>,
}

/// The centre lines of `region`'s strips, each with the region's thickness along it. The
/// boundary is taken to within `tolerance`, so that no sliver of nearly aligned points makes a
/// strip of its own, and sampled at most `step` apart: a strip's sides need points closer than
/// it is thick. `axis_at` gives the axis of the part's own frame at a point, a unit vector.
pub fn chordal_axis(
    region: &Region,
    step: f64,
    tolerance: f64,
    axis_at: &dyn Fn(Point) -> [f64; 2],
) -> Result<Vec<Centreline>, GeomError> {
    check_points(region.iter().flatten())?;
    check_value(step)?;
    check_value(tolerance)?;
    if step <= 0.0 {
        return Ok(Vec::new());
    }
    // Normalised keeping every sampled point, which the triangulation then keeps too.
    let shapes = region_to_lattice(&sampled(region, step, tolerance))
        .simplify(FillRule::NonZero, IntOverlayOptions::keep_all_points());
    let mesh = shapes.uncheck_triangulate().into_delaunay();
    let n = mesh.triangles.len();
    let mut boundary =
        Boundary { points: mesh.points.iter().map(from_lattice).collect(), next: HashMap::new() };
    let corner =
        |t: usize, k: usize| mesh.triangles.get(t).and_then(|t| t.vertices.get(k % 3)).map(|v| v.index);
    let across = |t: usize, k: usize| {
        mesh.triangles.get(t).and_then(|t| t.neighbors.get(k % 3)).copied().filter(|&m| m < n)
    };
    // The cells: triangles joined across each chord whose two triangles' corners lie on one
    // circle, to `RESOLUTION_MM`.
    let circle = |t: usize| -> Option<(Point, f64)> {
        circumcircle(boundary.at(corner(t, 0)?), boundary.at(corner(t, 1)?), boundary.at(corner(t, 2)?))
    };
    let on = |p: Point, c: Option<(Point, f64)>| {
        c.is_some_and(|(o, r)| ((p[0] - o[0]).hypot(p[1] - o[1]) - r).abs() <= RESOLUTION_MM)
    };
    let mut cell: Vec<usize> = (0..n).collect();
    for t in 0..n {
        for k in 0..3 {
            let Some(u) = across(t, k).filter(|&u| u > t) else { continue };
            let (Some(i), Some(j), Some(q)) = (corner(t, k + 1), corner(t, k + 2), corner(t, k)) else {
                continue;
            };
            let Some(o) = (0..3).filter_map(|m| corner(u, m)).find(|&v| v != i && v != j) else { continue };
            if on(boundary.at(o), circle(t)) && on(boundary.at(q), circle(u)) {
                let (a, b) = (root(&mut cell, t), root(&mut cell, u));
                if let Some(slot) = cell.get_mut(a.max(b)) {
                    *slot = a.min(b);
                }
            }
        }
    }
    let mut nodes: Vec<Node> = Vec::new();
    let mut links: Vec<Vec<usize>> = Vec::new();
    let mut chords: HashMap<(usize, usize), usize> = HashMap::new();
    let add = |nodes: &mut Vec<Node>, links: &mut Vec<Vec<usize>>, node: Node| {
        nodes.push(node);
        links.push(Vec::new());
        nodes.len() - 1
    };
    // Each cell's corners, and the nodes of its chords to other cells.
    let mut corners_of: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut chords_of: Vec<Vec<usize>> = vec![Vec::new(); n];
    for t in 0..n {
        let c = root(&mut cell, t);
        for k in 0..3 {
            let (Some(i), Some(j)) = (corner(t, k + 1), corner(t, k + 2)) else { continue };
            if let (Some(own), Some(v)) = (corners_of.get_mut(c), corner(t, k)) {
                own.push(v);
            }
            match across(t, k).map(|u| root(&mut cell, u)) {
                Some(other) if other != c => {
                    let key = (i.min(j), i.max(j));
                    let id = match chords.get(&key) {
                        Some(&id) => id,
                        None => {
                            let (a, b) = (boundary.at(i), boundary.at(j));
                            let at = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
                            let id = add(&mut nodes, &mut links, Node { at, corners: vec![i, j] });
                            chords.insert(key, id);
                            id
                        }
                    };
                    if let Some(own) = chords_of.get_mut(c) {
                        own.push(id);
                    }
                }
                Some(_) => {}
                None => {
                    boundary.next.entry(i).or_default().push(j);
                    boundary.next.entry(j).or_default().push(i);
                }
            }
        }
    }
    for (mut corners, inner) in corners_of.into_iter().zip(chords_of) {
        corners.sort_unstable();
        corners.dedup();
        match inner.as_slice() {
            [a, b] => link(&mut links, *a, *b),
            [_, _, _, ..] => {
                let k = corners.len() as f64;
                let at = corners
                    .iter()
                    .map(|&i| boundary.at(i))
                    .fold([0.0, 0.0], |c, p| [c[0] + p[0] / k, c[1] + p[1] / k]);
                let hub = add(&mut nodes, &mut links, Node { at, corners });
                for &c in &inner {
                    link(&mut links, hub, c);
                }
            }
            _ => {}
        }
    }
    let mut thickness: Vec<f64> =
        nodes.iter().map(|node| 2.0 * boundary.distance(node.at, &node.corners)).collect();
    prune(&nodes, &thickness, &mut links);
    blobs(region, axis_at, &mut nodes, &mut thickness, &mut links);
    extend(&mut nodes, &mut thickness, &mut links, &boundary, tolerance);
    let line = |(ids, closed): (Vec<usize>, bool)| Centreline {
        points: ids.iter().filter_map(|&i| nodes.get(i).map(|n| n.at)).collect(),
        thickness: ids.iter().filter_map(|&i| thickness.get(i).copied()).collect(),
        closed,
    };
    Ok(chains(&links).into_iter().map(line).collect())
}

/// The cell of triangle `t`: the least triangle joined to it, halving the path there.
fn root(cell: &mut [usize], mut t: usize) -> usize {
    for _ in 0..cell.len() {
        let up = cell.get(t).copied().unwrap_or(t);
        if up == t {
            break;
        }
        let next = cell.get(up).copied().unwrap_or(up);
        if let Some(slot) = cell.get_mut(t) {
            *slot = next;
        }
        t = next;
    }
    t
}

/// The centre and radius of the circle through three points, if they are not on one line.
fn circumcircle(a: Point, b: Point, c: Point) -> Option<(Point, f64)> {
    let (b, c) = ([b[0] - a[0], b[1] - a[1]], [c[0] - a[0], c[1] - a[1]]);
    let d = 2.0 * (b[0] * c[1] - b[1] * c[0]);
    let (bb, cc) = (b[0] * b[0] + b[1] * b[1], c[0] * c[0] + c[1] * c[1]);
    if d.abs() <= f64::EPSILON * (bb + cc) {
        return None;
    }
    let u = [(c[1] * bb - b[1] * cc) / d, (b[0] * cc - c[0] * bb) / d];
    Some(([a[0] + u[0], a[1] + u[1]], u[0].hypot(u[1])))
}

fn link(links: &mut [Vec<usize>], a: usize, b: usize) {
    if let Some(l) = links.get_mut(a) {
        l.push(b);
    }
    if let Some(l) = links.get_mut(b) {
        l.push(a);
    }
}

/// Drops each branch that runs from a junction to an end no longer than the strip is thick at
/// the junction: it stays within the junction's disc, reaching only into a corner, which the
/// junction's own bead fills. A junction with nothing else keeps its two longest.
fn prune(nodes: &[Node], thickness: &[f64], links: &mut [Vec<usize>]) {
    let degree = |links: &[Vec<usize>], i: usize| links.get(i).map_or(0, Vec::len);
    let at = |i: usize| nodes.get(i).map_or([0.0, 0.0], |n| n.at);
    let junction = |links: &[Vec<usize>], i: usize| degree(links, i) >= 3;
    loop {
        // Every branch from an end to a junction: the junction, the branch's length and nodes.
        let mut branches: Vec<(usize, f64, Vec<usize>)> = Vec::new();
        for end in (0..links.len()).filter(|&i| degree(links, i) == 1) {
            let (mut branch, mut length) = (vec![end], 0.0);
            let (mut from, mut next) = (end, links.get(end).and_then(|l| l.first()).copied());
            while let Some(i) = next {
                length += (at(i)[0] - at(from)[0]).hypot(at(i)[1] - at(from)[1]);
                if degree(links, i) != 2 {
                    break;
                }
                branch.push(i);
                (from, next) = (i, links.get(i).and_then(|l| l.iter().find(|&&m| m != from)).copied());
            }
            if let Some(j) = next.filter(|&j| junction(links, j)) {
                branches.push((j, length, branch));
            }
        }
        // Each junction's short branches, longest first.
        // A junction's centroid may sit off the centre line: the strip is as thick there as at
        // the thickest point next to it.
        let disc = |j: usize| {
            let around = links.get(j).into_iter().flatten().chain(std::iter::once(&j));
            around.filter_map(|&i| thickness.get(i)).fold(0.0, |m: f64, &t| m.max(t))
        };
        branches.retain(|(j, length, _)| *length <= disc(*j));
        branches.sort_by(|a, b| a.0.cmp(&b.0).then(b.1.total_cmp(&a.1)));
        let mut drop = Vec::new();
        for group in branches.chunk_by(|a, b| a.0 == b.0) {
            let Some((j, _, _)) = group.first() else { continue };
            // A junction with no other branch keeps its two longest, so its strip runs through.
            let keep = if degree(links, *j) > group.len() { 0 } else { group.len().min(2) };
            drop.extend(group.iter().skip(keep).flat_map(|(_, _, branch)| branch.iter().copied()));
        }
        if drop.is_empty() {
            break;
        }
        for i in drop {
            for m in links.get_mut(i).map(std::mem::take).unwrap_or_default() {
                if let Some(l) = links.get_mut(m) {
                    l.retain(|&k| k != i);
                }
            }
        }
    }
    // A junction's centre left between two chords is off the centre line: the line runs
    // straight from one chord's midpoint to the other's.
    for i in 0..links.len() {
        let hub = nodes.get(i).is_some_and(|n| n.corners.len() >= 3);
        let Some(&[a, b]) = links.get(i).map(Vec::as_slice).filter(|_| hub) else { continue };
        if a == b {
            continue;
        }
        for (m, other) in [(a, b), (b, a)] {
            if let Some(l) = links.get_mut(m) {
                l.retain(|&k| k != i);
                l.push(other);
            }
        }
        if let Some(l) = links.get_mut(i) {
            l.clear();
        }
    }
}

/// How far a ray from `p` along the unit `d` runs before it meets an edge of `shape`.
fn reach(shape: &[Contour], p: Point, d: [f64; 2]) -> Option<f64> {
    let cross = |a: [f64; 2], b: [f64; 2]| a[0] * b[1] - a[1] * b[0];
    let mut best: Option<f64> = None;
    for (a, b) in shape.iter().flat_map(|c| edges(c)) {
        let (e, f) = ([b[0] - a[0], b[1] - a[1]], [a[0] - p[0], a[1] - p[1]]);
        let den = cross(d, e);
        if den.abs() > 0.0 {
            let (u, v) = (cross(f, e) / den, cross(f, d) / den);
            if u > 0.0 && (0.0..=1.0).contains(&v) && best.is_none_or(|x| u < x) {
                best = Some(u);
            }
        }
    }
    best
}

/// Lays the centre line of each shape of `region` no longer than twice as thick as it is, its
/// chord at most twice its area over that chord, along the chord through its centroid on its
/// own axis, edge to edge, as thick as its area over the chord's length. Of the lines a
/// symmetric shape leaves open, as a square's two, it takes the one nearest the axis `axis_at`
/// its centroid gives; a shape with none of its own, as a disc, that axis. Within half its
/// thickness of either end, the points of a strip lie nearer the end than its sides, and the
/// axis there turns with which of them the triangulation joined (`extend`): such a shape is all
/// ends, and its axis would turn with the plate.
fn blobs(
    region: &Region,
    axis_at: &dyn Fn(Point) -> [f64; 2],
    nodes: &mut Vec<Node>,
    thickness: &mut Vec<f64>,
    links: &mut Vec<Vec<usize>>,
) {
    let n = links.len();
    let at = |nodes: &[Node], i: usize| nodes.get(i).map_or([0.0, 0.0], |n| n.at);
    // The axis's pieces, each by its least node, with their nodes.
    let mut piece: Vec<usize> = (0..n).collect();
    for i in 0..n {
        for j in links.get(i).cloned().unwrap_or_default() {
            let (a, b) = (root(&mut piece, i), root(&mut piece, j));
            if let Some(slot) = piece.get_mut(a.max(b)) {
                *slot = a.min(b);
            }
        }
    }
    let mut members = vec![Vec::new(); n];
    for i in (0..n).filter(|&i| links.get(i).is_some_and(|l| !l.is_empty())) {
        let r = root(&mut piece, i);
        if let Some(m) = members.get_mut(r) {
            m.push(i);
        }
    }
    let roots: Vec<usize> = (0..n).filter(|&r| members.get(r).is_some_and(|m| !m.is_empty())).collect();
    for shape in region {
        let Some(moments) = Moments::of(std::iter::once((shape.as_slice(), 1.0))) else { continue };
        let edges: Vec<(Point, Point)> = shape.iter().flat_map(|c| edges(c)).collect();
        let centre = moments.centroid;
        if !encloses(edges.iter().copied(), centre) {
            continue;
        }
        let frame = axis_at(centre);
        let nearest = |lines: &mut dyn Iterator<Item = [f64; 2]>| {
            let across = |u: [f64; 2]| (u[0] * frame[0] + u[1] * frame[1]).abs();
            lines.fold(None, |best: Option<[f64; 2]>, u| match best {
                Some(b) if across(b) >= across(u) => Some(b),
                _ => Some(u),
            })
        };
        let d = moments.axes.and_then(|axes| nearest(&mut axes.lines())).unwrap_or(frame);
        let (Some(fore), Some(aft)) = (reach(shape, centre, d), reach(shape, centre, [-d[0], -d[1]])) else {
            continue;
        };
        let chord = fore + aft;
        if chord * chord > 2.0 * moments.area {
            continue;
        }
        let [lo, hi] = edges.iter().fold([[f64::MAX; 2], [f64::MIN; 2]], |[lo, hi], (a, _)| {
            [[lo[0].min(a[0]), lo[1].min(a[1])], [hi[0].max(a[0]), hi[1].max(a[1])]]
        });
        let within = |p: Point| lo[0] <= p[0] && p[0] <= hi[0] && lo[1] <= p[1] && p[1] <= hi[1];
        for &r in &roots {
            let p = at(nodes, r);
            if within(p) && encloses(edges.iter().copied(), p) {
                for &i in members.get(r).into_iter().flatten() {
                    if let Some(l) = links.get_mut(i) {
                        l.clear();
                    }
                }
            }
        }
        let t = moments.area / chord;
        for end in [
            [centre[0] - aft * d[0], centre[1] - aft * d[1]],
            [centre[0] + fore * d[0], centre[1] + fore * d[1]],
        ] {
            nodes.push(Node { at: end, corners: Vec::new() });
            thickness.push(t);
            links.push(Vec::new());
        }
        link(links, nodes.len() - 2, nodes.len() - 1);
    }
}

/// Runs each end of the axis straight on to the boundary, as thick as the strip it ends.
/// Within half its thickness of its end a strip's points are nearer the end than its sides:
/// the axis there wanders between the end's chords and stops short, and the distance to the
/// boundary tells the end's nearness, not the strip's thickness. The line is kept up to the
/// first point, from the end back, whose disc stops short of the boundary past the end by more
/// than `tolerance`, touching the sides only; from there it runs on along its course over the
/// half thickness behind until it meets the boundary, as thick as the strip there or next to
/// it. Towards a wedge's tip every disc stops short of it: the line runs on from its last
/// point.
fn extend(
    nodes: &mut Vec<Node>,
    thickness: &mut Vec<f64>,
    links: &mut Vec<Vec<usize>>,
    boundary: &Boundary,
    tolerance: f64,
) {
    let at = |nodes: &[Node], i: usize| nodes.get(i).map_or([0.0, 0.0], |n| n.at);
    let thick = |thickness: &[f64], i: usize| thickness.get(i).copied().unwrap_or(0.0);
    let unit = |p: Point, q: Point| {
        let l = (p[0] - q[0]).hypot(p[1] - q[1]);
        (l > 0.0).then(|| [(p[0] - q[0]) / l, (p[1] - q[1]) / l])
    };
    let ends: Vec<usize> = (0..links.len()).filter(|&i| links.get(i).is_some_and(|l| l.len() == 1)).collect();
    for end in ends {
        // The line back from the end, as far as the thickest disc on it reaches, with the
        // length along it to each point.
        let (mut line, mut along, mut widest) = (vec![end], vec![0.0], thick(thickness, end));
        let (mut from, mut next) =
            (end, links.get(end).filter(|l| l.len() == 1).and_then(|l| l.first()).copied());
        while let Some(i) = next {
            let (p, q) = (at(nodes, from), at(nodes, i));
            let walked = along.last().copied().unwrap_or(0.0) + (p[0] - q[0]).hypot(p[1] - q[1]);
            line.push(i);
            along.push(walked);
            widest = widest.max(thick(thickness, i));
            if walked > widest || links.get(i).map_or(0, Vec::len) != 2 {
                break;
            }
            (from, next) = (i, links.get(i).and_then(|l| l.iter().find(|&&m| m != from)).copied());
        }
        let (Some(&second), Some(&last)) = (line.get(1), line.last()) else { continue };
        let corners = |nodes: &[Node], i: usize| nodes.get(i).map(|n| n.corners.clone()).unwrap_or_default();
        let Some(ahead) = unit(at(nodes, end), at(nodes, second))
            .and_then(|d| boundary.hit(at(nodes, end), d, &corners(nodes, end), widest))
        else {
            continue;
        };
        // The first point whose disc stops short of the boundary past the end: nearer its sides.
        let Some(k) = (0..line.len()).find(|&k| {
            let (i, l) = (line.get(k).copied().unwrap_or(end), along.get(k).copied().unwrap_or(0.0));
            l + ahead > thick(thickness, i) / 2.0 + tolerance
        }) else {
            continue;
        };
        let (Some(&kept), Some(&lk)) = (line.get(k), along.get(k)) else { continue };
        // The course over the half thickness behind the kept point.
        let half = thick(thickness, kept) / 2.0;
        let behind = (k + 1..line.len())
            .find(|&b| along.get(b).is_some_and(|&lb| lb - lk >= half))
            .and_then(|b| line.get(b).copied())
            .unwrap_or(last);
        let Some(d) =
            unit(at(nodes, kept), at(nodes, behind)).or_else(|| unit(at(nodes, end), at(nodes, second)))
        else {
            continue;
        };
        let reach = lk + ahead + widest;
        let Some(u) = boundary.hit(at(nodes, kept), d, &corners(nodes, kept), reach).filter(|&u| u > 0.0)
        else {
            continue;
        };
        let next_to = line.get(k + 1).map_or(0.0, |&i| thick(thickness, i));
        let t = thick(thickness, kept).max(next_to);
        // The points past the kept one leave the line.
        for &i in line.iter().take(k) {
            for m in links.get_mut(i).map(std::mem::take).unwrap_or_default() {
                if let Some(l) = links.get_mut(m) {
                    l.retain(|&j| j != i);
                }
            }
        }
        if let Some(slot) = thickness.get_mut(kept) {
            *slot = t;
        }
        let p = at(nodes, kept);
        nodes.push(Node { at: [p[0] + u * d[0], p[1] + u * d[1]], corners: Vec::new() });
        thickness.push(t);
        links.push(Vec::new());
        link(links, kept, nodes.len() - 1);
    }
}

/// The graph cut into chains between its ends and junctions, then its loops, each as its nodes
/// in order and whether it closes.
fn chains(links: &[Vec<usize>]) -> Vec<(Vec<usize>, bool)> {
    let degree = |i: usize| links.get(i).map_or(0, Vec::len);
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
            let Some(&to) = links.get(at).and_then(|l| l.iter().find(|&&m| m != from)) else { break };
            (from, at) = (at, to);
        }
        ids
    };
    for loops in [false, true] {
        for start in 0..links.len() {
            if !loops && degree(start) == 2 {
                continue;
            }
            for &next in links.get(start).into_iter().flatten() {
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
