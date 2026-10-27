//! How far each segment of a wall hangs over air.
//!
//! A bead `w` wide whose centre lies at signed distance `d` outside the section below (negative
//! inside it) reaches `d + w/2` past that section's edge: it hangs the fraction
//! `u = clamp(d/w + 1/2, 0, 1)` of its width over air, snapped to 0 or 1 within a resolution of
//! either. A segment takes the fraction at its
//! middle; one whose fraction changes along it by more than `STEP` is halved, down to pieces
//! `w · STEP` long, so the fraction changes where the overhang does. Distances matter only within
//! a bead, so each point looks only at the edges filed in the grid cells, a bead wide, around it.

use layercraft_geom::{Point, RESOLUTION_MM, Region};
use std::collections::HashMap;

/// The most a segment's fraction may change along it before it is halved.
const STEP: f64 = 0.125;

/// The most samples taken along one edge: past this many beads, the rest of it is not filed.
const MAX_SAMPLES: usize = 1_000_000;

/// The section below, its edges filed by grid cell.
pub struct Below {
    edges: Vec<(Point, Point)>,
    cells: HashMap<(i64, i64), Vec<usize>>,
    /// The edges by the rows of cells their height spans, for rays along x.
    rows: HashMap<i64, Vec<usize>>,
    /// The cell's side, a bead's width.
    cell: f64,
}

impl Below {
    /// The section `below`, for beads `width` wide.
    pub fn new(below: &Region, width: f64) -> Below {
        let cell = width.max(1e-3);
        let index = |v: f64| (v / cell).floor() as i64;
        let mut edges = Vec::new();
        let mut cells: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        let mut rows: HashMap<i64, Vec<usize>> = HashMap::new();
        for contour in below.iter().flatten() {
            let n = contour.len();
            for i in 0..n {
                let (Some(&a), Some(&b)) = (contour.get(i), contour.get((i + 1) % n)) else { continue };
                let k = edges.len();
                edges.push((a, b));
                let (y0, y1) = (index(a[1].min(b[1])), index(a[1].max(b[1])));
                for y in y0..=y1.min(y0.saturating_add(MAX_SAMPLES as i64)) {
                    rows.entry(y).or_default().push(k);
                }
                // Samples half a cell apart, each filing the edge in its cell and the eight around
                // it: every cell the edge passes through, and its neighbours.
                let samples =
                    (((b[0] - a[0]).hypot(b[1] - a[1]) / (cell / 2.0)).ceil() as usize).min(MAX_SAMPLES);
                for j in 0..=samples {
                    let t = j as f64 / samples.max(1) as f64;
                    let (x, y) = (index(a[0] + t * (b[0] - a[0])), index(a[1] + t * (b[1] - a[1])));
                    for dx in -1..=1 {
                        for dy in -1..=1 {
                            let filed = cells.entry((x + dx, y + dy)).or_default();
                            if filed.last() != Some(&k) {
                                filed.push(k);
                            }
                        }
                    }
                }
            }
        }
        Below { edges, cells, rows, cell }
    }

    fn index(&self, v: f64) -> i64 {
        (v / self.cell).floor() as i64
    }

    /// Whether `p` lies inside the section: the edges a ray from it to +x crosses, odd.
    fn holds(&self, p: Point) -> bool {
        let edges = self.rows.get(&self.index(p[1])).into_iter().flatten().filter_map(|&k| self.edges.get(k));
        edges
            .filter(|&&(a, b)| {
                (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
            })
            .count()
            % 2
            == 1
    }

    /// The distance from `p` to the section's edge, when under one cell; past it, one cell.
    fn near_distance(&self, p: Point) -> f64 {
        let cell = (self.index(p[0]), self.index(p[1]));
        let edges = self.cells.get(&cell).into_iter().flatten().filter_map(|&k| self.edges.get(k));
        edges.map(|&(a, b)| distance_to_segment(p, a, b)).fold(self.cell, f64::min)
    }

    /// The fraction of a bead `width` wide centred at `p` that hangs over air.
    pub fn fraction(&self, p: Point, width: f64) -> f64 {
        let d = self.near_distance(p).min(width);
        let signed = if self.holds(p) { -d } else { d };
        // How far the bead reaches past the edge, a resolution's rounding from either end snapped.
        let reach = signed + width / 2.0;
        if reach <= RESOLUTION_MM {
            0.0
        } else if reach >= width - RESOLUTION_MM {
            1.0
        } else {
            reach / width
        }
    }
}

fn distance_to_segment(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let len2 = d[0] * d[0] + d[1] * d[1];
    let t =
        if len2 > 0.0 { (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
}

fn midpoint(a: Point, b: Point) -> Point {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

/// The segment `a`–`b` in pieces, each with its fraction, appended after `a`.
fn split(
    below: &Below,
    width: f64,
    (a, ua): (Point, f64),
    (b, ub): (Point, f64),
    points: &mut Vec<Point>,
    fractions: &mut Vec<f64>,
) {
    let m = midpoint(a, b);
    let um = below.fraction(m, width);
    let even = (ua - um).abs() <= STEP && (ub - um).abs() <= STEP;
    let short = (b[0] - a[0]).hypot(b[1] - a[1]) <= width * STEP;
    if even || short {
        points.push(b);
        fractions.push(um);
    } else {
        split(below, width, (a, ua), (m, um), points, fractions);
        split(below, width, (m, um), (b, ub), points, fractions);
    }
}

/// The points of a wall bead `width` wide, its segments cut where their overhang changes, and
/// each segment's fraction over air; no fractions when nothing hangs.
pub fn overhang(points: &[Point], closed: bool, below: &Below, width: f64) -> (Vec<Point>, Vec<f64>) {
    let mut ends: Vec<Point> = points.to_vec();
    if closed && let Some(&first) = points.first() {
        ends.push(first);
    }
    let fractions_at: Vec<f64> = ends.iter().map(|&p| below.fraction(p, width)).collect();
    let mut out = Vec::with_capacity(ends.len());
    let mut fractions = Vec::with_capacity(ends.len());
    if let Some(&first) = ends.first() {
        out.push(first);
    }
    for i in 1..ends.len() {
        let (Some(&a), Some(&b), Some(&ua), Some(&ub)) =
            (ends.get(i - 1), ends.get(i), fractions_at.get(i - 1), fractions_at.get(i))
        else {
            continue;
        };
        split(below, width, (a, ua), (b, ub), &mut out, &mut fractions);
    }
    if closed {
        out.pop();
    }
    if fractions.iter().all(|&u| u <= 0.0) {
        return (points.to_vec(), Vec::new());
    }
    (out, fractions)
}
