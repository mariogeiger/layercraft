//! The curves where the rolls' field is zero: the infill's lines. Over each cell of the grid
//! whose four corners are live, the zero crosses the edges whose ends differ in sign, where the
//! field, linear along the edge, vanishes: through two of them, or at a saddle through all four,
//! cutting off the two corners of the other sign than the cell's centre, the mean of its corners.
//! Pieces sharing an edge join into curves, open where they leave the live cells, closed
//! otherwise.

use super::Nodes;
use layercraft_geom::Point;
use std::collections::{BTreeMap, BTreeSet};

/// The curves where `u` is zero over the cells whose corners are all `live`, in the frame.
pub(super) fn zeros(nodes: &Nodes, u: &[f64], live: &[bool]) -> Vec<Vec<Point>> {
    let nx = nodes.nx;
    let at = |i: usize, j: usize| u.get(j * nx + i).copied().unwrap_or(0.0);
    let free = |i: usize, j: usize| live.get(j * nx + i).copied().unwrap_or(false);
    // Edge `2n` runs from node `n` along x, edge `2n + 1` along y.
    let (along_x, along_y) =
        (|i: usize, j: usize| 2 * (j * nx + i), |i: usize, j: usize| 2 * (j * nx + i) + 1);
    let mut links: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for j in 0..nodes.ny.saturating_sub(1) {
        for i in 0..nx.saturating_sub(1) {
            if !(free(i, j) && free(i + 1, j) && free(i + 1, j + 1) && free(i, j + 1)) {
                continue;
            }
            // Corners anticlockwise from the lower left, each with its two edges: along x below
            // or above it, along y to its side.
            let (below, right, above, left) =
                (along_x(i, j), along_y(i + 1, j), along_x(i, j + 1), along_y(i, j));
            let corners = [
                (at(i, j) > 0.0, below, left),
                (at(i + 1, j) > 0.0, below, right),
                (at(i + 1, j + 1) > 0.0, right, above),
                (at(i, j + 1) > 0.0, above, left),
            ];
            let sides = [(below, 0, 1), (right, 1, 2), (above, 3, 2), (left, 0, 3)];
            let sign = |c: usize| corners.get(c).is_some_and(|c| c.0);
            let crossed: Vec<usize> = sides.iter().filter(|s| sign(s.1) != sign(s.2)).map(|s| s.0).collect();
            let mut link = |a: usize, b: usize| {
                links.entry(a).or_default().push(b);
                links.entry(b).or_default().push(a);
            };
            match crossed.as_slice() {
                &[a, b] => link(a, b),
                // A saddle: the corners of the other sign than the cell's centre are cut off.
                [_, _, _, _] => {
                    let centre = (at(i, j) + at(i + 1, j) + at(i + 1, j + 1) + at(i, j + 1)) > 0.0;
                    for &(positive, a, b) in &corners {
                        if positive != centre {
                            link(a, b);
                        }
                    }
                }
                _ => {}
            }
        }
    }
    let point = |edge: usize| {
        let n = edge / 2;
        let (i, j) = (n % nx, n / nx);
        let (i1, j1) = if edge.is_multiple_of(2) { (i + 1, j) } else { (i, j + 1) };
        let (a, b) = (at(i, j), at(i1, j1));
        let t = if a != b { a / (a - b) } else { 0.5 };
        let (p, q) = (nodes.node(i, j), nodes.node(i1, j1));
        [p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])]
    };
    let mut curves = Vec::new();
    let mut seen = BTreeSet::new();
    // Open curves from their ends first, then the loops.
    let ends: Vec<usize> = links.iter().filter(|(_, l)| l.len() == 1).map(|(&e, _)| e).collect();
    let starts = ends.into_iter().chain(links.keys().copied().collect::<Vec<_>>());
    for start in starts {
        if seen.contains(&start) {
            continue;
        }
        let mut curve = vec![point(start)];
        seen.insert(start);
        let (mut previous, mut current) = (usize::MAX, start);
        loop {
            let next = links
                .get(&current)
                .and_then(|l| l.iter().copied().find(|&e| e != previous && !seen.contains(&e)));
            let Some(next) = next else {
                // Back at the start of a loop: close it.
                if links.get(&current).is_some_and(|l| l.contains(&start)) && curve.len() > 2 {
                    curve.push(point(start));
                }
                break;
            };
            seen.insert(next);
            curve.push(point(next));
            (previous, current) = (current, next);
        }
        if curve.len() >= 2 {
            curves.push(curve);
        }
    }
    curves
}
