//! The toolhead against what is printed: no part of the head comes down into a bead laid before.
//!
//! Seen from below, the head is a solid of revolution about the nozzle's axis whose underside
//! lies `floor(r)` above the tip at distance `r` from it: nothing on the tip's flat, then the
//! nozzle's cone up to `nozzle_height`, the rest of the head out to `head_radius`, and past it
//! the gantry; all of it taken `margin` lower, for beads that curl up and a plate not quite
//! flat. A nozzle at `z` meets a printed point `p` standing `h(p)` high when
//! `h(p) > z + floor(|p − tip|)`. A bead stands as high as the nozzle that laid it, over its
//! width. The heights are kept on a grid `CELL` fine, under a pyramid of maxima, each cell of a
//! level the highest of the four under it, so a query descends only where something stands
//! higher than the head clears there: nowhere, while each layer is laid over the last.

use crate::Toolhead;
use layercraft_motion::Move;

/// The side of a cell of the height grid, in mm.
const CELL: f64 = 0.2;
/// Room for rounding, in mm.
const SLACK: f64 = 1e-6;
/// The most cells along a side of the grid: a plate 400 mm across.
const MAX_SIDE: usize = 2000;

impl Toolhead {
    /// How high the head's underside stands above its tip at distance `r` from its axis.
    pub fn floor(&self, r: f64) -> f64 {
        if r <= self.tip_radius {
            return 0.0;
        }
        let up = if r > self.head_radius {
            self.gantry_height
        } else if self.flare > 0.0 {
            ((r - self.tip_radius) / self.flare).min(self.nozzle_height)
        } else {
            self.nozzle_height
        };
        (up - self.margin).max(0.0)
    }
}

/// One level of the pyramid: `width × depth` cells, each `side` mm across.
struct Level {
    width: usize,
    depth: usize,
    side: f64,
    cells: Vec<f64>,
}

/// The height of what is printed, over the plate.
pub struct Heights {
    origin: [f64; 2],
    levels: Vec<Level>,
}

impl Heights {
    /// Nothing printed yet over the area `[min, max]`.
    pub fn new(area: [[f64; 2]; 2]) -> Heights {
        let cells = |span: f64| ((span / CELL).ceil().max(1.0) as usize).min(MAX_SIDE);
        let (mut width, mut depth) = (cells(area[1][0] - area[0][0]), cells(area[1][1] - area[0][1]));
        let mut side = CELL;
        let mut levels = Vec::new();
        loop {
            levels.push(Level { width, depth, side, cells: vec![0.0; width * depth] });
            if width == 1 && depth == 1 {
                break;
            }
            (width, depth, side) = (width.div_ceil(2), depth.div_ceil(2), side * 2.0);
        }
        Heights { origin: area[0], levels }
    }

    /// Raises the cell `(i, j)` of the grid to `h`, and every cell above it.
    fn raise(&mut self, mut i: usize, mut j: usize, h: f64) {
        for level in &mut self.levels {
            let Some(cell) = level.cells.get_mut(j * level.width + i) else { return };
            if *cell >= h {
                return;
            }
            *cell = h;
            (i, j) = (i / 2, j / 2);
        }
    }

    /// Lays a bead `width` wide from `a` to `b`, standing at `h`: every cell whose centre lies
    /// within half the width and half a cell's diagonal of its centre line.
    pub fn lay(&mut self, a: [f64; 2], b: [f64; 2], width: f64, h: f64) {
        let Some(grid) = self.levels.first() else { return };
        let (cols, rows) = (grid.width, grid.depth);
        let reach = width / 2.0 + CELL * std::f64::consts::FRAC_1_SQRT_2;
        let index =
            |v: f64, o: f64, n: usize| (((v - o) / CELL).floor().max(0.0) as usize).min(n.saturating_sub(1));
        let (i0, i1) = (
            index(a[0].min(b[0]) - reach, self.origin[0], cols),
            index(a[0].max(b[0]) + reach, self.origin[0], cols),
        );
        let (j0, j1) = (
            index(a[1].min(b[1]) - reach, self.origin[1], rows),
            index(a[1].max(b[1]) + reach, self.origin[1], rows),
        );
        for j in j0..=j1 {
            for i in i0..=i1 {
                let c = [self.origin[0] + (i as f64 + 0.5) * CELL, self.origin[1] + (j as f64 + 0.5) * CELL];
                if segment_distance(c, a, b) <= reach {
                    self.raise(i, j, h);
                }
            }
        }
    }

    /// How far the head with its tip at `tip`, anywhere within `blur` of it across the plate,
    /// comes down into what is printed; `None` when it clears it all.
    pub fn intrusion(&self, head: &Toolhead, tip: [f64; 3], blur: f64) -> Option<f64> {
        let top = self.levels.len().checked_sub(1)?;
        self.descend(head, tip, blur, top, 0, 0)
    }

    fn descend(
        &self,
        head: &Toolhead,
        tip: [f64; 3],
        blur: f64,
        k: usize,
        i: usize,
        j: usize,
    ) -> Option<f64> {
        let level = self.levels.get(k)?;
        let h = *level.cells.get(j * level.width + i)?;
        if h <= tip[2] + SLACK {
            return None;
        }
        let lo = [self.origin[0] + i as f64 * level.side, self.origin[1] + j as f64 * level.side];
        let dx = (lo[0] - tip[0]).max(tip[0] - lo[0] - level.side).max(0.0);
        let dy = (lo[1] - tip[1]).max(tip[1] - lo[1] - level.side).max(0.0);
        let over = h - tip[2] - head.floor((dx.hypot(dy) - blur).max(0.0));
        if over <= SLACK {
            return None;
        }
        if k == 0 {
            return Some(over);
        }
        let below = self.levels.get(k - 1)?;
        let children = [(2 * i, 2 * j), (2 * i + 1, 2 * j), (2 * i, 2 * j + 1), (2 * i + 1, 2 * j + 1)];
        children
            .into_iter()
            .filter(|&(ci, cj)| ci < below.width && cj < below.depth)
            .filter_map(|(ci, cj)| self.descend(head, tip, blur, k - 1, ci, cj))
            .reduce(f64::max)
    }
}

/// The distance from `p` to the segment `a`–`b`.
fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let len2 = d[0] * d[0] + d[1] * d[1];
    let t =
        if len2 > 0.0 { (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
}

/// The points a move from `from` passes, no more than half a cell apart, `from` and its end
/// included.
pub fn samples(m: &Move, from: [f64; 3]) -> impl Iterator<Item = [f64; 3]> + '_ {
    let steps = ((m.length(from) / (CELL / 2.0)).ceil() as usize).clamp(1, 1 << 20);
    (0..=steps).map(move |k| m.point(from, k as f64 / steps as f64))
}

/// How far a move from `from` comes down into what `heights` holds, if at all. The nozzle
/// lies within a quarter of a cell of one of the points it passes, on the halves of the steps
/// on either side of it, and no lower than they reach.
pub fn intrusion(heights: &Heights, head: &Toolhead, m: &Move, from: [f64; 3]) -> Option<f64> {
    let points: Vec<[f64; 3]> = samples(m, from).collect();
    let low = |k: usize| {
        let z = points.get(k).map_or(f64::INFINITY, |p| p[2]);
        let half = |j: Option<usize>| j.and_then(|j| points.get(j)).map_or(z, |p| (z + p[2]) / 2.0);
        z.min(half(k.checked_sub(1))).min(half(Some(k + 1)))
    };
    (0..points.len())
        .filter_map(|k| {
            let p = points.get(k)?;
            heights.intrusion(head, [p[0], p[1], low(k)], CELL / 4.0)
        })
        .reduce(f64::max)
}

/// Lays the bead a move from `from` lays, if any.
pub fn lay(heights: &mut Heights, m: &Move, from: [f64; 3]) {
    let Some(bead) = m.bead else { return };
    if m.e <= 0.0 {
        return;
    }
    let points: Vec<[f64; 3]> = samples(m, from).collect();
    for w in points.windows(2) {
        if let [a, b] = w {
            heights.lay([a[0], a[1]], [b[0], b[1]], bead.width, a[2].max(b[2]));
        }
    }
}
