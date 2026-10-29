//! The toolhead against what is printed: no part of the head comes down into a bead laid before.
//!
//! Seen from below, the head is a solid of revolution about the nozzle's axis whose underside
//! lies `floor(r)` above the tip at distance `r` from it: nothing on the tip's flat, then the
//! nozzle's cone, the rest of the head out to `head_radius`, and past it the gantry; the head
//! and the gantry taken `margin` lower, for beads that curl up and a plate not quite flat. A
//! nozzle at `z` meets a printed point `p` standing `h(p)` high when `h(p) > z + floor(|p − tip|)`.
//! On a bent layer the nozzle's cone meets the beads of its own layer as it lays them, beside
//! and behind it up the slope: they lie on the layer's top, their flat tops standing above it by
//! up to half a bead's width times the layer's steepest slope `s`, and the grid's cells place
//! them up to a cell and a half's diagonal off. So within the cone's reach, out to where it meets
//! the rest of the head, a point counts only when it also stands above the layer's top there,
//! raised by that much. The layers before lie at least a slab lower than the layer's top. A bead stands as high as the nozzle that laid it, over its
//! width. The heights are kept on a grid `CELL` fine, under a pyramid of maxima, each cell of a
//! level the highest of the four under it, so a query descends only where something stands
//! higher than the head clears there: nowhere, while each layer is laid over the last.

use crate::Toolhead;
use layercraft_motion::{Layer, Move};

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
        let body = (self.nozzle_height - self.margin).max(0.0);
        if r > self.head_radius {
            (self.gantry_height - self.margin).max(0.0)
        } else if self.flare > 0.0 {
            ((r - self.tip_radius) / self.flare).min(body)
        } else {
            body
        }
    }

    /// How far the nozzle's cone reaches from its axis, to where it meets the rest of the head.
    fn cone_reach(&self) -> f64 {
        (self.tip_radius + self.flare * (self.nozzle_height - self.margin).max(0.0)).min(self.head_radius)
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
    /// The widest bead laid, in mm.
    widest: f64,
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
        Heights { origin: area[0], levels, widest: 0.0 }
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
        self.widest = self.widest.max(width);
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
    /// printing `layer`, whose steepest slope is `steepest`, comes down into what is printed;
    /// `None` when it clears it all.
    pub fn intrusion(
        &self,
        head: &Toolhead,
        tip: [f64; 3],
        blur: f64,
        layer: &Layer,
        steepest: f64,
    ) -> Option<f64> {
        let top = self.levels.len().checked_sub(1)?;
        // A bead marks cells up to half a diagonal past its edge, and the layer's top is taken at
        // its lowest over a cell, up to a diagonal from the plastic in it.
        let rounding = CELL * (std::f64::consts::FRAC_1_SQRT_2 + std::f64::consts::SQRT_2) + blur;
        let lift = (self.widest / 2.0 + rounding) * steepest;
        let reach = if steepest > 0.0 { head.cone_reach() } else { 0.0 };
        let query = Query { head, tip, blur, layer, lift, reach };
        self.descend(&query, top, 0, 0)
    }

    fn descend(&self, q: &Query, k: usize, i: usize, j: usize) -> Option<f64> {
        let level = self.levels.get(k)?;
        let h = *level.cells.get(j * level.width + i)?;
        let tip = q.tip;
        if h <= tip[2] + SLACK {
            return None;
        }
        let lo = [self.origin[0] + i as f64 * level.side, self.origin[1] + j as f64 * level.side];
        let dx = (lo[0] - tip[0]).max(tip[0] - lo[0] - level.side).max(0.0);
        let dy = (lo[1] - tip[1]).max(tip[1] - lo[1] - level.side).max(0.0);
        let near = (dx.hypot(dy) - q.blur).max(0.0);
        let mut over = h - tip[2] - q.head.floor(near);
        // On the finest cells only: a coarser one descends, its floor alone a bound.
        if over > SLACK && near <= q.reach && k == 0 {
            // The layer's own top, at its lowest over the cell, raised by its beads' flat tops.
            let [low, _] = q.layer.warp.range_over(lo, [lo[0] + level.side, lo[1] + level.side]);
            over = over.min(h - q.layer.z - low - q.lift);
        }
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
            .filter_map(|(ci, cj)| self.descend(q, k - 1, ci, cj))
            .reduce(f64::max)
    }
}

/// A head at a tip, anywhere within `blur` of it, printing a layer whose beads stand up to
/// `lift` above its top, out to `reach`.
struct Query<'a> {
    head: &'a Toolhead,
    tip: [f64; 3],
    blur: f64,
    layer: &'a Layer,
    lift: f64,
    reach: f64,
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

/// How far a move from `from` printing `layer`, whose steepest slope is `steepest`, comes down
/// into what `heights` holds, if at all. The nozzle lies within a quarter of a cell of one of
/// the points it passes, on the halves of the steps on either side of it, and no lower than they
/// reach.
pub fn intrusion(
    heights: &Heights,
    head: &Toolhead,
    m: &Move,
    from: [f64; 3],
    layer: &Layer,
    steepest: f64,
) -> Option<f64> {
    let points: Vec<[f64; 3]> = samples(m, from).collect();
    let low = |k: usize| {
        let z = points.get(k).map_or(f64::INFINITY, |p| p[2]);
        let half = |j: Option<usize>| j.and_then(|j| points.get(j)).map_or(z, |p| (z + p[2]) / 2.0);
        z.min(half(k.checked_sub(1))).min(half(Some(k + 1)))
    };
    (0..points.len())
        .filter_map(|k| {
            let p = points.get(k)?;
            heights.intrusion(head, [p[0], p[1], low(k)], CELL / 4.0, layer, steepest)
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
