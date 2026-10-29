//! The room a bead keeps on its layer. The nozzle's flat face, a disc of radius `ρ` at its tip,
//! levels what it passes over down to its own height: on a bent layer it passes later and lower
//! over beads already laid (on down its own bead, wiping back over it, gliding on to the next,
//! laying the next line downhill), and planes off what stood above it, which a bead pushing its
//! whole slab squeezes out at its sides. Each bead pushes only the plastic that stays.
//!
//! A bead fills its strip, as wide as beads lie apart, from the top of the layer below up to the
//! nozzle, level under its face. At a point of the strip laid with the nozzle at `z`, the lowest
//! the face stands over it afterwards, `F`, cuts `max(0, z − F)` off; a top cut by `c` below
//! leaves `c` more room. A straight piece pushes its slab's plastic times the room it keeps over
//! the slab, both sampled at the middles of cells a third of the strip wide, and no longer, tiling
//! it: a straight pass covers a point while it runs over an interval, which a quadratic bounds,
//! and stands lowest at one of its ends. A layer that stays level cuts nothing, and over a top uncut leaves every
//! bead's plastic as it is.

use crate::leaf::Leaf;
use crate::moves::{Point3, chained};
use crate::{Bead, Move};
use layercraft_geom::{Point, bead_spacing};

/// The chords a lift's helix passes as, per turn.
const CHORDS_PER_TURN: f64 = 16.0;

/// The most cells a grid of a layer holds; wider, its cells grow.
const MOST_CELLS: f64 = 4_194_304.0;

/// A straight stretch the nozzle passes, from `a` to `b`, standing lowest at `low`.
struct Pass {
    a: Point3,
    b: Point3,
    low: f64,
}

impl Pass {
    fn new(a: Point3, b: Point3) -> Pass {
        Pass { a, b, low: a[2].min(b[2]) }
    }

    /// The nozzle's height the fraction `u` along the pass.
    fn z(&self, u: f64) -> f64 {
        self.a[2] + u * (self.b[2] - self.a[2])
    }

    /// Whether `p` lies within `r` of the pass across the plate.
    fn within(&self, p: Point, r: f64) -> bool {
        let d = [self.b[0] - self.a[0], self.b[1] - self.a[1]];
        let f = [p[0] - self.a[0], p[1] - self.a[1]];
        let dd = d[0] * d[0] + d[1] * d[1];
        let u = if dd > 0.0 { ((f[0] * d[0] + f[1] * d[1]) / dd).clamp(0.0, 1.0) } else { 0.0 };
        let (x, y) = (f[0] - u * d[0], f[1] - u * d[1]);
        x * x + y * y <= r * r
    }

    /// The lowest the nozzle stands from the fraction `from` of the pass on, while its face
    /// `radius` wide covers `p`.
    fn lowest_over(&self, from: f64, p: Point, radius: f64) -> Option<f64> {
        let d = [self.b[0] - self.a[0], self.b[1] - self.a[1]];
        let f = [self.a[0] - p[0], self.a[1] - p[1]];
        let dd = d[0] * d[0] + d[1] * d[1];
        let ff = f[0] * f[0] + f[1] * f[1] - radius * radius;
        // |f + u·d|² ≤ ρ²: between the roots of dd·u² + 2·fd·u + ff; standing still, all along.
        let (u0, u1) = if dd > 0.0 {
            let fd = f[0] * d[0] + f[1] * d[1];
            let discriminant = fd * fd - dd * ff;
            if discriminant < 0.0 {
                return None;
            }
            let root = discriminant.sqrt();
            ((-fd - root) / dd, (-fd + root) / dd)
        } else if ff <= 0.0 {
            (0.0, 1.0)
        } else {
            return None;
        };
        let (u0, u1) = (u0.max(from), u1.min(1.0));
        (u0 <= u1).then(|| self.z(u0).min(self.z(u1)))
    }
}

/// Items filed by the square cells of a box of the plate, each cell's in the order filed.
struct Grid<T> {
    origin: Point,
    size: f64,
    columns: usize,
    rows: usize,
    /// Where each cell's items start, and the last's end.
    starts: Vec<usize>,
    items: Vec<T>,
}

impl<T: Copy> Grid<T> {
    /// A grid of cells at least `size` wide over the box `[lo, hi]`, empty.
    fn over(lo: Point, hi: Point, size: f64) -> Grid<T> {
        let (w, h) = ((hi[0] - lo[0]).max(0.0), (hi[1] - lo[1]).max(0.0));
        let size = size.max((w * h / MOST_CELLS).sqrt()).max(w / MOST_CELLS).max(h / MOST_CELLS);
        let (columns, rows) = ((w / size).floor() as usize + 1, (h / size).floor() as usize + 1);
        Grid { origin: lo, size, columns, rows, starts: vec![0; columns * rows + 1], items: Vec::new() }
    }

    /// The column and row of `p`, which may lie outside.
    fn place(&self, p: Point) -> [i64; 2] {
        [
            ((p[0] - self.origin[0]) / self.size).floor() as i64,
            ((p[1] - self.origin[1]) / self.size).floor() as i64,
        ]
    }

    /// The cell at column `i` and row `j`, if within the grid.
    fn cell(&self, [i, j]: [i64; 2]) -> Option<usize> {
        let (i, j) = (usize::try_from(i).ok()?, usize::try_from(j).ok()?);
        (i < self.columns && j < self.rows).then_some(j * self.columns + i)
    }

    /// The centre of the cell at column `i` and row `j`.
    fn centre(&self, [i, j]: [i64; 2]) -> Point {
        [self.origin[0] + (i as f64 + 0.5) * self.size, self.origin[1] + (j as f64 + 0.5) * self.size]
    }

    /// Files `filed`, each item in its cell, keeping their order within each: counted by cell,
    /// then laid out cell after cell.
    fn fill(&mut self, filed: &[(usize, T)]) {
        let mut next = vec![0; self.starts.len()];
        for &(c, _) in filed {
            if let Some(n) = next.get_mut(c + 1) {
                *n += 1;
            }
        }
        let mut total = 0;
        for n in &mut next {
            total += *n;
            *n = total;
        }
        self.starts.clone_from(&next);
        let mut order = vec![0; total];
        for (e, &(c, _)) in filed.iter().enumerate() {
            if let Some(n) = next.get_mut(c) {
                if let Some(slot) = order.get_mut(*n) {
                    *slot = e;
                }
                *n += 1;
            }
        }
        self.items = order.iter().filter_map(|&e| filed.get(e)).map(|&(_, item)| item).collect();
    }

    /// The items of the cell `c`.
    fn items(&self, c: usize) -> &[T] {
        let (Some(&from), Some(&to)) = (self.starts.get(c), self.starts.get(c + 1)) else { return &[] };
        self.items.get(from..to).unwrap_or(&[])
    }

    /// The items of the cell holding `p`; none outside the grid.
    fn at(&self, p: Point) -> &[T] {
        self.cell(self.place(p)).map_or(&[], |c| self.items(c))
    }
}

/// How deep the face cut into a layer's top, sampled across its beads' strips.
#[derive(Default)]
pub(crate) struct Cuts(Option<Grid<(Point, f64)>>);

impl Cuts {
    /// The cut at `p`: that of the nearest point sampled within a cell, which lies in the strip `p`
    /// lies in; none off the beads.
    fn at(&self, p: Point) -> f64 {
        let Some(grid) = &self.0 else { return 0.0 };
        let [i, j] = grid.place(p);
        let mut nearest = (grid.size * grid.size, 0.0);
        for di in -1..=1 {
            for dj in -1..=1 {
                let Some(c) = grid.cell([i.saturating_add(di), j.saturating_add(dj)]) else { continue };
                for &(q, cut) in grid.items(c) {
                    let d = (q[0] - p[0]) * (q[0] - p[0]) + (q[1] - p[1]) * (q[1] - p[1]);
                    if d <= nearest.0 {
                        nearest = (d, cut);
                    }
                }
            }
        }
        nearest.1
    }
}

/// Every straight stretch the nozzle passes over a layer's `moves`, in order, with the first of
/// each move's; a helix as chords, a move from where the nozzle is unknown as its end.
fn passes(moves: &[Move]) -> (Vec<Pass>, Vec<usize>) {
    let (mut passes, mut first) = (Vec::new(), Vec::with_capacity(moves.len()));
    for (from, m) in chained(moves, None) {
        first.push(passes.len());
        let Some(to) = m.to else { continue };
        let from = from.unwrap_or(to);
        let chords =
            m.arc.map_or(1.0, |arc| (arc.turn.abs() / std::f64::consts::TAU * CHORDS_PER_TURN).ceil());
        let n = chords.clamp(1.0, CHORDS_PER_TURN) as usize;
        let mut a = from;
        for k in 1..=n {
            let b = if k == n { to } else { m.point(from, k as f64 / n as f64) };
            passes.push(Pass::new(a, b));
            a = b;
        }
    }
    (passes, first)
}

/// The passes that may stand below the layer's top, `highest`, over a grid of cells `size` wide
/// across `[lo, hi]`: each filed in every cell whose centre lies within its face's `radius` and
/// half a cell's diagonal of it, so in each cell, in order, every pass whose face may cover a
/// point of it, with the lowest it and those after it in the cell stand.
fn file(passes: &[Pass], radius: f64, [lo, hi]: [Point; 2], size: f64, highest: f64) -> Grid<(usize, f64)> {
    let mut grid = Grid::over(lo, hi, size);
    let reach = radius + grid.size * std::f64::consts::FRAC_1_SQRT_2;
    let (mut filed, mut cells) = (Vec::new(), Vec::new());
    for (k, pass) in passes.iter().enumerate().filter(|(_, pass)| pass.low < highest) {
        let at = |u: f64| [pass.a[0] + u * (pass.b[0] - pass.a[0]), pass.a[1] + u * (pass.b[1] - pass.a[1])];
        // A pass across the plate is cut into pieces a cell long, each looking about itself.
        let n = ((pass.b[0] - pass.a[0]).hypot(pass.b[1] - pass.a[1]) / grid.size).ceil().clamp(1.0, 1e6);
        cells.clear();
        for piece in 0..n as usize {
            let (p, q) = (at(piece as f64 / n), at((piece + 1) as f64 / n));
            let [i0, j0] = grid.place([p[0].min(q[0]) - reach, p[1].min(q[1]) - reach]);
            let [i1, j1] = grid.place([p[0].max(q[0]) + reach, p[1].max(q[1]) + reach]);
            for j in j0.max(0)..=j1.min(grid.rows as i64 - 1) {
                for i in i0.max(0)..=i1.min(grid.columns as i64 - 1) {
                    if let Some(c) = grid.cell([i, j]).filter(|_| pass.within(grid.centre([i, j]), reach)) {
                        cells.push(c);
                    }
                }
            }
        }
        cells.sort_unstable();
        cells.dedup();
        filed.extend(cells.iter().map(|&c| (c, (k, pass.low))));
    }
    grid.fill(&filed);
    for c in 0..grid.columns * grid.rows {
        let (Some(&from), Some(&to)) = (grid.starts.get(c), grid.starts.get(c + 1)) else { continue };
        let mut floor = f64::INFINITY;
        for (_, low) in grid.items.get_mut(from..to).unwrap_or(&mut []).iter_mut().rev() {
            floor = floor.min(*low);
            *low = floor;
        }
    }
    grid
}

/// The lowest the face stands over `p` from the fraction `from` of pass `k` on, if lower than
/// `z`.
fn lowest(
    passes: &[Pass],
    index: &Grid<(usize, f64)>,
    k: usize,
    from: f64,
    p: Point,
    radius: f64,
    z: f64,
) -> f64 {
    let filed = index.at(p);
    let mut low = z;
    for &(j, floor) in filed.get(filed.partition_point(|&(j, _)| j < k)..).unwrap_or(&[]) {
        // Nothing after in the cell stands lower.
        if floor >= low {
            break;
        }
        let Some(pass) = passes.get(j).filter(|pass| pass.low < low) else { continue };
        let from = if j == k { from } else { 0.0 };
        low = pass.lowest_over(from, p, radius).map_or(low, |z| low.min(z));
    }
    low
}

/// The width of the strip a bead fills: as wide as beads lie apart; a round strand, its own.
fn strip(bead: &Bead, height: f64) -> f64 {
    if bead.role.strand() { bead.width } else { bead_spacing(bead.width, height) }
}

/// Scales the plastic of each bead among a layer's `moves` on `leaf` to the room it keeps, the
/// face `radius` round passing over it later, on a top cut by `below`: the cuts the face leaves.
/// A strand, hanging, keeps its plastic, the cuts into it noted all the same.
pub(crate) fn keep(moves: &mut [Move], leaf: &Leaf, below: &Cuts, radius: f64) -> Cuts {
    let bends = radius > 0.0 && !leaf.warp.is_zero();
    if !bends && below.0.is_none() {
        return Cuts::default();
    }
    // The beads, and the box their strips lie in.
    let beads: Vec<(usize, Point3, Point3, f64)> = chained(moves, None)
        .enumerate()
        .filter_map(|(i, (from, m))| {
            let (a, b, bead) = (from?, m.to?, m.bead?);
            let laid = m.e > 0.0 && m.arc.is_none() && (b[0] - a[0]).hypot(b[1] - a[1]) > 0.0;
            laid.then(|| (i, a, b, strip(&bead, leaf.height)))
        })
        .collect();
    let widest = beads.iter().map(|&(_, _, _, s)| s).fold(0.0, f64::max);
    let mut bounds = [[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]];
    for &(_, a, b, _) in &beads {
        bounds = [
            [bounds[0][0].min(a[0].min(b[0]) - widest), bounds[0][1].min(a[1].min(b[1]) - widest)],
            [bounds[1][0].max(a[0].max(b[0]) + widest), bounds[1][1].max(a[1].max(b[1]) + widest)],
        ];
    }
    if beads.is_empty() || widest <= 0.0 {
        return Cuts::default();
    }
    let (passes, first) = passes(moves);
    let index = bends.then(|| file(&passes, radius, bounds, radius.max(widest) / 2.0, leaf.highest()));
    let mut samples = Vec::new();
    let mut cut_any = false;
    for (i, a, b, s) in beads {
        let (d, k) = ([b[0] - a[0], b[1] - a[1]], first.get(i).copied().unwrap_or(0));
        let length = d[0].hypot(d[1]);
        let across = [-d[1] / length, d[0] / length];
        let n = (3.0 * length / s).ceil().clamp(1.0, 1e6) as usize;
        let (mut slab, mut room) = (0.0, 0.0);
        for j in 0..n {
            let u = (j as f64 + 0.5) / n as f64;
            let (c, z) = ([a[0] + u * d[0], a[1] + u * d[1]], a[2] + u * (b[2] - a[2]));
            for o in [-s / 3.0, 0.0, s / 3.0] {
                let p = [c[0] + o * across[0], c[1] + o * across[1]];
                let cut = index.as_ref().map_or(0.0, |index| z - lowest(&passes, index, k, u, p, radius, z));
                let t = leaf.thickness(p);
                slab += t;
                room += (t + below.at(p) - cut).max(0.0);
                if index.is_some() {
                    samples.push((p, cut));
                    cut_any |= cut > 0.0;
                }
            }
        }
        let Some(m) = moves.get_mut(i).filter(|m| m.bead.is_some_and(|b| !b.role.strand())) else { continue };
        if slab > 0.0 {
            m.e *= room / slab;
        }
    }
    if !cut_any {
        return Cuts::default();
    }
    // A point of a strip lies within half the diagonal of a sampled cell, at most a third of the
    // strip on a side, of its nearest sample.
    let mut grid = Grid::over(bounds[0], bounds[1], widest * std::f64::consts::SQRT_2 / 6.0);
    let filed: Vec<(usize, (Point, f64))> =
        samples.iter().filter_map(|&(p, cut)| Some((grid.cell(grid.place(p))?, (p, cut)))).collect();
    grid.fill(&filed);
    Cuts(Some(grid))
}
