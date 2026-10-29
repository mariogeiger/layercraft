//! Sparse infill as convection rolls: the lines are where a field `u` over the layer vanishes,
//! `u` flowing down an energy whose minima are rolls `d` apart (`field`). Walls no flux crosses,
//! which the rolls meet at right angles, or tilted off square, hold them from wall to wall
//! (`walls`): struts, welded at both ends, that bend round only where the area does, with no
//! loose end inside. Each layer's field starts from the one below and is pulled toward it, so its
//! lines rest on the lines below; the tilt changes way every `ply` of height, and each way's field
//! goes on from the last layer it lay on, two plies down. The first field starts from a hash of
//! its grid's nodes, the grid lying in the part's own frame (`frame`), so a part turned and moved
//! gets the same lines, turned and moved. The lines are the field's zeros (`contour`), cut to the
//! area.

mod contour;
mod field;
mod frame;
mod walls;

use field::Flow;
use frame::Frame;
use layercraft_geom::{GeomError, Point, Polyline, Region, clip_lines};
use layercraft_region::RegionLayer;
use walls::Walls;

/// The grid's nodes per wavelength of the rolls, two lines.
const NODES_PER_WAVE: f64 = 12.0;
/// The most nodes a grid holds: a part too large for its rolls keeps straight lines.
const MOST_NODES: usize = 1 << 22;
/// The steps from the seed, on the layer below, and from the same way's field two plies down.
const FIRST_STEPS: usize = 600;
const STEPS: usize = 25;
const PLY_STEPS: usize = 100;
/// How hard the field is held to nothing beyond its walls.
const HOLD: f64 = 3.0;

/// The nodes of a grid, `h` apart from `origin` in the frame, `nx` along a row, `ny` rows.
#[derive(Debug, Clone, Copy)]
struct Nodes {
    origin: Point,
    h: f64,
    nx: usize,
    ny: usize,
}

impl Nodes {
    fn len(&self) -> usize {
        self.nx * self.ny
    }

    fn node(&self, i: usize, j: usize) -> Point {
        [self.origin[0] + i as f64 * self.h, self.origin[1] + j as f64 * self.h]
    }

    /// The node at or below and left of `p`, which may lie off the grid.
    fn place(&self, p: Point) -> [i64; 2] {
        [((p[0] - self.origin[0]) / self.h).floor() as i64, ((p[1] - self.origin[1]) / self.h).floor() as i64]
    }
}

/// The least whole number from `n` on whose only prime factors are 2, 3 and 5: fast transforms.
fn smooth(n: usize) -> usize {
    let five_smooth = |mut m: usize| {
        for p in [2, 3, 5] {
            while m.is_multiple_of(p) {
                m /= p;
            }
        }
        m == 1
    };
    (n.max(1)..).find(|&m| five_smooth(m)).unwrap_or(n)
}

/// A value in `[−½, ½)` from a node's indices from the frame's origin: splitmix64.
fn hash(i: i64, j: i64) -> f64 {
    let mut z =
        (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ (j as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    (z >> 11) as f64 / (1u64 << 53) as f64 - 0.5
}

/// One way of the tilt's field, as it lay on its last layer: its values, and where it was free.
struct Ply {
    u: Vec<f64>,
    layer: usize,
    live: Vec<bool>,
}

/// The rolls of a part's sparse areas, layer after layer.
pub(crate) struct Rolls {
    frame: Frame,
    nodes: Nodes,
    flow: Flow,
    tilt: f64,
    ply: f64,
    plies: [Option<Ply>; 2],
}

impl Rolls {
    /// Rolls `spacing` apart over the sparse areas of `layers` grown by `grow`, tilted `tilt`
    /// degrees off square to the walls, the other way every `ply` of height; none when the
    /// layers have no sparse area or the grid would be too large.
    pub fn new(layers: &[RegionLayer], spacing: f64, grow: f64, tilt: f64, ply: f64) -> Option<Rolls> {
        if !(spacing.is_finite() && spacing > 0.0 && grow.is_finite() && tilt.is_finite() && ply.is_finite())
        {
            return None;
        }
        let frame = Frame::of(layers.iter().map(|l| (&l.sparse, l.height)));
        let points = layers.iter().flat_map(|l| l.sparse.iter().flatten().flatten()).map(|&p| frame.local(p));
        let [lo, hi] = points.fold([[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]], |[lo, hi], p| {
            [[lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]]
        });
        if !(lo[0] <= hi[0] && lo[1] <= hi[1]) {
            return None;
        }
        let wave = 2.0 * spacing;
        let h = wave / NODES_PER_WAVE;
        // Room beyond the walls for the margin, and for the damped field between the copies of
        // the periodic grid.
        let pad = grow.max(0.0) + wave / 4.0 + 1.5 * wave;
        let origin = [((lo[0] - pad) / h).floor() * h, ((lo[1] - pad) / h).floor() * h];
        let count = |lo: f64, hi: f64| ((hi - lo) / h).ceil();
        let (nx, ny) = (count(origin[0], hi[0] + pad), count(origin[1], hi[1] + pad));
        if nx * ny > MOST_NODES as f64 || (nx * ny).is_nan() {
            return None;
        }
        let nodes = Nodes { origin, h, nx: smooth(nx as usize + 1), ny: smooth(ny as usize + 1) };
        (nodes.len() <= MOST_NODES).then(|| Rolls {
            frame,
            nodes,
            flow: Flow::new(nodes),
            tilt: tilt.to_radians(),
            ply,
            plies: [None, None],
        })
    }

    /// The field's first values: a hash of each node's indices from the frame's origin, faint.
    fn seed(&self) -> Vec<f64> {
        let (i0, j0) = (
            (self.nodes.origin[0] / self.nodes.h).round() as i64,
            (self.nodes.origin[1] / self.nodes.h).round() as i64,
        );
        (0..self.nodes.len())
            .map(|n| 0.1 * hash(i0 + (n % self.nodes.nx) as i64, j0 + (n / self.nodes.nx) as i64))
            .collect()
    }

    /// The lines of a layer's sparse `area`, rolls `spacing` apart: the layer's field relaxed,
    /// its zeros cut to the area. Layers come in order.
    pub fn lines(
        &mut self,
        layer: &RegionLayer,
        area: &Region,
        spacing: f64,
    ) -> Result<Vec<Polyline>, GeomError> {
        if area.is_empty() || !(spacing.is_finite() && spacing > 0.0) {
            return Ok(Vec::new());
        }
        let contours: Vec<Vec<Point>> =
            area.iter().flatten().map(|c| c.iter().map(|&p| self.frame.local(p)).collect()).collect();
        let tilted = self.ply > 0.0 && self.tilt != 0.0;
        let way = if tilted { ((layer.top / self.ply).floor() as i64).rem_euclid(2) as usize } else { 0 };
        let tilt = if way == 0 { self.tilt } else { -self.tilt };
        let wave = 2.0 * spacing;
        let walls = Walls::new(&self.nodes, &contours, wave / 4.0, wave / 4.0, HOLD, tilt);
        let last = self.plies.get_mut(way).and_then(Option::take);
        let (mut u, below, steps) = match last {
            Some(ply) if ply.layer + 1 == layer.index => {
                let below = (ply.u.clone(), ply.live);
                (ply.u, Some(below), STEPS)
            }
            Some(ply) => (ply.u, None, PLY_STEPS),
            None => (self.seed(), None, FIRST_STEPS),
        };
        let pull = below.as_ref().map(|(u, live)| (u.as_slice(), live.as_slice()));
        self.flow.relax(&mut u, &walls, std::f64::consts::PI / spacing, steps, pull);
        let live = walls.live();
        let curves: Vec<Polyline> = contour::zeros(&self.nodes, &u, &live)
            .into_iter()
            .map(|c| c.into_iter().map(|q| self.frame.plate(q)).collect())
            .collect();
        if let Some(slot) = self.plies.get_mut(way) {
            *slot = Some(Ply { u, layer: layer.index, live });
        }
        clip_lines(&curves, area)
    }
}
