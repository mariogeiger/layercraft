//! A dense area's beads as families of parallel curves, with the fewest bead ends there can be.
//!
//! Beads `s` apart are the level sets of a phase rising one bead per `s`: they lie exactly `s`
//! apart, each holding the plastic of its band, on a family of parallel curves, straight lines or
//! the insets of the area's edge. A bead ends where its curve meets the edge across it or a curve
//! of another family, and each end lays a blob or a gap; and a layer laid along the beads below
//! instead of across them bonds the less. An area costs its ends, plus the area it lays along the
//! layer below, `∫|cos ∠|` over `s·W*`: across a band `W` wide, lines end twice per `s` of its
//! length; along it, they lie on those below over `W`. So a band narrower than `W* = v²/a`, the run
//! a bead needs to reach its speed and stop, runs along itself on every layer, and a wider area
//! crosses the layer below. The bands narrower than `W*` are what an opening by `W*/2` takes away;
//! they and the rest are the area's cells, each laid by one family. Over the angle of straight
//! lines both costs are least at a kink, along an edge or across the beads below: a cell tries the
//! classes of directions its edges, its neighbours' and the beads below it run along, its area's
//! first axis, and, for a band, the insets. Where a cell meets one of its family, the curves go
//! on: no end. Each cell starts from its best alone, then each in turn, largest first, takes the
//! family that costs least beside its neighbours', until none changes; ties go to the insets, then
//! nearest the area's frame. Lines are centred across their area; the insets lie `s/2 + k·s` deep,
//! and the strips they leave, narrower than two beads, go to the gap fill. On a bent layer's top,
//! lines follow the fall line. A top surface also pays for the joins it shows (`kinks`).

use crate::frame::Frame;
use layercraft_geom::{
    Contour, Erosions, GeomError, Moments, Point, Polyline, RESOLUTION_MM, Region, Relief, bounds,
    clip_lines, difference, disc_offset, disc_opening, edges, encloses, hatch, intersection, polyline_length,
    segment_distance, union, winding_region,
};
use std::cmp::Ordering;

/// How far past a curve's end it is looked for going on, in mm: over the lattice's rounding of
/// where it was cut, under any bead.
const STEP_MM: f64 = 1e-4;

/// The classes of directions that a cell's edges, and the beads below it, run along most, which
/// it tries.
const DIRECTIONS_TRIED: usize = 3;

/// Directions closer than this, in degrees, are one.
const SAME_DIRECTION: f64 = 0.5;

/// Weights of directions closer than this share are a tie: a part per million, the lattice's
/// nanometre in a millimetre.
const SAME_WEIGHT: f64 = 1e-6;

/// The most rounds of the cells' choices.
const MAX_ROUNDS: usize = 16;

/// Costs closer than this, in bead ends, are a tie: a thousandth of an end, over what the
/// lattice's rounding moves the area laid along the layer below.
const TIE: f64 = 1e-3;

/// The weight of area laid along the layer below when nothing may be: every area crosses.
const ALWAYS_CROSS: f64 = 1e12;

/// The mean slope under which a piece counts as flat: a degree.
const FLAT_SLOPE: f64 = 0.017_455_064_928_217_585;

/// What an end at the area's edge costs on a top surface at the least, in ends: the wall it overlaps
/// hides it, where a bend or an end against another family lies open on the surface.
const HIDDEN_END: f64 = 0.25;

/// The family of curves a cell's beads follow.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Family {
    /// The insets of the area's edge.
    Insets,
    /// Straight lines at this angle, in degrees, in [0, 180).
    Lines(f64),
}

/// The dense beads of the layer below: segments, each with the width it fills.
#[derive(Debug, Default)]
pub struct Beneath {
    segments: Vec<(Point, Point, f64)>,
}

impl Beneath {
    pub fn new(segments: Vec<(Point, Point, f64)>) -> Beneath {
        Beneath { segments }
    }
}

/// A dense area's beads, `s` apart.
#[derive(Debug, Default)]
pub struct Dense {
    /// Straight lines.
    pub lines: Vec<Polyline>,
    /// The insets' beads, each closed when it is a whole loop.
    pub insets: Vec<(Polyline, bool)>,
    /// What the insets leave, narrower than two beads: for the gap fill.
    pub residue: Region,
}

/// How an area is laid: beads `spacing` apart and `width` wide, crossing the layer below where
/// wider than `crossing`; `shows` when it is a top surface, whose joins are seen.
#[derive(Debug, Clone, Copy)]
pub struct Spacing {
    pub spacing: f64,
    pub width: f64,
    pub crossing: f64,
    pub shows: bool,
}

/// The direction of a segment, in degrees, and its length.
fn direction(a: Point, b: Point) -> (f64, f64) {
    ((b[1] - a[1]).atan2(b[0] - a[0]).to_degrees(), (b[0] - a[0]).hypot(b[1] - a[1]))
}

/// How far apart two directions of lines are, in degrees: within a half turn.
fn apart(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(180.0);
    d.min(180.0 - d)
}

/// Lines at `a` and at `b` degrees in the order ties go in, beside `frame`: nearest its axes
/// first, then nearest its first, then on from it towards its second.
fn frame_order(a: f64, b: f64, frame: &Frame) -> Ordering {
    let (first, sense) = (frame.angle(), if frame.right_handed() { 1.0 } else { -1.0 });
    let key = |x: f64| {
        let d = apart(x, first);
        (d.min(90.0 - d), d, (sense * (x - first)).rem_euclid(180.0))
    };
    let (x, y) = (key(a), key(b));
    x.0.total_cmp(&y.0).then(x.1.total_cmp(&y.1)).then(x.2.total_cmp(&y.2))
}

/// A class of directions, a quarter turn counting as none, so of lines along them and across:
/// its directions span `width` degrees on from `from`, about their mean `angle`, in [0, 90).
#[derive(Debug, Clone, Copy)]
struct Class {
    angle: f64,
    from: f64,
    width: f64,
}

impl Class {
    /// How far a direction, in degrees, lies outside the class.
    fn gap(&self, a: f64) -> f64 {
        let t = (a - self.from).rem_euclid(90.0);
        if t <= self.width { 0.0 } else { (t - self.width).min(90.0 - t) }
    }
}

/// Directions in degrees with their weights, in classes: those closer than `SAME_DIRECTION`
/// chain into one, at the mean of their weights taken as `4θ` round the circle, a quarter turn
/// being a whole one. The chains start past the widest gap, which the directions place, not the
/// plate.
fn classes(items: &[(f64, f64)]) -> Vec<Class> {
    let mut all: Vec<(f64, f64)> = items
        .iter()
        .filter(|(a, w)| a.is_finite() && w.is_finite() && *w > 0.0)
        .map(|&(a, w)| (a.rem_euclid(90.0), w))
        .collect();
    all.sort_by(|x, y| x.0.total_cmp(&y.0));
    let n = all.len();
    let at = |i: usize| all.get(i % n.max(1)).copied().unwrap_or_default();
    // From each direction on to the next, round the quarter turn.
    let gap = |i: usize| if n == 1 { 90.0 } else { (at(i + 1).0 - at(i).0).rem_euclid(90.0) };
    let Some(widest) = (0..n).max_by(|&i, &j| gap(i).total_cmp(&gap(j))) else { return Vec::new() };
    let close = |(from, width, sum): (f64, f64, [f64; 2])| Class {
        angle: (sum[1].atan2(sum[0]).to_degrees() / 4.0).rem_euclid(90.0),
        from,
        width,
    };
    let mut out: Vec<Class> = Vec::new();
    // The chain open: where it starts, how far on it reaches, and its weights' sum as `4θ`.
    let mut open: Option<(f64, f64, [f64; 2])> = None;
    for i in widest + 1..=widest + n {
        let (a, w) = at(i);
        let (sin, cos) = (4.0 * a).to_radians().sin_cos();
        open = match open {
            Some((from, _, [x, y])) if gap(i - 1) < SAME_DIRECTION => {
                Some((from, (a - from).rem_euclid(90.0), [x + w * cos, y + w * sin]))
            }
            done => {
                out.extend(done.map(close));
                Some((a, 0.0, [w * cos, w * sin]))
            }
        };
    }
    out.extend(open.map(close));
    out
}

/// The class of a direction in degrees: the nearest of `classes`, within `SAME_DIRECTION`.
fn class_of(classes: &[Class], a: f64) -> Option<usize> {
    let gaps = classes.iter().map(|c| c.gap(a)).enumerate();
    gaps.filter(|(_, g)| *g < SAME_DIRECTION).min_by(|x, y| x.1.total_cmp(&y.1)).map(|(k, _)| k)
}

/// The classes that `items`, directions with their weights, run along most: the
/// `DIRECTIONS_TRIED` heaviest, and any as heavy as the last of them.
fn heaviest(classes: &[Class], items: impl Iterator<Item = (f64, f64)>) -> Vec<usize> {
    let mut weights = vec![0.0; classes.len()];
    for (a, w) in items {
        if let Some(total) = class_of(classes, a).and_then(|k| weights.get_mut(k)) {
            *total += w;
        }
    }
    let mut ranked: Vec<(usize, f64)> = weights.into_iter().enumerate().filter(|(_, w)| *w > 0.0).collect();
    ranked.sort_by(|x, y| y.1.total_cmp(&x.1));
    let last = ranked.get(DIRECTIONS_TRIED - 1).map_or(0.0, |r| r.1 * (1.0 - SAME_WEIGHT));
    ranked.into_iter().take_while(|r| r.1 >= last).map(|r| r.0).collect()
}

/// One shape of the area, which decides its lines: their phase, the directions they may take
/// and the order of ties, its frame's.
struct Component {
    outer: Contour,
    frame: Frame,
    edges: Vec<(Point, Point)>,
    /// The classes of its edges' directions, then of those of the beads below it, off them.
    classes: Vec<Class>,
    /// The fall line of a bent top over it.
    fall: Option<f64>,
}

impl Component {
    fn new(shape: &[Contour], fall: &dyn Fn(&Region) -> Option<f64>) -> Component {
        let region = vec![shape.to_vec()];
        let edges: Vec<(Point, Point)> = shape.iter().flat_map(|c| edges(c)).collect();
        let directions: Vec<(f64, f64)> = edges.iter().map(|&(a, b)| direction(a, b)).collect();
        Component {
            outer: shape.first().cloned().unwrap_or_default(),
            frame: Frame::of(std::iter::once((&region, 1.0))),
            classes: classes(&directions),
            fall: fall(&region),
            edges,
        }
    }

    /// Takes in the classes of the directions of `segments`, the beads below it with their
    /// widths, that none of its own holds.
    fn add_below<'a>(&mut self, segments: impl Iterator<Item = &'a (Point, Point, f64)>) {
        let loose: Vec<(f64, f64)> = segments
            .map(|&(a, b, w)| {
                let (d, l) = direction(a, b);
                (d, l * w)
            })
            .filter(|&(d, _)| class_of(&self.classes, d).is_none())
            .collect();
        self.classes.extend(classes(&loose));
    }

    /// Its frame's first axis, in degrees within [0, 90): its class's when it lies within one.
    fn axis(&self) -> f64 {
        let angle = self.frame.angle();
        let class = class_of(&self.classes, angle).and_then(|k| self.classes.get(k));
        class.map_or(angle.rem_euclid(90.0), |c| c.angle)
    }

    /// Where lines at `angle` lie across the component, `s` apart and centred: the distance across
    /// them of one, as `hatch` takes it.
    fn phase(&self, angle: f64, s: f64) -> f64 {
        let (sin, cos) = angle.to_radians().sin_cos();
        let across = |p: &Point| -p[0] * sin + p[1] * cos;
        let (lo, hi) =
            self.outer.iter().map(across).fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v), b.max(v)));
        let n = ((hi - lo) / s).floor().max(0.0);
        lo + (hi - lo - n * s) / 2.0 + s / 2.0
    }

    /// The direction, in radians, of the edge nearest `p`: the insets' there.
    fn inset_direction(&self, p: Point) -> f64 {
        let nearest = self
            .edges
            .iter()
            .map(|&(a, b)| (segment_distance(p, a, b), a, b))
            .min_by(|x, y| x.0.total_cmp(&y.0));
        nearest.map_or(0.0, |(_, a, b)| (b[1] - a[1]).atan2(b[0] - a[0]))
    }
}

/// A family a cell may take, and what it costs there.
struct Choice {
    family: Family,
    /// The ends on the area's edge, by what each costs.
    edge_ends: f64,
    /// What its bends cost, where the area shows.
    kinks: f64,
    /// The ends on each neighbour's edge, by its host.
    ends_to: Vec<(usize, f64)>,
    /// The area laid along the beads below, over `s·W*`.
    along_below: f64,
}

/// A cell of the area: a band narrower than `W*`, or a piece of the rest.
struct Cell {
    region: Region,
    box_: [Point; 2],
    area: f64,
    /// Its centroid in its area's frame, which orders the cells of one area.
    centre: Point,
    thin: bool,
    component: usize,
    /// The cells it borders, with the length they share.
    neighbours: Vec<(usize, f64)>,
    /// The beads below it, with their widths.
    below: Vec<(Point, Point, f64)>,
    choices: Vec<Choice>,
    chosen: usize,
}

impl Cell {
    fn holds(&self, q: Point) -> bool {
        let [lo, hi] = self.box_;
        lo[0] <= q[0]
            && q[0] <= hi[0]
            && lo[1] <= q[1]
            && q[1] <= hi[1]
            && encloses(self.region.iter().flatten().flat_map(|c| edges(c)), q)
    }

    fn family(&self) -> Option<Family> {
        self.choices.get(self.chosen).map(|c| c.family)
    }

    /// The directions of its edges, with their lengths.
    fn edge_directions(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.region.iter().flatten().flat_map(|c| edges(c)).map(|(a, b)| direction(a, b))
    }

    /// The directions of the beads below it, each with the area it fills.
    fn below_directions(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.below.iter().map(|&(a, b, w)| {
            let (d, l) = direction(a, b);
            (d, l * w)
        })
    }
}

/// The cell holding `q`, other than `except`.
fn holder(cells: &[Cell], q: Point, except: usize) -> Option<usize> {
    cells.iter().enumerate().find(|(i, c)| *i != except && c.holds(q)).map(|(i, _)| i)
}

/// A point just inside a shape, off the middle of its outer contour's first edge, on its left.
fn inside(shape: &[Contour]) -> Option<Point> {
    let outer = shape.first()?;
    let (a, b) = edges(outer).next()?;
    let (d, l) = ([b[0] - a[0], b[1] - a[1]], (b[0] - a[0]).hypot(b[1] - a[1]));
    (l > 0.0).then(|| [(a[0] + b[0]) / 2.0 - STEP_MM * d[1] / l, (a[1] + b[1]) / 2.0 + STEP_MM * d[0] / l])
}

/// The beads of `beneath` within `region`, whose box is `box_`, cut at its edge.
fn below_within(
    region: &Region,
    box_: [Point; 2],
    beneath: &Beneath,
) -> Result<Vec<(Point, Point, f64)>, GeomError> {
    let [lo, hi] = box_;
    // By width, so that each width is cut once.
    let mut by_width: Vec<(f64, Vec<Polyline>)> = Vec::new();
    for &(a, b, width) in &beneath.segments {
        if a[0].max(b[0]) < lo[0]
            || a[0].min(b[0]) > hi[0]
            || a[1].max(b[1]) < lo[1]
            || a[1].min(b[1]) > hi[1]
        {
            continue;
        }
        match by_width.iter_mut().find(|(w, _)| *w == width) {
            Some((_, lines)) => lines.push(vec![a, b]),
            None => by_width.push((width, vec![vec![a, b]])),
        }
    }
    let mut out = Vec::new();
    for (width, lines) in by_width {
        for piece in clip_lines(&lines, region)? {
            out.extend(piece.windows(2).filter_map(|pair| Some((*pair.first()?, *pair.get(1)?, width))));
        }
    }
    Ok(out)
}

/// The cells of an area: the shapes of `thick`, where it is wider than `W*`, and of `thin`, the
/// rest, each in its component and over its beads `beneath`, with the cells it borders.
fn cells_of(
    thick: &Region,
    thin: &Region,
    components: &[Component],
    beneath: &Beneath,
) -> Result<Vec<Cell>, GeomError> {
    let mut cells: Vec<Cell> = Vec::new();
    for (shape, thin) in thick.iter().map(|s| (s, false)).chain(thin.iter().map(|s| (s, true))) {
        let region = vec![shape.clone()];
        let (Some(box_), Some(q)) = (bounds(&region), inside(shape)) else { continue };
        let component = components.iter().position(|c| encloses(c.edges.iter().copied(), q)).unwrap_or(0);
        let centroid = Moments::of(std::iter::once((shape.as_slice(), 1.0))).map_or(q, |m| m.centroid);
        let centre = components.get(component).map_or(centroid, |c| c.frame.local(centroid));
        let below = below_within(&region, box_, beneath)?;
        let area = layercraft_geom::area(&region);
        cells.push(Cell {
            region,
            box_,
            area,
            centre,
            thin,
            component,
            neighbours: Vec::new(),
            below,
            choices: Vec::new(),
            chosen: 0,
        });
    }
    // Who borders whom: past the middle of each edge, on its right, outside the cell.
    for i in 0..cells.len() {
        let mut touching: Vec<(usize, f64)> = Vec::new();
        let Some(cell) = cells.get(i) else { continue };
        for (a, b) in cell.region.iter().flatten().flat_map(|c| edges(c)) {
            let (d, l) = ([b[0] - a[0], b[1] - a[1]], (b[0] - a[0]).hypot(b[1] - a[1]));
            if l <= 0.0 {
                continue;
            }
            let q = [(a[0] + b[0]) / 2.0 + STEP_MM * d[1] / l, (a[1] + b[1]) / 2.0 - STEP_MM * d[0] / l];
            if let Some(n) = holder(&cells, q, i) {
                match touching.iter_mut().find(|(m, _)| *m == n) {
                    Some((_, shared)) => *shared += l,
                    None => touching.push((n, l)),
                }
            }
        }
        if let Some(cell) = cells.get_mut(i) {
            cell.neighbours = touching;
        }
    }
    Ok(cells)
}

/// Each cell's host, whose family it takes: itself, or for a sliver thinner than half a bead,
/// which holds none of its own, the neighbour it shares most edge with.
fn hosts(cells: &[Cell], s: f64) -> Vec<usize> {
    let sliver = |c: &Cell| {
        let perimeter: f64 = c.region.iter().flatten().map(|c| polyline_length(&closed(c))).sum();
        perimeter > 0.0 && 2.0 * c.area / perimeter < s / 2.0
    };
    let slivers: Vec<bool> = cells.iter().map(sliver).collect();
    (0..cells.len())
        .map(|i| {
            let own = cells.get(i).map(|c| c.neighbours.as_slice()).unwrap_or(&[]);
            if slivers.get(i) != Some(&true) {
                return i;
            }
            own.iter()
                .filter(|(n, _)| slivers.get(*n) == Some(&false))
                .max_by(|x, y| x.1.total_cmp(&y.1))
                .map_or(i, |(n, _)| *n)
        })
        .collect()
}

/// The families each cell may take: none for a sliver; on a bent top, lines down its fall line;
/// else lines along and across the classes that its edges, its neighbours' and the beads below it
/// run along most, and its area's first axis, after the insets for a band, in the order ties go.
fn families(cells: &[Cell], host: &[usize], components: &[Component]) -> Vec<Vec<Family>> {
    let classes = |c: &Cell| components.get(c.component).map_or(&[][..], |k| k.classes.as_slice());
    let own: Vec<Vec<usize>> = cells.iter().map(|c| heaviest(classes(c), c.edge_directions())).collect();
    let mut out = Vec::with_capacity(cells.len());
    for (i, cell) in cells.iter().enumerate() {
        let Some(component) = components.get(cell.component).filter(|_| host.get(i) == Some(&i)) else {
            out.push(Vec::new());
            continue;
        };
        if let Some(angle) = component.fall {
            out.push(vec![Family::Lines(angle.rem_euclid(180.0))]);
            continue;
        }
        let mut picked: Vec<usize> = own.get(i).cloned().unwrap_or_default();
        for &(n, _) in &cell.neighbours {
            if cells.get(n).is_some_and(|c| c.component == cell.component) {
                picked.extend(own.get(n).into_iter().flatten());
            }
        }
        picked.extend(heaviest(&component.classes, cell.below_directions()));
        let angles = picked.iter().filter_map(|&k| component.classes.get(k)).map(|c| c.angle);
        let mut lines: Vec<f64> = Vec::new();
        for a in angles.chain([component.axis()]).flat_map(|a| [a, a + 90.0]) {
            if !lines.contains(&a) {
                lines.push(a);
            }
        }
        lines.sort_by(|&a, &b| frame_order(a, b, &component.frame));
        let insets = cell.thin.then_some(Family::Insets);
        out.push(insets.into_iter().chain(lines.into_iter().map(Family::Lines)).collect());
    }
    out
}

/// The direction of the fall line of `warp` over `piece`, in degrees, when its mean slope there is
/// past `FLAT_SLOPE`. Slopes are averaged as axes over the relief's triangles whose centre lies
/// in the piece, each weighed by its area, its gradient `g` at angle `φ` adding
/// `|g|²·(cos 2φ, sin 2φ)`, so that the two sides of a ridge agree on their line; a piece
/// holding no triangle's centre takes the gradient at its own.
pub(crate) fn fall_line(piece: &Region, warp: &Relief) -> Option<f64> {
    if warp.is_zero() {
        return None;
    }
    let [lo, hi] = bounds(piece)?;
    let edges: Vec<(Point, Point)> = piece.iter().flatten().flat_map(|c| edges(c)).collect();
    let mut samples: Vec<([f64; 2], f64)> = warp
        .facets_over(lo, hi)
        .filter(|([a, b, c], _)| {
            encloses(edges.iter().copied(), [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0])
        })
        .map(|([a, b, c], g)| {
            (g, ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])).abs() / 2.0)
        })
        .collect();
    if samples.is_empty() {
        let centre = Moments::of(piece.iter().map(|s| (s.as_slice(), 1.0)))?.centroid;
        samples.push((warp.gradient(centre), 1.0));
    }
    let (mut axis, mut slope, mut weight) = ([0.0f64; 2], 0.0, 0.0);
    for (g, w) in samples {
        axis = [axis[0] + w * (g[0] * g[0] - g[1] * g[1]), axis[1] + w * 2.0 * g[0] * g[1]];
        slope += w * g[0].hypot(g[1]);
        weight += w;
    }
    (weight > 0.0 && slope / weight > FLAT_SLOPE).then(|| axis[1].atan2(axis[0]).to_degrees() / 2.0)
}

/// An inset of an area `d` deep: the region past its bead, `d + w/2` deep, and the region it
/// edges, that grown back by `w/2`, the region `d` deep less what is narrower than the bead.
struct Inset {
    past: Region,
    centre: Region,
}

/// The insets of an area `s/2 + k·s` deep, the level sets of the distance to its edge, where a bead
/// `w` wide passes, as far as `depth` and as long as they reach into `bands`.
fn insets(area: &Region, bands: &Region, s: f64, w: f64, depth: f64) -> Result<Vec<Inset>, GeomError> {
    let mut out = Vec::new();
    let mut erosions = Erosions::new(area, depth + w / 2.0)?;
    let mut k = 0usize;
    while s / 2.0 + k as f64 * s <= depth {
        let past = erosions.at(s / 2.0 + k as f64 * s + w / 2.0)?;
        let centre = disc_offset(&past, w / 2.0)?;
        if intersection(&centre, bands)?.is_empty() {
            break;
        }
        out.push(Inset { past, centre });
        k += 1;
    }
    Ok(out)
}

/// The insets' contours, each closed back to its start, with its box.
struct Rings {
    lines: Vec<Polyline>,
    boxes: Vec<[Point; 2]>,
}

impl Rings {
    fn of(insets: &[Inset]) -> Rings {
        let contours = insets.iter().flat_map(|i| i.centre.iter().flatten());
        let lines: Vec<Polyline> = contours.filter(|c| c.len() > 1).map(closed).collect();
        let boxes = lines
            .iter()
            .map(|l| {
                l.iter().fold([[f64::MAX; 2], [f64::MIN; 2]], |[lo, hi], p| {
                    [[lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]]
                })
            })
            .collect();
        Rings { lines, boxes }
    }

    /// The rings whose box meets `box_`.
    fn near(&self, [lo, hi]: [Point; 2]) -> Vec<Polyline> {
        let meets = |[a, b]: &[Point; 2]| a[0] <= hi[0] && lo[0] <= b[0] && a[1] <= hi[1] && lo[1] <= b[1];
        self.lines.iter().zip(&self.boxes).filter(|(_, b)| meets(b)).map(|(l, _)| l.clone()).collect()
    }
}

/// Whether a piece of a loop is the whole loop: closed back on itself.
fn whole(piece: &[Point]) -> bool {
    piece.len() > 3 && piece.first() == piece.last()
}

/// The beads of a family over a cell: open pieces of lines, or of the insets' `rings`.
fn beads(
    family: Family,
    cell: &Cell,
    component: &Component,
    s: f64,
    rings: &Rings,
) -> Result<Vec<Polyline>, GeomError> {
    match family {
        Family::Lines(angle) => hatch(&cell.region, s, angle, component.phase(angle, s)),
        Family::Insets => clip_lines(&rings.near(cell.box_), &cell.region),
    }
}

/// The two ends of an open piece, each with the way out of it.
fn ends(piece: &[Point]) -> Vec<(Point, [f64; 2])> {
    let out = |end: Option<&Point>, before: Option<&Point>| {
        let (e, b) = (end?, before?);
        let (d, l) = ([e[0] - b[0], e[1] - b[1]], (e[0] - b[0]).hypot(e[1] - b[1]));
        (l > 0.0).then_some((*e, [d[0] / l, d[1] / l]))
    };
    let n = piece.len();
    [out(piece.first(), piece.get(1)), out(piece.last(), n.checked_sub(2).and_then(|i| piece.get(i)))]
        .into_iter()
        .flatten()
        .collect()
}

/// What a bead's bends cost on a top surface, which shows where the beads' direction jumps and
/// hides their ends at its edge under the wall they overlap. A curve's direction jumps where it
/// bends within a spacing `s`, so a bead costs its bending energy, `(s/2)∫κ²`: nothing on a long
/// straight or a wide arc, about `π/2` at a corner turned through a quarter on the bead's radius.
/// It is taken at each corner, the closing one too, as `θ²/2 · s/ℓ`, a turn through `θ` over the
/// mean `ℓ` of its two chords, which a curve cut finer leaves the same; a chord longer than `s`
/// turns as sharply as one `s` long. So a square `W` wide pays `πW/s` for its insets' corners
/// against `W/(2s)` for its lines' ends (`HIDDEN_END`), at any size, and takes lines; a rib a bead
/// or two wide, where lines across are all ends, keeps its insets.
fn kinks(piece: &[Point], s: f64) -> f64 {
    let bend = |a: &Point, b: &Point, c: &Point| {
        let (u, v) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
        let (lu, lv) = (u[0].hypot(u[1]), v[0].hypot(v[1]));
        if lu > 0.0 && lv > 0.0 {
            let turn = (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]);
            turn * turn / 2.0 * s / ((lu + lv) / 2.0).min(s)
        } else {
            0.0
        }
    };
    let inner: f64 = piece.windows(3).map(|w| if let [a, b, c] = w { bend(a, b, c) } else { 0.0 }).sum();
    let closing = match (whole(piece), piece.get(piece.len().wrapping_sub(2)), piece.first(), piece.get(1)) {
        (true, Some(a), Some(b), Some(c)) => bend(a, b, c),
        _ => 0.0,
    };
    inner + closing
}

/// Fills a dense area with beads: `area` with the parameters `p`, over the dense beads
/// `beneath`, its lines along `fall` where it gives a shape of it one.
pub(crate) fn fill(
    area: &Region,
    p: Spacing,
    beneath: &Beneath,
    fall: impl Fn(&Region) -> Option<f64>,
) -> Result<Dense, GeomError> {
    let (s, w) = (p.spacing, p.width);
    if area.is_empty() || !(s.is_finite() && s > 0.0 && w.is_finite() && w > 0.0) {
        return Ok(Dense::default());
    }
    let crossing = if p.crossing.is_finite() && p.crossing > 0.0 { p.crossing } else { 0.0 };
    let weight = if crossing > 0.0 { 1.0 / (s * crossing) } else { ALWAYS_CROSS };
    let mut components: Vec<Component> = area.iter().map(|shape| Component::new(shape, &fall)).collect();
    // What a disc `W*` across reaches in the area, and the kernel's resolution on: where it runs
    // along the area's edge it covers it, which the two edges, rounded each its own way, would
    // leave needles between as thin as the lattice. The rest is the bands, what that reach leaves
    // of the area: the area less their common part would bring those needles back.
    let (thick, thin) = match crossing > 0.0 {
        true => {
            let core = disc_offset(area, -crossing / 2.0)?;
            let reached = disc_offset(&core, crossing / 2.0 + RESOLUTION_MM)?;
            (intersection(&reached, area)?, difference(area, &reached)?)
        }
        false => (area.clone(), Vec::new()),
    };
    let mut cells = cells_of(&thick, &thin, &components, beneath)?;
    for (j, component) in components.iter_mut().enumerate() {
        component.add_below(cells.iter().filter(|c| c.component == j).flat_map(|c| c.below.iter()));
    }
    let host = hosts(&cells, s);
    let wanted = families(&cells, &host, &components);
    let banded = wanted.iter().any(|f| f.contains(&Family::Insets));
    let levels = if banded { insets(area, &thin, s, w, crossing / 2.0 + s)? } else { Vec::new() };
    let rings = Rings::of(&levels);
    // What each family costs each deciding cell.
    for (i, families) in wanted.iter().enumerate() {
        let Some(cell) = cells.get(i) else { continue };
        let Some(component) = components.get(cell.component) else { continue };
        let mut choices = Vec::with_capacity(families.len());
        for &family in families {
            let mut choice =
                Choice { family, edge_ends: 0.0, kinks: 0.0, ends_to: Vec::new(), along_below: 0.0 };
            for piece in beads(family, cell, component, s, &rings)? {
                // A bead hardly longer than it is wide is all ends, hidden or not.
                let edge_end = if p.shows {
                    let short = (s / polyline_length(&piece).max(s)).min(1.0);
                    HIDDEN_END + (1.0 - HIDDEN_END) * short
                } else {
                    1.0
                };
                if p.shows {
                    choice.kinks += kinks(&piece, s);
                }
                if family == Family::Insets && whole(&piece) {
                    continue;
                }
                for (end, out) in ends(&piece) {
                    let q = [end[0] + STEP_MM * out[0], end[1] + STEP_MM * out[1]];
                    match holder(&cells, q, i).and_then(|n| host.get(n).copied()) {
                        Some(n) if n != i => match choice.ends_to.iter_mut().find(|(m, _)| *m == n) {
                            Some((_, k)) => *k += 1.0,
                            None => choice.ends_to.push((n, 1.0)),
                        },
                        Some(_) => {}
                        None => choice.edge_ends += edge_end,
                    }
                }
            }
            for &(a, b, width) in &cell.below {
                let along = (b[1] - a[1]).atan2(b[0] - a[0]);
                let own = match family {
                    Family::Lines(angle) => angle.to_radians(),
                    Family::Insets => component.inset_direction([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]),
                };
                choice.along_below +=
                    (b[0] - a[0]).hypot(b[1] - a[1]) * width * (along - own).cos().abs() * weight;
            }
            choices.push(choice);
        }
        if let Some(cell) = cells.get_mut(i) {
            cell.choices = choices;
        }
    }
    choose(&mut cells, &host);
    laid(&cells, &host, &components, s, w, &rings, &levels)
}

/// A contour closed back to its start, as a polyline.
fn closed(c: &Contour) -> Polyline {
    c.iter().copied().chain(c.first().copied()).collect()
}

/// What cell `i` costs with its choice `k`, beside its neighbours as they are: its own ends, those
/// on the edges of cells of another family, theirs on its edge likewise, and its area along the
/// layer below.
fn cost(cells: &[Cell], i: usize, k: usize) -> f64 {
    let Some(cell) = cells.get(i) else { return f64::INFINITY };
    let Some(choice) = cell.choices.get(k) else { return f64::INFINITY };
    let family_of = |n: usize| cells.get(n).and_then(|c| c.family());
    let mut total = choice.edge_ends + choice.kinks + choice.along_below;
    for &(n, count) in &choice.ends_to {
        if family_of(n) != Some(choice.family) {
            total += count;
        }
    }
    for &(n, _) in &cell.neighbours {
        let Some(other) = cells.get(n).and_then(|c| c.choices.get(c.chosen)) else { continue };
        if other.family != choice.family {
            total += other.ends_to.iter().filter(|(m, _)| *m == i).map(|(_, c)| c).sum::<f64>();
        }
    }
    total
}

/// The cells' families: each cell's best alone, then each in turn, largest first, the family
/// that costs least beside its neighbours', until none changes.
fn choose(cells: &mut [Cell], host: &[usize]) {
    let alone =
        |c: &Choice| c.edge_ends + c.kinks + c.along_below + c.ends_to.iter().map(|(_, k)| k).sum::<f64>();
    let deciding: Vec<usize> = (0..cells.len()).filter(|&i| host.get(i) == Some(&i)).collect();
    for &i in &deciding {
        if let Some(cell) = cells.get_mut(i) {
            cell.chosen = least(cell.choices.iter().map(alone).collect(), 0);
        }
    }
    let mut order = deciding;
    let key = |i: usize| cells.get(i).map_or((0.0, [0.0, 0.0]), |c| (c.area, c.centre));
    order.sort_by(|&a, &b| {
        let ((area_a, at_a), (area_b, at_b)) = (key(a), key(b));
        area_b.total_cmp(&area_a).then(at_a[0].total_cmp(&at_b[0])).then(at_a[1].total_cmp(&at_b[1]))
    });
    for _ in 0..MAX_ROUNDS {
        let mut changed = false;
        for &i in &order {
            let Some(n) = cells.get(i).map(|c| c.choices.len()) else { continue };
            let current = cells.get(i).map_or(0, |c| c.chosen);
            let best = least((0..n).map(|k| cost(cells, i, k)).collect(), current);
            if best != current
                && let Some(cell) = cells.get_mut(i)
            {
                cell.chosen = best;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
}

/// The choice of least cost among `costs`: `current` unless another beats it by over `TIE`, the
/// earliest of those within `TIE` of the least.
fn least(costs: Vec<f64>, current: usize) -> usize {
    let Some(min) = costs.iter().copied().min_by(f64::total_cmp) else { return current };
    if costs.get(current).is_some_and(|&c| c <= min + TIE) {
        return current;
    }
    costs.iter().position(|&c| c <= min + TIE).unwrap_or(current)
}

/// The beads of the chosen families: each family's lines over all its cells of one shape, so
/// that they go on from cell to cell; the insets over all theirs, and what they leave.
fn laid(
    cells: &[Cell],
    host: &[usize],
    components: &[Component],
    s: f64,
    w: f64,
    rings: &Rings,
    levels: &[Inset],
) -> Result<Dense, GeomError> {
    let family = |i: usize| host.get(i).and_then(|&h| cells.get(h)).and_then(|c| c.family());
    // The cells of each family, joined at once.
    let joined = |of: &dyn Fn(usize, &Cell) -> bool| {
        let contours: Vec<Contour> = cells
            .iter()
            .enumerate()
            .filter(|(i, c)| of(*i, c))
            .flat_map(|(_, c)| c.region.iter().flatten().cloned())
            .collect();
        winding_region(&contours)
    };
    let mut out = Dense::default();
    for (j, component) in components.iter().enumerate() {
        let mut angles: Vec<f64> = Vec::new();
        for (i, _) in cells.iter().enumerate().filter(|(_, c)| c.component == j) {
            if let Some(Family::Lines(angle)) = family(i)
                && !angles.contains(&angle)
            {
                angles.push(angle);
            }
        }
        for angle in angles {
            let region = joined(&|i, c| c.component == j && family(i) == Some(Family::Lines(angle)))?;
            out.lines.extend(hatch(&region, s, angle, component.phase(angle, s))?);
        }
    }
    let inset_region = joined(&|i, _| family(i) == Some(Family::Insets))?;
    if inset_region.is_empty() {
        return Ok(out);
    }
    for piece in clip_lines(&rings.lines, &inset_region)? {
        let closed = whole(&piece);
        let mut piece = piece;
        if closed {
            piece.pop();
        }
        out.insets.push((piece, closed));
    }
    // What no bead's band reaches: outside the first's, and past each bead where the next one's
    // does not reach; a band reaches `w/2` past its inset's region, `w` past the region past it.
    let reach = |k: usize| levels.get(k).map_or(Ok(Vec::new()), |l| disc_offset(&l.past, w));
    let mut left = difference(&inset_region, &reach(0)?)?;
    for (k, level) in levels.iter().enumerate() {
        let past = intersection(&level.past, &inset_region)?;
        left = union(&left, &difference(&past, &reach(k + 1)?)?)?;
    }
    out.residue = disc_opening(&left, w / 4.0)?;
    Ok(out)
}
