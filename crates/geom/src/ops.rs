//! Boolean operations, offsets and line clipping on regions, in integer nanometres.
//!
//! Coordinates go onto one global lattice of `GRID_MM` (integers in nanometres, origin at
//! zero) and `i_overlay`'s integer engine works there exactly. Every result lies on that lattice,
//! so two results agree bit for bit where they meet, and two consecutive points of a contour are
//! at least a lattice step apart.
//!
//! Rounding to the lattice moves a point by under half a step, so two roundings of one edge,
//! such as the sections of a vertical wall at two heights, differ by strips a step thin, an
//! offset leaves needles as thin where very short edges meet, and a chain of operations makes
//! them a few steps thick. The exact results have none: no result keeps a contour thinner than
//! `RESOLUTION_MM`, a thousand steps.

use crate::{GeomError, Point, Polyline, Region, check_points, check_value, signed_area};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::IntOverlayOptions;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::core::single::SingleIntOverlay;
use i_overlay::i_float::int::angle::Angle;
use i_overlay::i_float::int::point::IntPoint;
use i_overlay::mesh::int::outline::offset::IntOutlineOffset;
use i_overlay::mesh::int::style::{IntLineJoin, IntOutlineStyle};
use i_overlay::string::clip::{ClipRule, IntClip};
use i_overlay::string::overlay::StringOverlay;
use i_overlay::string::rule::StringRule;
use i_triangle::int::triangulatable::IntTriangulatable;
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, SQRT_2, TAU};

/// The lattice every result lies on, in mm: a nanometre, a thousandth of a printer's finest
/// step, and the distance under which CAD programs such as SolidCraft weld points.
pub const GRID_MM: f64 = 1e-6;
/// Lattice steps per millimetre.
const STEPS: f64 = 1e6;
/// The thinnest contour a result keeps, in mm: a micrometre, a printer's finest step, far above
/// the few steps that rounding debris reaches and far below anything a bead can fill.
pub const RESOLUTION_MM: f64 = 1e-3;

/// The smallest interior angle kept sharp by an offset; sharper corners are clipped (radians).
const MITER_MIN_ANGLE: f64 = 0.1;

type Lattice = IntPoint<i64>;

fn to_lattice(p: &Point) -> Lattice {
    // Checked inputs are within LIMIT_MM, so the steps fit an i64 with room to spare.
    IntPoint::new((p[0] * STEPS).round() as i64, (p[1] * STEPS).round() as i64)
}

pub(crate) fn from_lattice(q: &Lattice) -> Point {
    [q.x as f64 / STEPS, q.y as f64 / STEPS]
}

fn contours_to_lattice(contours: &[Vec<Point>]) -> Vec<Vec<Lattice>> {
    contours.iter().map(|c| c.iter().map(to_lattice).collect()).collect()
}

pub(crate) fn region_to_lattice(region: &Region) -> Vec<Vec<Vec<Lattice>>> {
    region.iter().map(|shape| contours_to_lattice(shape)).collect()
}

fn region_from_lattice(shapes: Vec<Vec<Vec<Lattice>>>) -> Region {
    without_slivers(shapes)
        .iter()
        .map(|shape| shape.iter().map(|c| c.iter().map(from_lattice).collect()).collect())
        .collect()
}

fn check_region(region: &Region) -> Result<(), GeomError> {
    check_points(region.iter().flatten())
}

/// The region covered by contours under the even-odd rule: a closed loop of any orientation
/// encloses what is inside it, and an enclosed loop makes a hole.
pub fn union_contours(contours: &[Vec<Point>]) -> Result<Region, GeomError> {
    fill(contours, FillRule::EvenOdd)
}

/// The points of positive winding number: the region bounded by oriented loops, where a
/// counterclockwise loop adds one and a clockwise loop subtracts one. Overlapping solids merge.
pub fn winding_region(contours: &[Vec<Point>]) -> Result<Region, GeomError> {
    fill(contours, FillRule::Positive)
}

fn fill(contours: &[Vec<Point>], rule: FillRule) -> Result<Region, GeomError> {
    check_points(contours)?;
    let empty: Vec<Vec<Lattice>> = Vec::new();
    Ok(region_from_lattice(contours_to_lattice(contours).overlay(&empty, OverlayRule::Subject, rule)))
}

/// The union of two regions.
pub fn union(a: &Region, b: &Region) -> Result<Region, GeomError> {
    boolean(a, b, OverlayRule::Union)
}

/// The part of `a` outside `b`.
pub fn difference(a: &Region, b: &Region) -> Result<Region, GeomError> {
    boolean(a, b, OverlayRule::Difference)
}

/// The part of `a` inside `b`.
pub fn intersection(a: &Region, b: &Region) -> Result<Region, GeomError> {
    boolean(a, b, OverlayRule::Intersect)
}

fn boolean(a: &Region, b: &Region, rule: OverlayRule) -> Result<Region, GeomError> {
    check_region(a)?;
    check_region(b)?;
    let (a, b) = (region_to_lattice(a), region_to_lattice(b));
    Ok(region_from_lattice(a.overlay(&b, rule, FillRule::NonZero)))
}

/// The region grown by `distance` (shrunk when negative), with mitred corners.
pub fn offset(region: &Region, distance: f64) -> Result<Region, GeomError> {
    check_region(region)?;
    check_value(distance)?;
    let steps = (distance * STEPS).round() as i64;
    if region.is_empty() {
        return Ok(Vec::new());
    }
    if steps == 0 {
        return Ok(region.clone());
    }
    let angle = Angle::from_radians(MITER_MIN_ANGLE).ok_or(GeomError::Engine("miter angle".into()))?;
    let style = IntOutlineStyle::new(steps).line_join(IntLineJoin::Miter(angle));
    let shapes =
        region_to_lattice(region).outline(&style).map_err(|e| GeomError::Engine(format!("{e:?}")))?;
    Ok(region_from_lattice(shapes))
}

/// The region grown by `distance` (shrunk when negative) with round corners: its Minkowski sum
/// with a disc, or what lies at least that deep in it. It moves with the region's boundary as a
/// set, so a step of rounding there moves it by no more; a mitre, which follows each edge's
/// direction, would turn a step-long edge into a spike. Each arc is cut into equal chords within
/// `RESOLUTION_MM` of it, the same however the region is turned or mirrored, which turns the
/// arc round.
///
/// Each contour's edges move `distance` along their normal out of the region and are joined
/// round each corner where they part, through it where they cross; the points that curve winds
/// round once, the way the contour does, are the contour's offset, and the region's is the
/// outer contours' less its holes'.
pub fn disc_offset(region: &Region, distance: f64) -> Result<Region, GeomError> {
    check_region(region)?;
    check_value(distance)?;
    let total = (distance * STEPS).round() as i64;
    if region.is_empty() {
        return Ok(Vec::new());
    }
    if total == 0 {
        return Ok(region.clone());
    }
    let shapes = disc_stages(region_to_lattice(region), total, chord_within_resolution(total));
    Ok(from_lattice_shapes(&shapes))
}

/// The erosions of one region by growing depths, each from the one before, the disc's erosions
/// adding up: `E(E(A, a), b) = E(A, a + b)`.
pub struct Erosions {
    shapes: Vec<Vec<Vec<Lattice>>>,
    depth: i64,
    chord: f64,
}

impl Erosions {
    /// A region's erosions, as deep as `deepest` at most, their arcs within `RESOLUTION_MM`.
    pub fn new(region: &Region, deepest: f64) -> Result<Erosions, GeomError> {
        check_region(region)?;
        check_value(deepest)?;
        let chord = chord_within_resolution((deepest * STEPS).round() as i64);
        Ok(Erosions { shapes: region_to_lattice(region), depth: 0, chord })
    }

    /// The erosion `depth` deep, or as deep as it already is.
    pub fn at(&mut self, depth: f64) -> Result<Region, GeomError> {
        check_value(depth)?;
        let target = (depth * STEPS).round() as i64;
        if target > self.depth {
            self.shapes = disc_stages(std::mem::take(&mut self.shapes), self.depth - target, self.chord);
            self.depth = target;
        }
        Ok(from_lattice_shapes(&self.shapes))
    }
}

/// The angle a chord of an arc `steps` of the lattice in radius may span to stay within
/// `RESOLUTION_MM` of it.
fn chord_within_resolution(steps: i64) -> f64 {
    let ratio = 1.0 - RESOLUTION_MM * STEPS / steps.abs().max(1) as f64;
    2.0 * ratio.clamp(-1.0, 1.0).acos()
}

/// A region moved `total` steps of the lattice in equal stages, whose rounding adds up to that
/// exactly, and whose single-vertex turns leave arcs of any radius up to `total` within a chord.
fn disc_stages(mut shapes: Vec<Vec<Vec<Lattice>>>, total: i64, chord: f64) -> Vec<Vec<Vec<Lattice>>> {
    let stages = (total.unsigned_abs() as f64 / (DISC_STAGE_MM * STEPS)).ceil().max(1.0) as i64;
    let mut done = 0;
    for k in 1..=stages {
        let upto = ((total as i128 * k as i128) as f64 / stages as f64).round() as i64;
        shapes = disc_offset_stage(&shapes, (upto - done) as f64, chord);
        done = upto;
        if shapes.is_empty() {
            break;
        }
    }
    shapes
}

fn from_lattice_shapes(shapes: &[Vec<Vec<Lattice>>]) -> Region {
    shapes.iter().map(|shape| shape.iter().map(|c| c.iter().map(from_lattice).collect()).collect()).collect()
}

/// One stage of a disc offset, `steps` of the lattice long, its arcs cut into chords of `chord`.
fn disc_offset_stage(region: &[Vec<Vec<Lattice>>], steps: f64, chord: f64) -> Vec<Vec<Vec<Lattice>>> {
    let empty: Vec<Vec<Lattice>> = Vec::new();
    let mut parts: Vec<Vec<Lattice>> = Vec::new();
    for contour in region.iter().flatten() {
        let mut contour = contour.clone();
        contour.dedup();
        while contour.len() > 1 && contour.first() == contour.last() {
            contour.pop();
        }
        let twice_area = doubled_area(&contour);
        if contour.len() < 3 || twice_area == 0 {
            continue;
        }
        let outer = twice_area > 0;
        let rule = if outer { FillRule::Positive } else { FillRule::Negative };
        let moved = vec![moved_contour(&contour, steps, chord)];
        for shape in moved.overlay(&empty, OverlayRule::Subject, rule) {
            parts.extend(shape.into_iter().map(|c| if outer { c } else { c.into_iter().rev().collect() }));
        }
    }
    without_slivers(parts.overlay(&empty, OverlayRule::Subject, FillRule::Positive))
}

/// How far one stage of a disc offset goes, in mm. The edges of a contour moved past what it
/// bounds all cross: an erosion deep past a thin band, the common case, costs the square of its
/// edges. The disc's erosions and dilations add up, `E(E(A, a), b) = E(A, a + b)`, so that stages
/// reach the same set with the crossings local, what is thin gone early, and an end at empty. A
/// stage is under a bead's half-width, so that a band of a few beads dies within a stage or two;
/// on a Benchy, 0.25 mm spends 9 s in here where 1 mm spent 50 s.
const DISC_STAGE_MM: f64 = 0.25;

/// The most chords one arc of a disc offset is cut into.
const MAX_CHORDS: f64 = 65_536.0;

/// Twice a lattice contour's signed area, positive counterclockwise.
fn doubled_area(contour: &[Lattice]) -> i128 {
    let n = contour.len();
    (0..n)
        .filter_map(|i| Some((contour.get(i)?, contour.get((i + 1) % n)?)))
        .map(|(a, b)| a.x as i128 * b.y as i128 - b.x as i128 * a.y as i128)
        .sum()
}

/// One corner of a contour moved `d` steps along its edges' right-hand normals.
struct MovedCorner {
    at: Lattice,
    /// The moves of the edges coming in and going out, one step of the lattice rounded once for
    /// each edge, so that it stays parallel to itself.
    moves: [Lattice; 2],
    /// The lengths of those edges.
    lengths: [f64; 2],
    /// Whether the moved edges part here, or else cross.
    parting: bool,
    /// How much of each edge crossing cuts off, zero where they part.
    trim: f64,
    /// The turn's sine and cosine, times the edges' lengths.
    turn: f64,
    along: f64,
}

impl MovedCorner {
    fn new([a, b, c]: [Lattice; 3], d: f64) -> Self {
        let shift = |[dx, dy]: [f64; 2], l: f64| {
            IntPoint::new((d * dy / l).round() as i64, (-d * dx / l).round() as i64)
        };
        let (u, v) = ([(b.x - a.x) as f64, (b.y - a.y) as f64], [(c.x - b.x) as f64, (c.y - b.y) as f64]);
        let lengths = [u[0].hypot(u[1]), v[0].hypot(v[1])];
        let (turn, along) = (u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1]);
        let parting = if turn != 0.0 { (turn > 0.0) == (d > 0.0) } else { along < 0.0 };
        // The half-angle's tangent, sin / (1 + cos), times the distance.
        let half_tangent = turn.abs() / (lengths[0] * lengths[1] + along);
        let trim = if parting { 0.0 } else { d.abs() * half_tangent };
        MovedCorner {
            at: b,
            moves: [shift(u, lengths[0]), shift(v, lengths[1])],
            lengths,
            parting,
            trim,
            turn,
            along,
        }
    }

    /// Where the lines of the two moved edges meet: `d / cos(turn / 2)` along the bisector.
    fn meet(&self) -> Lattice {
        let [m, k] = self.moves;
        let squared = |t: Lattice| t.x as f64 * t.x as f64 + t.y as f64 * t.y as f64;
        let mean = (squared(m) + squared(k)) / 2.0;
        let reach = mean / (mean + m.x as f64 * k.x as f64 + m.y as f64 * k.y as f64);
        let along = |a: i64, b: i64| ((a + b) as f64 * reach).round() as i64;
        IntPoint::new(self.at.x + along(m.x, k.x), self.at.y + along(m.y, k.y))
    }
}

/// A lattice contour's edges moved `d` steps along their right-hand normals, out of the region it
/// bounds: the raw offset curve, whose positive winding number marks the offset (Chen & McMains).
///
/// Where the moved edges part, an arc of radius `|d|` about the corner joins them, cut into equal
/// chords spanning at most `chord` radians. A turn within one chord keeps a single vertex, where
/// the edges' lines meet, as far outside the arc as a chord is inside it: the stages of an offset
/// carry a curve's vertices and lines along, instead of doubling the vertices. Where they cross
/// they meet at that same point, exactly, when each edge has the room for it: a point this cuts
/// away, swept by both edges, has lost two to the one it gets back and still does not wind
/// positively. Without the room, the curve goes back through the corner, and the winding number
/// does the cutting.
fn moved_contour(contour: &[Lattice], d: f64, chord: f64) -> Vec<Lattice> {
    let n = contour.len();
    let at = |i: usize| contour.get(i % n).copied().unwrap_or(IntPoint::new(0, 0));
    let corners: Vec<MovedCorner> =
        (0..n).map(|i| MovedCorner::new([at(i), at(i + 1), at(i + 2)], d)).collect();
    let trim = |i: usize| corners.get(i % n).map_or(0.0, |c| c.trim);
    let plus = |p: Lattice, t: Lattice| IntPoint::new(p.x + t.x, p.y + t.y);
    let mut out = Vec::with_capacity(n);
    for (i, c) in corners.iter().enumerate() {
        let (b, [m, k]) = (c.at, c.moves);
        if !c.parting {
            if trim(i + n - 1) + c.trim < c.lengths[0] && c.trim + trim(i + 1) < c.lengths[1] {
                out.push(c.meet());
            } else {
                out.extend([plus(b, m), b, plus(b, k)]);
            }
            continue;
        }
        // Round the corner, the way the normal turns: anticlockwise growing, clockwise shrinking.
        let (from, to) = ((m.y as f64).atan2(m.x as f64), (k.y as f64).atan2(k.x as f64));
        let sweep = if d > 0.0 { (to - from).rem_euclid(TAU) } else { -(from - to).rem_euclid(TAU) };
        let sweep = if c.turn == 0.0 { PI.copysign(sweep) } else { sweep };
        if sweep.abs() <= chord && c.along > 0.0 {
            out.push(c.meet());
            continue;
        }
        // The chords' ends round to the lattice, each by half a diagonal at most, so that a turn
        // cut under `chord` by what that can tip over stays under it in the next stages.
        let r = d.abs();
        let span = (chord - 2.0 * SQRT_2 / (r * chord)).max(chord / 2.0);
        let pieces = (sweep.abs() / span).ceil().clamp(1.0, MAX_CHORDS) as usize;
        out.push(plus(b, m));
        out.extend((1..pieces).map(|j| {
            let angle = from + sweep * j as f64 / pieces as f64;
            IntPoint::new(b.x + (r * angle.cos()).round() as i64, b.y + (r * angle.sin()).round() as i64)
        }));
        out.push(plus(b, k));
    }
    out
}

/// What of the region a disc of radius `r` within it reaches: shrunk by `r` and grown back,
/// round, the parts narrower than `2r` gone.
pub fn disc_opening(region: &Region, r: f64) -> Result<Region, GeomError> {
    if r <= 0.0 || region.is_empty() {
        return Ok(region.clone());
    }
    let core = disc_offset(region, -r)?;
    intersection(&disc_offset(&core, r)?, region)
}

/// Whether a contour is at least `RESOLUTION_MM` thick, its thickness being twice its area over
/// its perimeter: the width of a strip, half the radius of a disc.
fn resolved(contour: &[Lattice]) -> bool {
    let n = contour.len();
    let (mut doubled_area, mut perimeter) = (0i128, 0.0f64);
    for (i, a) in contour.iter().enumerate() {
        let Some(b) = contour.get((i + 1) % n) else { continue };
        doubled_area += a.x as i128 * b.y as i128 - b.x as i128 * a.y as i128;
        perimeter += ((b.x - a.x) as f64).hypot((b.y - a.y) as f64);
    }
    doubled_area.unsigned_abs() as f64 >= perimeter * RESOLUTION_MM * STEPS
}

/// The shapes without the contours thinner than `RESOLUTION_MM`: rounding debris.
fn without_slivers(shapes: Vec<Vec<Vec<Lattice>>>) -> Vec<Vec<Vec<Lattice>>> {
    shapes
        .into_iter()
        .filter(|shape| shape.first().is_some_and(|outer| resolved(outer)))
        .map(|shape| {
            let mut contours = shape.into_iter();
            let outer = contours.next();
            outer.into_iter().chain(contours.filter(|hole| resolved(hole))).collect()
        })
        .collect()
}

/// The pieces of `lines` inside `region`, each running the way its line runs and as far as it
/// runs inside: a piece ends where its line ends, meets the region's boundary or crosses a line.
/// A line that ends where it starts is a ring, which goes on through its start.
pub fn clip_lines(lines: &[Polyline], region: &Region) -> Result<Vec<Polyline>, GeomError> {
    check_points(lines)?;
    check_region(region)?;
    if lines.is_empty() || region.is_empty() {
        return Ok(Vec::new());
    }
    let rule = ClipRule { invert: false, boundary_included: false };
    let paths = contours_to_lattice(lines);
    let pieces = region_to_lattice(region).clip_paths(&paths, FillRule::NonZero, rule);
    let pieces = rejoined(pieces, &cut_points(&paths));
    Ok(pieces.iter().map(|p| p.iter().map(from_lattice).collect()).collect())
}

/// Where a run of a line inside a region may come back cut in two, the half before ending there
/// and the half after starting there: the start of a ring, a line closed back to it, and the
/// vertices where a line, closed or not, comes in from above and goes back up, in the order x
/// then y, below both of their neighbours, since `i_overlay` walks each run from its least point.
fn cut_points(paths: &[Vec<Lattice>]) -> BTreeSet<Lattice> {
    let mut cuts = BTreeSet::new();
    for path in paths {
        let mut points = path.clone();
        points.dedup();
        let closed = points.len() > 2 && points.first() == points.last();
        if closed {
            points.pop();
            cuts.extend(points.first().copied());
        }
        let n = points.len();
        let inner = if closed { 0..n } else { 1..n.saturating_sub(1) };
        for i in inner {
            let around = [points.get((i + n - 1) % n), points.get(i), points.get((i + 1) % n)];
            if let [Some(before), Some(p), Some(after)] = around
                && p < before
                && p < after
            {
                cuts.insert(*p);
            }
        }
    }
    cuts
}

/// The pieces joined again where one ends and the next starts at one of `cuts`.
fn rejoined(pieces: Vec<Vec<Lattice>>, cuts: &BTreeSet<Lattice>) -> Vec<Vec<Lattice>> {
    let mut starting: BTreeMap<Lattice, Vec<usize>> = BTreeMap::new();
    for (i, piece) in pieces.iter().enumerate() {
        if let Some(start) = piece.first().filter(|s| cuts.contains(s)) {
            starting.entry(*start).or_default().push(i);
        }
    }
    let mut next = vec![None; pieces.len()];
    let mut follows = vec![false; pieces.len()];
    for (i, piece) in pieces.iter().enumerate() {
        let Some(after) = piece.last().and_then(|end| starting.get_mut(end)) else { continue };
        let Some(k) = after.iter().position(|&j| j != i) else { continue };
        let j = after.swap_remove(k);
        if let (Some(n), Some(f)) = (next.get_mut(i), follows.get_mut(j)) {
            (*n, *f) = (Some(j), true);
        }
    }
    // Chains from the pieces that follow none, then the loops left.
    let mut taken = vec![false; pieces.len()];
    let heads = (0..pieces.len()).filter(|&i| follows.get(i) == Some(&false));
    let mut joined = Vec::new();
    for head in heads.chain(0..pieces.len()) {
        let mut chain: Vec<Lattice> = Vec::new();
        let mut at = Some(head);
        while let Some(i) = at.filter(|&i| taken.get(i) == Some(&false)) {
            if let Some(t) = taken.get_mut(i) {
                *t = true;
            }
            let piece = pieces.get(i).map_or(&[][..], |p| p.as_slice());
            chain.extend(piece.iter().skip(usize::from(!chain.is_empty())));
            at = next.get(i).copied().flatten();
        }
        if !chain.is_empty() {
            joined.push(chain);
        }
    }
    joined
}

/// The region without its parts narrower than `2r`: shrunk by `r`, then grown back.
pub fn opening(region: &Region, r: f64) -> Result<Region, GeomError> {
    if r <= 0.0 || region.is_empty() {
        return Ok(region.clone());
    }
    let core = offset(region, -r)?;
    intersection(&offset(&core, r)?, region)
}

/// Triangles covering the region exactly, without overlap: their corners, on the lattice, and
/// counterclockwise index triples into them.
pub fn triangulate(region: &Region) -> Result<(Vec<Point>, Vec<[u32; 3]>), GeomError> {
    check_region(region)?;
    let mesh = region_to_lattice(region).triangulate().into_triangulation::<u32>();
    let (triangles, _) = mesh.indices.as_chunks::<3>();
    Ok((mesh.points.iter().map(from_lattice).collect(), triangles.to_vec()))
}

/// The faces segments cut a region into, each a shape of the result, all from one pass so that
/// neighbours share their corners.
pub fn slice(region: &Region, segments: &[[Point; 2]]) -> Result<Region, GeomError> {
    check_region(region)?;
    check_points(&[segments.iter().flatten().copied().collect::<Vec<Point>>()])?;
    let paths: Vec<Vec<Lattice>> = segments.iter().map(|s| s.iter().map(to_lattice).collect()).collect();
    let shapes = region_to_lattice(region);
    let mut overlay = StringOverlay::from_shape_custom(&shapes, IntOverlayOptions::keep_all_points());
    overlay.add_string_source(&paths);
    let faces = overlay
        .build_graph_view(FillRule::NonZero)
        .map(|graph| graph.extract_shapes_custom(StringRule::Slice, IntOverlayOptions::keep_all_points()))
        .unwrap_or_default();
    Ok(faces
        .iter()
        .map(|shape| shape.iter().map(|c| c.iter().map(from_lattice).collect()).collect())
        .collect())
}

/// The area of a region: outer contours count positive, holes negative.
pub fn area(region: &Region) -> f64 {
    region.iter().flatten().map(|c| signed_area(c)).sum()
}
