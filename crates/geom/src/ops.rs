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
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::core::single::SingleIntOverlay;
use i_overlay::i_float::int::angle::Angle;
use i_overlay::i_float::int::point::IntPoint;
use i_overlay::mesh::int::outline::offset::IntOutlineOffset;
use i_overlay::mesh::int::style::{IntLineJoin, IntOutlineStyle};
use i_overlay::string::clip::{ClipRule, IntClip};
use i_triangle::int::triangulatable::IntTriangulatable;
use std::collections::{BTreeMap, BTreeSet};

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
pub fn clip_lines(lines: &[Polyline], region: &Region) -> Result<Vec<Polyline>, GeomError> {
    check_points(lines)?;
    check_region(region)?;
    if lines.is_empty() || region.is_empty() {
        return Ok(Vec::new());
    }
    let rule = ClipRule { invert: false, boundary_included: false };
    let paths = contours_to_lattice(lines);
    let pieces = region_to_lattice(region).clip_paths(&paths, FillRule::NonZero, rule);
    let pieces = rejoined(pieces, &lowest_turns(&paths));
    Ok(pieces.iter().map(|p| p.iter().map(from_lattice).collect()).collect())
}

/// The vertices where a line, closed when it ends where it starts, comes in from above and goes
/// back up, in the order x then y: below both of their neighbours.
///
/// `i_overlay` walks each run of a line inside a region from the run's least point on, so where
/// that point is one of these vertices it hands the run back cut in two there, the half before it
/// ending at it and the half after starting at it.
fn lowest_turns(paths: &[Vec<Lattice>]) -> BTreeSet<Lattice> {
    let mut turns = BTreeSet::new();
    for path in paths {
        let mut points = path.clone();
        points.dedup();
        let closed = points.len() > 2 && points.first() == points.last();
        if closed {
            points.pop();
        }
        let n = points.len();
        let inner = if closed { 0..n } else { 1..n.saturating_sub(1) };
        for i in inner {
            let around = [points.get((i + n - 1) % n), points.get(i), points.get((i + 1) % n)];
            if let [Some(before), Some(p), Some(after)] = around
                && p < before
                && p < after
            {
                turns.insert(*p);
            }
        }
    }
    turns
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

/// The area of a region: outer contours count positive, holes negative.
pub fn area(region: &Region) -> f64 {
    region.iter().flatten().map(|c| signed_area(c)).sum()
}
