//! Boolean operations, offsets and line clipping on regions, in integer nanometres.
//!
//! Coordinates go onto one global lattice of `GRID_MM` (integers in nanometres, origin at
//! zero) and `i_overlay`'s integer engine works there exactly. Every result lies on that lattice,
//! so two results agree bit for bit where they meet, and two consecutive points of a contour are
//! at least a lattice step apart. No contour of a result is thinner than a lattice step.

use crate::{GeomError, Point, Polyline, Region, check_points, check_value, signed_area};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::core::single::SingleIntOverlay;
use i_overlay::i_float::int::angle::Angle;
use i_overlay::i_float::int::point::IntPoint;
use i_overlay::mesh::int::outline::offset::IntOutlineOffset;
use i_overlay::mesh::int::style::{IntLineJoin, IntOutlineStyle};
use i_overlay::string::clip::{ClipRule, IntClip};

/// The lattice every result lies on, in mm: a nanometre, a thousandth of a printer's finest
/// step, and the distance under which CAD programs such as SolidCraft weld points.
pub const GRID_MM: f64 = 1e-6;
/// Lattice steps per millimetre.
const STEPS: f64 = 1e6;

/// The smallest interior angle kept sharp by an offset; sharper corners are clipped (radians).
const MITER_MIN_ANGLE: f64 = 0.1;

type Lattice = IntPoint<i64>;

fn to_lattice(p: &Point) -> Lattice {
    // Checked inputs are within LIMIT_MM, so the steps fit an i64 with room to spare.
    IntPoint::new((p[0] * STEPS).round() as i64, (p[1] * STEPS).round() as i64)
}

fn from_lattice(q: &Lattice) -> Point {
    [q.x as f64 / STEPS, q.y as f64 / STEPS]
}

fn contours_to_lattice(contours: &[Vec<Point>]) -> Vec<Vec<Lattice>> {
    contours.iter().map(|c| c.iter().map(to_lattice).collect()).collect()
}

fn region_to_lattice(region: &Region) -> Vec<Vec<Vec<Lattice>>> {
    region.iter().map(|shape| contours_to_lattice(shape)).collect()
}

fn region_from_lattice(shapes: Vec<Vec<Vec<Lattice>>>) -> Region {
    shapes.iter().map(|shape| shape.iter().map(|c| c.iter().map(from_lattice).collect()).collect()).collect()
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
    Ok(region_from_lattice(without_slivers(shapes)))
}

/// Whether a contour is at least a lattice step thick, its thickness being twice its area over
/// its perimeter: the width of a strip, half the radius of a disc.
fn thicker_than_a_step(contour: &[Lattice]) -> bool {
    let n = contour.len();
    let (mut doubled_area, mut perimeter) = (0i128, 0.0f64);
    for (i, a) in contour.iter().enumerate() {
        let Some(b) = contour.get((i + 1) % n) else { continue };
        doubled_area += a.x as i128 * b.y as i128 - b.x as i128 * a.y as i128;
        perimeter += ((b.x - a.x) as f64).hypot((b.y - a.y) as f64);
    }
    doubled_area.unsigned_abs() as f64 >= perimeter
}

/// The shapes without the contours thinner than a lattice step. Rounding an offset to the
/// lattice leaves such slivers where very short edges meet, reaching from the offset contour
/// back to the original one; the exact offset has none.
fn without_slivers(shapes: Vec<Vec<Vec<Lattice>>>) -> Vec<Vec<Vec<Lattice>>> {
    shapes
        .into_iter()
        .filter(|shape| shape.first().is_some_and(|outer| thicker_than_a_step(outer)))
        .map(|shape| {
            let mut contours = shape.into_iter();
            let outer = contours.next();
            outer.into_iter().chain(contours.filter(|hole| thicker_than_a_step(hole))).collect()
        })
        .collect()
}

/// The pieces of `lines` that lie inside `region`.
pub fn clip_lines(lines: &[Polyline], region: &Region) -> Result<Vec<Polyline>, GeomError> {
    check_points(lines)?;
    check_region(region)?;
    if lines.is_empty() || region.is_empty() {
        return Ok(Vec::new());
    }
    let rule = ClipRule { invert: false, boundary_included: false };
    let paths = contours_to_lattice(lines);
    let pieces = region_to_lattice(region).clip_paths(&paths, FillRule::NonZero, rule);
    Ok(pieces.iter().map(|p| p.iter().map(from_lattice).collect()).collect())
}

/// The area of a region: outer contours count positive, holes negative.
pub fn area(region: &Region) -> f64 {
    region.iter().flatten().map(|c| signed_area(c)).sum()
}
