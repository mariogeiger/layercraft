//! Boolean operations, offsets and line clipping on regions, through `i_overlay`'s i64 engine.

use crate::{GeomError, Point, Polyline, Region, check_points, check_value, signed_area};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::clip::FloatClip;
use i_overlay::float::single::SingleFloatOverlay;
use i_overlay::mesh::float::outline::offset::OutlineOffset;
use i_overlay::mesh::float::style::{LineJoin, OutlineStyle};
use i_overlay::string::clip::ClipRule;

/// The smallest interior angle kept sharp by an offset; sharper corners are clipped (radians).
const MITER_MIN_ANGLE: f64 = 0.1;

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
    let empty: Vec<Vec<Point>> = Vec::new();
    Ok(contours.to_vec().overlay_as::<i64>(&empty, OverlayRule::Subject, rule))
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
    Ok(a.overlay_as::<i64>(b, rule, FillRule::NonZero))
}

/// The region grown by `distance` (shrunk when negative), with mitred corners.
pub fn offset(region: &Region, distance: f64) -> Result<Region, GeomError> {
    check_region(region)?;
    check_value(distance)?;
    if region.is_empty() {
        return Ok(Vec::new());
    }
    if distance == 0.0 {
        return Ok(region.clone());
    }
    let style = OutlineStyle::new(distance).line_join(LineJoin::Miter(MITER_MIN_ANGLE));
    Ok(region.outline_as::<i64>(&style))
}

/// The pieces of `lines` that lie inside `region`.
pub fn clip_lines(lines: &[Polyline], region: &Region) -> Result<Vec<Polyline>, GeomError> {
    check_points(lines)?;
    check_region(region)?;
    if lines.is_empty() || region.is_empty() {
        return Ok(Vec::new());
    }
    let rule = ClipRule { invert: false, boundary_included: false };
    Ok(lines.to_vec().clip_by_as::<i64>(region, FillRule::NonZero, rule))
}

/// The area of a region: outer contours count positive, holes negative.
pub fn area(region: &Region) -> f64 {
    region.iter().flatten().map(|c| signed_area(c)).sum()
}
