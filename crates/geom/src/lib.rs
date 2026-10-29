//! Planar regions in millimetres and the exact set operations on them.
//!
//! A `Region` is a set of shapes; a shape is an outer contour (counterclockwise) followed by
//! its holes (clockwise). Every operation goes through `i_overlay`'s integer engine, so the
//! results are topologically valid whatever the input. Inputs are checked first: coordinates
//! must be finite and inside `LIMIT_MM`, which keeps the engine away from its panicking paths.
//! Results lie on a nanometre lattice (`GRID_MM`) and keep no contour thinner than a
//! micrometre (`RESOLUTION_MM`).

mod axis;
mod bands;
mod bead;
mod ops;
mod relief;

pub use axis::{Centreline, chordal_axis};
pub use bands::{Bands, ON_EDGE_MM};
pub use bead::{bead_area, bead_spacing, disc_area};
pub use relief::{Crossing, Grid, Lines, MAX_VALUES, Relief, levels};

pub use ops::{
    GRID_MM, RESOLUTION_MM, area, clip_lines, difference, intersection, offset, opening, triangulate, union,
    union_contours, winding_region,
};

/// A point in the plane, in millimetres.
pub type Point = [f64; 2];
/// A closed polygon; the last point connects back to the first.
pub type Contour = Vec<Point>;
/// An outer contour (counterclockwise) followed by its holes (clockwise).
pub type Shape = Vec<Contour>;
/// A set of disjoint shapes.
pub type Region = Vec<Shape>;
/// An open polyline.
pub type Polyline = Vec<Point>;

/// The largest coordinate magnitude accepted, in millimetres (well past any print bed).
pub const LIMIT_MM: f64 = 100_000.0;

/// Why a geometric operation refused its input.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum GeomError {
    /// A coordinate or distance was NaN or infinite.
    #[error("a coordinate is not finite")]
    NonFinite,
    /// A coordinate or distance was beyond `LIMIT_MM`.
    #[error("coordinate {0} mm is beyond {LIMIT_MM} mm")]
    OutOfRange(f64),
    /// The overlay engine refused the input on its lattice.
    #[error("the overlay engine refused the input: {0}")]
    Engine(String),
}

/// Checks that a value is finite and within `LIMIT_MM`.
pub fn check_value(v: f64) -> Result<f64, GeomError> {
    if !v.is_finite() {
        Err(GeomError::NonFinite)
    } else if v.abs() > LIMIT_MM {
        Err(GeomError::OutOfRange(v))
    } else {
        Ok(v)
    }
}

/// Checks every point of a set of point lists.
pub fn check_points<'a>(lists: impl IntoIterator<Item = &'a Vec<Point>>) -> Result<(), GeomError> {
    for list in lists {
        for p in list {
            check_value(p[0])?;
            check_value(p[1])?;
        }
    }
    Ok(())
}

/// The edges of a closed contour, each from a vertex to the next, the last back to the first.
pub fn edges(contour: &[Point]) -> impl Iterator<Item = (Point, Point)> + '_ {
    contour.iter().copied().zip(contour.iter().copied().cycle().skip(1))
}

/// The signed area of a contour: positive when counterclockwise.
pub fn signed_area(contour: &[Point]) -> f64 {
    edges(contour).fold(0.0, |twice, (a, b)| twice + a[0] * b[1] - b[0] * a[1]) / 2.0
}

/// Whether the edge `a`–`b` crosses the ray from `p` toward +x.
pub fn crosses_ray(p: Point, (a, b): (Point, Point)) -> bool {
    (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
}

/// Whether `p` lies inside the contours whose edges are `edges`, by the even-odd rule: the ray
/// from it crosses them an odd number of times.
pub fn encloses(edges: impl IntoIterator<Item = (Point, Point)>, p: Point) -> bool {
    edges.into_iter().filter(|&e| crosses_ray(p, e)).count() % 2 == 1
}

/// The distance from `p` to the segment `a`–`b`.
pub fn segment_distance(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let len2 = d[0] * d[0] + d[1] * d[1];
    let t =
        if len2 > 0.0 { (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
}

/// The length of an open polyline.
pub fn polyline_length(line: &[Point]) -> f64 {
    line.windows(2)
        .map(|w| match w {
            [a, b] => (b[0] - a[0]).hypot(b[1] - a[1]),
            _ => 0.0,
        })
        .sum()
}

/// The axis-aligned bounds `[min, max]` of a region, or `None` when it is empty.
pub fn bounds(region: &Region) -> Option<[Point; 2]> {
    let mut points = region.iter().flatten().flatten();
    let first = *points.next()?;
    Some(points.fold([first, first], |[lo, hi], p| {
        [[lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]]
    }))
}

#[cfg(test)]
mod tests;
