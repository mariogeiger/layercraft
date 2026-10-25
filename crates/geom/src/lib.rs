//! Planar regions in millimetres and the exact set operations on them.
//!
//! A `Region` is a set of shapes; a shape is an outer contour (counterclockwise) followed by
//! its holes (clockwise). Every operation goes through `i_overlay`'s integer engine, so the
//! results are topologically valid whatever the input. Inputs are checked first: coordinates
//! must be finite and inside `LIMIT_MM`, which keeps the engine away from its panicking paths.
//! Results lie on a nanometre lattice (`GRID_MM`).
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

mod ops;

pub use ops::{
    GRID_MM, area, clip_lines, difference, intersection, offset, union, union_contours, winding_region,
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
#[derive(Debug, Clone, PartialEq)]
pub enum GeomError {
    /// A coordinate or distance was NaN or infinite.
    NonFinite,
    /// A coordinate or distance was beyond `LIMIT_MM`.
    OutOfRange(f64),
    /// The overlay engine refused the input on its lattice.
    Engine(String),
}

impl std::fmt::Display for GeomError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GeomError::NonFinite => write!(f, "a coordinate is not finite"),
            GeomError::OutOfRange(v) => write!(f, "coordinate {v} mm is beyond {LIMIT_MM} mm"),
            GeomError::Engine(e) => write!(f, "the overlay engine refused the input: {e}"),
        }
    }
}

impl std::error::Error for GeomError {}

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

/// The signed area of a contour: positive when counterclockwise.
pub fn signed_area(contour: &[Point]) -> f64 {
    let n = contour.len();
    let mut twice = 0.0;
    for i in 0..n {
        if let (Some(a), Some(b)) = (contour.get(i), contour.get((i + 1) % n)) {
            twice += a[0] * b[1] - b[0] * a[1];
        }
    }
    twice / 2.0
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
