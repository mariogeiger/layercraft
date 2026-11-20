//! Vectors across the plate, as the motion stage's geometry uses them: differences, a point moved
//! along a direction, dot and cross products, lengths.

use layercraft_geom::Point;

pub(crate) fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}

/// `a` moved `t` times along `d`.
pub(crate) fn add_scaled(a: Point, d: Point, t: f64) -> Point {
    [a[0] + t * d[0], a[1] + t * d[1]]
}

pub(crate) fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

/// The cross product's vertical component: positive when `b` turns anticlockwise from `a`.
pub(crate) fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

pub(crate) fn norm(a: Point) -> f64 {
    a[0].hypot(a[1])
}
