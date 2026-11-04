//! The side of a line a point lies on, decided exactly for any floating-point coordinates.
//!
//! `orientation(a, b, c)` is the sign of the determinant `|b − a, c − a|`: `Greater` when `c`
//! lies to the left of the line from `a` to `b`, `Less` to its right, `Equal` exactly on it.
//! Rounding can flip the sign the plain formula gives for points nearly on the line, and then
//! two computations that ask about the same three points, in two triangles sharing an edge, can
//! disagree. So the determinant is first bounded with its rounding error, and only where that
//! bound leaves the sign open is it summed exactly: each product split into its rounded value
//! and its exact error by a fused multiply-add, the twelve parts added without loss into an
//! expansion of non-overlapping doubles, Shewchuk's error-free sums. Every caller then gets the
//! same, true answer.

use crate::Point;
use std::cmp::Ordering;

/// Half the gap between 1 and the next double: the relative error of one rounding.
const UNIT: f64 = f64::EPSILON / 2.0;
/// The plain determinant's error, relative to the sum of its two products' magnitudes.
const FILTER: f64 = (3.0 + 16.0 * UNIT) * UNIT;

/// The side of the line from `a` to `b` that `c` lies on: `Greater` to its left.
pub fn orientation(a: Point, b: Point, c: Point) -> Ordering {
    let left = (a[0] - c[0]) * (b[1] - c[1]);
    let right = (a[1] - c[1]) * (b[0] - c[0]);
    let det = left - right;
    let bound = FILTER * (left.abs() + right.abs());
    if det > bound {
        Ordering::Greater
    } else if -det > bound {
        Ordering::Less
    } else {
        exact(a, b, c)
    }
}

/// `x · y` as its rounded value and the exact rest.
fn product(x: f64, y: f64) -> [f64; 2] {
    let p = x * y;
    [p, x.mul_add(y, -p)]
}

/// `x + y` as its rounded value and the exact rest.
fn sum(x: f64, y: f64) -> (f64, f64) {
    let s = x + y;
    let back = s - x;
    (s, (x - (s - back)) + (y - back))
}

/// The sign of the determinant, from its six products summed without loss.
fn exact(a: Point, b: Point, c: Point) -> Ordering {
    let terms = [
        product(a[0], b[1]),
        product(-a[0], c[1]),
        product(-c[0], b[1]),
        product(-a[1], b[0]),
        product(a[1], c[0]),
        product(c[1], b[0]),
    ];
    // Components of increasing magnitude, none overlapping another.
    let mut expansion: Vec<f64> = Vec::with_capacity(13);
    for x in terms.into_iter().flatten() {
        let mut carry = x;
        for component in expansion.iter_mut() {
            let (s, rest) = sum(carry, *component);
            *component = rest;
            carry = s;
        }
        expansion.push(carry);
    }
    let top = expansion.iter().rev().find(|&&x| x != 0.0).copied().unwrap_or(0.0);
    top.partial_cmp(&0.0).unwrap_or(Ordering::Equal)
}
