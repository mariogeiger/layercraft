//! Region operations checked against areas computed by hand.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;

fn square(x: f64, y: f64, side: f64) -> Vec<Point> {
    vec![[x, y], [x + side, y], [x + side, y + side], [x, y + side]]
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6 * (1.0 + a.abs().max(b.abs()))
}

#[test]
fn even_odd_makes_holes_whatever_the_orientation() {
    let mut inner = square(2.0, 2.0, 6.0);
    inner.reverse();
    let region = union_contours(&[square(0.0, 0.0, 10.0), square(2.0, 2.0, 6.0)]).unwrap();
    assert!(close(area(&region), 100.0 - 36.0));
    let reversed = union_contours(&[square(0.0, 0.0, 10.0), inner]).unwrap();
    assert!(close(area(&reversed), 64.0));
}

#[test]
fn booleans_add_up() {
    let a = union_contours(&[square(0.0, 0.0, 4.0)]).unwrap();
    let b = union_contours(&[square(2.0, 2.0, 4.0)]).unwrap();
    assert!(close(area(&union(&a, &b).unwrap()), 28.0));
    assert!(close(area(&intersection(&a, &b).unwrap()), 4.0));
    assert!(close(area(&difference(&a, &b).unwrap()), 12.0));
}

#[test]
fn inset_of_a_square_is_a_smaller_square() {
    let a = union_contours(&[square(0.0, 0.0, 10.0)]).unwrap();
    let inset = offset(&a, -1.0).unwrap();
    assert!(close(area(&inset), 64.0), "{}", area(&inset));
    let grown = offset(&a, 1.0).unwrap();
    assert!(close(area(&grown), 144.0), "{}", area(&grown));
    assert!(offset(&a, -5.5).unwrap().is_empty());
}

#[test]
fn clipped_lines_stay_inside() {
    let a = union_contours(&[square(0.0, 0.0, 10.0)]).unwrap();
    let lines = vec![vec![[-5.0, 5.0], [15.0, 5.0]]];
    let clipped = clip_lines(&lines, &a).unwrap();
    let total: f64 = clipped.iter().map(|l| polyline_length(l)).sum();
    assert!(close(total, 10.0), "{total}");
}

#[test]
fn hostile_coordinates_are_refused() {
    assert_eq!(union_contours(&[vec![[f64::NAN, 0.0]]]), Err(GeomError::NonFinite));
    let far = vec![[0.0, 0.0], [1e9, 0.0], [0.0, 1.0]];
    assert!(matches!(union_contours(&[far]), Err(GeomError::OutOfRange(_))));
    assert!(offset(&Vec::new(), f64::INFINITY).is_err());
}
