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

#[test]
fn results_lie_on_the_lattice_with_distinct_points() {
    // A rotated, irregular star and its offsets make many intersection points.
    let star: Vec<Point> = (0..40)
        .map(|i| {
            let a = i as f64 * std::f64::consts::TAU / 40.0 + 0.123;
            let r = if i % 2 == 0 { 10.0 } else { 3.7 + 0.01 * i as f64 };
            [128.3 + r * a.cos(), 127.9 + r * a.sin()]
        })
        .collect();
    let region = union_contours(&[star]).unwrap();
    let ring = difference(&offset(&region, 0.37).unwrap(), &offset(&region, -0.21).unwrap()).unwrap();
    let lines = vec![vec![[100.0, 128.0], [160.0, 128.33333333]]];
    let clipped = clip_lines(&lines, &ring).unwrap();
    let contours = ring.iter().flatten().map(|c| (c, true));
    for (points, closed) in contours.chain(clipped.iter().map(|l| (l, false))) {
        for p in points {
            for v in p {
                let units = v / GRID_MM;
                assert!((units - units.round()).abs() < 1e-3, "{v} is off the lattice");
            }
        }
        let n = points.len();
        let steps = if closed { n } else { n - 1 };
        for i in 0..steps {
            let (a, b) = (points[i], points[(i + 1) % n]);
            assert!((a[0] - b[0]).hypot(a[1] - b[1]) >= GRID_MM * 0.999, "repeated point {a:?}");
        }
    }
}

#[test]
fn an_inset_leaves_no_slivers() {
    // A sliver of a rounded corner as a mesh cuts it: very short edges between long ones.
    // Rounding its insets to the lattice once left needles from the inset back to the short
    // edges' corner, two each time.
    let corner = vec![
        [2.989087, 0.0],
        [0.10489, 9.825475],
        [0.039171, 9.545889],
        [0.001481, 9.27145],
        [0.001222, 9.269564],
        [0.001213, 9.26936],
        [0.0, 9.242424],
    ];
    let region = union_contours(&[corner]).unwrap();
    let inset = offset(&region, -0.08).unwrap();
    assert_eq!(inset.len(), 1, "{inset:?}");
    let contour = &inset[0][0];
    let mut closed = contour.clone();
    closed.push(contour[0]);
    let thickness = 2.0 * signed_area(contour).abs() / polyline_length(&closed);
    assert!(thickness > 0.01, "a sliver {thickness} mm thick: {contour:?}");
    // Deeper than the corner is thick, nothing is left.
    assert_eq!(offset(&region, -0.21).unwrap(), Vec::<Shape>::new());
}

#[test]
fn two_roundings_of_one_edge_leave_nothing_between_them() {
    // The same 10 mm square cut twice, one edge rounded a step further out: what lies between
    // the two is a strip one step wide, rounding debris, not a region.
    let a = union_contours(&[square(0.0, 0.0, 10.0)]).unwrap();
    let b = union_contours(&[vec![[0.0, 0.0], [10.0 + GRID_MM, 0.0], [10.0 + GRID_MM, 10.0], [0.0, 10.0]]])
        .unwrap();
    assert_eq!(difference(&b, &a).unwrap(), Vec::<Shape>::new());
    // So is one a few steps wide, left by a chain of operations; one 10 µm wide is a region.
    let b = union_contours(&[vec![[0.0, 0.0], [10.000004, 0.0], [10.000004, 10.0], [0.0, 10.0]]]).unwrap();
    assert_eq!(difference(&b, &a).unwrap(), Vec::<Shape>::new());
    let c = union_contours(&[vec![[0.0, 0.0], [10.01, 0.0], [10.01, 10.0], [0.0, 10.0]]]).unwrap();
    assert!(close(area(&difference(&c, &a).unwrap()), 0.1));
}

#[test]
fn triangles_cover_a_region_with_holes_exactly() {
    // Two islands, one with a hole: 100 − 36 + 9.
    let region =
        union_contours(&[square(0.0, 0.0, 10.0), square(2.0, 2.0, 6.0), square(20.0, 0.0, 3.0)]).unwrap();
    let (points, triangles) = triangulate(&region).unwrap();
    let mut total = 0.0;
    for t in &triangles {
        let [a, b, c] = t.map(|i| points[i as usize]);
        let doubled = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        assert!(doubled > 0.0, "triangle {t:?} is not counterclockwise");
        total += doubled / 2.0;
    }
    assert!(close(total, 73.0), "{total}");
    assert!(triangulate(&Vec::new()).unwrap().1.is_empty());
}
