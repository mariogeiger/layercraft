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

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Point> {
    vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
}

#[test]
fn a_strip_has_one_centre_line_as_thick_as_it_is_wide() {
    let lines = chordal_axis(&vec![vec![rect(0.0, 0.0, 10.0, 0.5)]], 0.25, 1e-3).unwrap();
    assert_eq!(lines.len(), 1);
    let line = &lines[0];
    assert!(!line.closed);
    // Straight down the middle, as thick as the strip all along, its ends run on to the strip's.
    for (p, t) in line.points.iter().zip(&line.thickness) {
        assert!((p[1] - 0.25).abs() < 1e-6, "{p:?} off the centre line");
        assert!((t - 0.5).abs() < 1e-6, "thickness {t} at {p:?}");
    }
    let xs: Vec<f64> = line.points.iter().map(|p| p[0]).collect();
    let (lo, hi) = (xs.iter().copied().fold(f64::MAX, f64::min), xs.iter().copied().fold(f64::MIN, f64::max));
    assert!(lo.abs() < 1e-6 && (hi - 10.0).abs() < 1e-6, "the line spans the strip: {lo}..{hi}");
}

#[test]
fn a_wedge_has_its_centre_line_to_its_tip() {
    // A thin isosceles wedge, 0.6 mm at its base, 6 mm long: the line runs to the tip, thinning
    // with it.
    let wedge = vec![vec![vec![[0.0, 0.0], [6.0, 0.3], [0.0, 0.6]]]];
    let lines = chordal_axis(&wedge, 0.1, 1e-3).unwrap();
    assert_eq!(lines.len(), 1, "{lines:?}");
    let line = &lines[0];
    for (p, t) in line.points.iter().zip(&line.thickness) {
        assert!((p[1] - 0.3).abs() < 0.02, "{p:?} off the centre line");
        // The inscribed disc at x: the half angle's sine times the distance to the tip, twice.
        let expected = 2.0 * (6.0 - p[0]) * (0.3f64).atan2(6.0).sin();
        assert!(*t <= 0.6 + 1e-6 && (t - expected).abs() < 0.05, "thickness {t} at {p:?}, not {expected}");
    }
    let hi = line.points.iter().map(|p| p[0]).fold(f64::MIN, f64::max);
    assert!(hi > 5.9, "the line stops at {hi}, short of the tip");
}

#[test]
fn a_ring_has_a_closed_centre_line() {
    let mut hole = rect(0.4, 0.4, 9.6, 9.6);
    hole.reverse();
    let lines = chordal_axis(&vec![vec![rect(0.0, 0.0, 10.0, 10.0), hole]], 0.2, 1e-3).unwrap();
    assert_eq!(lines.len(), 1, "{lines:?}");
    assert!(lines[0].closed);
    for (p, t) in lines[0].points.iter().zip(&lines[0].thickness) {
        let d = [p[0] - 0.2, 9.8 - p[0], p[1] - 0.2, 9.8 - p[1]].iter().fold(f64::MAX, |m, v| m.min(v.abs()));
        assert!(d < 1e-6, "{p:?} off the ring's middle");
        assert!(*t > 0.3 && *t < 0.6, "thickness {t} at {p:?}");
    }
}

#[test]
fn a_segment_lies_in_a_region_along_its_edge_but_not_across_its_hole() {
    let ring = vec![vec![square(0.0, 0.0, 10.0), square(4.0, 4.0, 2.0).into_iter().rev().collect()]];
    let bands = Bands::new(&ring);
    assert!(bands.encloses([1.0, 1.0]) && !bands.encloses([5.0, 5.0]) && !bands.encloses([11.0, 1.0]));
    assert!(bands.covers([10.0, 3.0]) && bands.covers([4.0, 5.0]) && !bands.covers([5.0, 5.0]));
    assert!(bands.holds([1.0, 1.0], [9.0, 2.0]));
    assert!(bands.holds([0.0, 0.0], [10.0, 0.0]), "along the edge");
    assert!(bands.holds([4.0, 1.0], [4.0, 9.0]), "along the hole's side");
    assert!(!bands.holds([1.0, 5.0], [9.0, 5.0]), "across the hole");
    assert!(!bands.holds([9.0, 1.0], [11.0, 1.0]), "out of the side");
    // Through a corner and out: its middle lies inside, the piece past the corner does not.
    assert!(!bands.holds([7.0, 7.0], [11.0, 11.0]));
    // Along the hole's diagonal, through two of its corners: the piece between runs inside it.
    assert!(!bands.holds([2.0, 8.0], [8.0, 2.0]));
    assert!(bands.holds([2.0, 8.0], [4.0, 6.0]) && bands.holds([6.0, 4.0], [8.0, 2.0]));
    assert!(bands.holds([3.0, 7.0], [5.0, 7.0]), "touching the hole's corner");
}
