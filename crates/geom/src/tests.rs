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
    let lines = chordal_axis(&vec![vec![rect(0.0, 0.0, 10.0, 0.5)]], 0.25, 1e-3, &|_| [1.0, 0.0]).unwrap();
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
    let lines = chordal_axis(&wedge, 0.1, 1e-3, &|_| [1.0, 0.0]).unwrap();
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
    let lines =
        chordal_axis(&vec![vec![rect(0.0, 0.0, 10.0, 10.0), hole]], 0.2, 1e-3, &|_| [1.0, 0.0]).unwrap();
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

/// The vertex `(i, j)` of the grid the sampled fields lie on.
fn vertex(i: i64, j: i64) -> Point {
    [0.3 + 0.5 * i as f64, -0.7 + 0.5 * j as f64]
}

/// A field from a function sampled at every vertex of a block of a regular grid, each square
/// split by its diagonal, as stage files from before reliefs had triangles of their own held it.
fn sampled(f: impl Fn(Point) -> f64) -> Relief {
    let values: Vec<f64> =
        (-3..5).flat_map(|j| (-4..5).map(move |i| (i, j))).map(|(i, j)| f(vertex(i, j))).collect();
    Relief::from_grid([0.3, -0.7], 0.5, [-4, -3], [9, 8], &values).unwrap()
}

#[test]
fn a_field_is_linear_on_each_triangle_and_continuous() {
    // A plane is reproduced exactly inside the block.
    let plane = sampled(|p| 0.2 * p[0] - 0.1 * p[1] + 0.05);
    for p in [[0.31, 0.2], [-1.0, -1.5], [1.1, 0.9], [0.8, 0.8 + 1e-12]] {
        assert!(close(plane.at(p), 0.2 * p[0] - 0.1 * p[1] + 0.05), "{p:?}");
        let g = plane.gradient(p);
        assert!(close(g[0], 0.2) && close(g[1], -0.1), "{g:?}");
    }
    // A bump: values at vertices, zero far away, continuous across every kind of edge.
    let bump = sampled(|p| (1.0 - p[0].hypot(p[1]) / 2.0).max(0.0));
    let v = vertex(1, 1);
    assert!(close(bump.at(v), (1.0 - v[0].hypot(v[1]) / 2.0).max(0.0)));
    assert_eq!(bump.at([40.0, 40.0]), 0.0);
    let c = vertex(0, 0);
    for d in [[0.2, 0.0], [0.0, 0.2], [0.13, 0.13], [0.2, 0.2]] {
        let p = [c[0] + d[0], c[1] + d[1]];
        let eps = [1e-9, -1e-9];
        let (a, b) = (bump.at([p[0] + eps[0], p[1] + eps[1]]), bump.at([p[0] + eps[1], p[1] + eps[0]]));
        assert!((a - b).abs() < 1e-8, "a jump at {p:?}: {a} {b}");
    }
}

/// Triangles of no regular grid, moved `shift` mm along x: a fan about `(1, 1)` inside a ring
/// of triangles out to the corners from the sixth on.
fn fan(shift: f64) -> (Vec<Point>, Vec<[u32; 3]>) {
    let points = [
        [1.0, 1.0],
        [2.4, 0.9],
        [1.6, 2.3],
        [-0.1, 1.4],
        [0.6, -0.4],
        [3.9, -0.9],
        [4.4, 2.6],
        [2.2, 3.9],
        [-1.3, 2.9],
        [-1.6, -0.2],
        [0.9, -1.9],
    ];
    let triangles = vec![
        [0, 1, 2],
        [0, 2, 3],
        [0, 3, 4],
        [0, 4, 1],
        [1, 5, 6],
        [1, 6, 2],
        [2, 6, 7],
        [2, 7, 8],
        [2, 8, 3],
        [3, 8, 9],
        [3, 9, 4],
        [4, 9, 10],
        [4, 10, 5],
        [4, 5, 1],
    ];
    (points.into_iter().map(|p: Point| [p[0] + shift, p[1]]).collect(), triangles)
}

/// A relief on the fan moved `shift` mm along x, zero on its outer corners.
fn irregular(shift: f64) -> Relief {
    let (points, triangles) = fan(shift);
    let mut values = vec![0.7, 0.3, -0.2, 0.1, 0.25];
    values.resize(points.len(), 0.0);
    Relief::new(points, triangles, values).unwrap()
}

#[test]
fn a_field_is_linear_between_its_breaks() {
    let bump = sampled(|p| (1.0 - p[0].hypot(p[1]) / 2.0).max(0.0) * 0.7);
    for field in [bump, irregular(0.0)] {
        for (a, b) in [([-3.0, -2.2], [2.7, 1.9]), ([-1.0, 1.0], [4.0, 1.0]), ([2.2, 2.9], [0.9, -1.6])] {
            let at = |t: f64| field.at([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
            let mut ts = vec![0.0];
            ts.extend(field.breaks(a, b));
            ts.push(1.0);
            assert!(ts.len() > 3, "{ts:?}");
            for w in ts.windows(2) {
                assert!(w[0] < w[1]);
                for f in [0.25, 0.5, 0.75] {
                    let (m, e) = (w[0] + f * (w[1] - w[0]), at(w[0]) + f * (at(w[1]) - at(w[0])));
                    assert!((at(m) - e).abs() < 1e-12, "bent inside [{}, {}]", w[0], w[1]);
                }
            }
            assert!(close(field.max_along(a, b), ts.iter().map(|&t| at(t)).fold(f64::MIN, f64::max)));
        }
        // Far from its triangles the field is flat: no break.
        assert!(field.breaks([30.0, 30.0], [40.0, 31.0]).is_empty());
    }
}

#[test]
fn fields_combine_trim_and_check() {
    let one = sampled(|p| if p[0] > 0.0 { 0.1 } else { 0.0 });
    let other = sampled(|p| if p[1] > 0.0 { 0.3 } else { 0.0 });
    let mean = Relief::combine(&one, &other, |a, b| (a + b) / 2.0).unwrap();
    for p in [[1.0, 1.0], [-1.0, 1.0], [1.0, -1.0], [0.4, 0.2]] {
        assert!(close(mean.at(p), (one.at(p) + other.at(p)) / 2.0), "{p:?}");
    }
    let zero = Relief::zero();
    assert_eq!(Relief::combine(&zero, &zero, |a, b| a + b).unwrap(), zero);
    assert_eq!(Relief::combine(&one, &zero, |a, b| a + b).unwrap(), one);
    // Stored, it keeps the triangles where it is not zero: those of the 11 × 10 vertices
    // widened by a ring of zeros, but for the columns left of x = 0.
    let kept = serde_json::to_value(&one).unwrap()["points"].as_array().map_or(0, Vec::len);
    assert!(kept > 0 && kept < 11 * 10, "{kept}");
    // Reliefs apart on the plate, each on triangles of its own, add up side by side; reliefs on
    // triangles of their own that overlap do not.
    let both = Relief::combine(&irregular(0.0), &irregular(20.0), |a, b| a + b).unwrap();
    for p in [[1.0, 1.0], [21.0, 1.0], [2.0, 0.5], [22.0, 0.5], [11.0, 1.0]] {
        assert!(close(both.at(p), irregular(0.0).at(p) + irregular(20.0).at(p)), "{p:?}");
    }
    assert!(Relief::combine(&irregular(0.0), &irregular(0.5), |a, b| a + b).is_err());
    // Corners, values and triangles are checked.
    let square = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
    let halves = vec![[0, 1, 2], [0, 2, 3]];
    assert!(Relief::new(square.clone(), halves.clone(), vec![f64::NAN, 0.0, 0.0, 0.0]).is_err());
    assert!(Relief::new(square.clone(), halves.clone(), vec![0.0; 3]).is_err());
    assert!(Relief::new(square.clone(), vec![[0, 1, 4]], vec![0.0; 4]).is_err());
    assert!(Relief::new(square, halves, vec![0.1, 0.0, 0.0, 0.0]).is_err(), "not zero on its outer edge");
    let plane = sampled(|p| 0.25 * p[0]);
    assert!((plane.steepest() - 0.25).abs() < 1e-9 || plane.steepest() > 0.25, "{}", plane.steepest());
}

#[test]
fn reliefs_on_one_carrier_behave_as_their_own_triangles_alone() {
    // The fan carries two reliefs, one not zero on the six triangles about its corner
    // (1.6, 2.3) alone, one on the six about (0.6, −0.4). Each answers, and is stored, as those
    // triangles alone: read back on a carrier of its own it is the same, even along the edges
    // it shares with triangles where it is zero, which bend it nowhere.
    let (points, triangles) = fan(0.0);
    let carrier = Carrier::new(points, triangles).unwrap();
    let peak = |corner: usize, value: f64| {
        let mut values = vec![0.0; 11];
        values[corner] = value;
        carrier.relief(values).unwrap()
    };
    let (top, low) = (peak(2, -0.2), peak(4, 0.25));
    assert_eq!((top.range(), low.range()), ([-0.2, 0.0], [0.0, 0.25]));
    let alone = |r: &Relief| -> Relief { serde_json::from_str(&serde_json::to_string(r).unwrap()).unwrap() };
    for relief in [&top, &low] {
        let own = alone(relief);
        assert_eq!(&own, relief);
        assert_eq!((relief.facets().count(), own.facets().count()), (6, 6));
        // On the edge from (1, 1) to (2.4, 0.9), and inside triangles about it.
        for p in [[1.7, 0.95], [1.6, 1.3], [1.4, 0.6], [0.2, 2.0], [3.0, 1.0]] {
            assert_eq!(own.at(p), relief.at(p), "{p:?}");
            assert_eq!(own.gradient(p), relief.gradient(p), "{p:?}");
        }
        for (a, b) in [([-1.0, 1.0], [4.0, 1.0]), ([2.2, 2.9], [0.9, -1.6])] {
            assert_eq!(own.breaks(a, b), relief.breaks(a, b));
        }
        for (lo, hi) in [([-1.0, 0.5], [2.0, 1.5]), ([0.0, -1.0], [3.0, 3.0])] {
            assert_eq!(own.range_over(lo, hi), relief.range_over(lo, hi));
        }
        assert_eq!(own.steepest(), relief.steepest());
    }
    assert_ne!(top.gradient([1.7, 0.95]), [0.0, 0.0], "the edge's side where it is not zero");
    // In across the outer edge from (−1.6, −0.2) to (0.9, −1.9), `top` is zero and flat.
    let (a, b) = ([-1.5, -0.5], [0.8, -1.7]);
    assert!(top.breaks(a, b).is_empty() && !low.breaks(a, b).is_empty());
    // Added on their carrier, or each on its own, they are the same relief.
    let sum = Relief::combine(&top, &low, |a, b| a + b).unwrap();
    let apart = Relief::combine(&alone(&top), &alone(&low), |a, b| a + b).unwrap();
    for p in [[1.0, 1.0], [1.7, 0.95], [0.6, -0.4], [1.6, 2.3], [0.4, 0.9]] {
        assert!(close(sum.at(p), top.at(p) + low.at(p)) && close(apart.at(p), sum.at(p)), "{p:?}");
    }
    // Its values are checked: one at each corner, zero on the rim and off the triangles.
    assert!(carrier.relief(vec![0.0; 10]).is_err());
    let mut rim = vec![0.0; 11];
    rim[5] = 0.1;
    assert!(carrier.relief(rim).is_err(), "not zero on the rim");
    let (mut points, triangles) = fan(0.0);
    points.push([9.0, 9.0]);
    let spare = Carrier::new(points, triangles).unwrap();
    let mut off = vec![0.0; 12];
    assert!(spare.relief(off.clone()).unwrap().is_zero());
    off[11] = 0.1;
    assert!(spare.relief(off).is_err(), "not zero off the triangles");
}

#[test]
fn a_relief_from_an_older_stage_file_is_its_grid_split_by_diagonals() {
    let old = r#"{"grid":{"origin":[0.3,-0.7],"cell":0.5},"start":[-1,0],"size":[3,2],"values":[0.1,0.2,0.0,-0.05,0.4,0.3]}"#;
    let read: Relief = serde_json::from_str(old).unwrap();
    // The value the older files' grid gave a point: in its square `(i, j)`, on the lower
    // triangle when `u − i ≥ v − j`.
    let value = |i: i64, j: i64| -> f64 {
        let (di, dj) = (i + 1, j);
        if !(0..3).contains(&di) || !(0..2).contains(&dj) {
            return 0.0;
        }
        [0.1, 0.2, 0.0, -0.05, 0.4, 0.3][(dj * 3 + di) as usize]
    };
    let grid_at = |p: Point| {
        let (u, v) = ((p[0] - 0.3) / 0.5, (p[1] + 0.7) / 0.5);
        let (i, j) = (u.floor() as i64, v.floor() as i64);
        let (fu, fv) = (u - i as f64, v - j as f64);
        let (f00, f11) = (value(i, j), value(i + 1, j + 1));
        if fu >= fv {
            let f10 = value(i + 1, j);
            f00 + fu * (f10 - f00) + fv * (f11 - f10)
        } else {
            let f01 = value(i, j + 1);
            f00 + fv * (f01 - f00) + fu * (f11 - f01)
        }
    };
    for k in 0..200 {
        let p = [-1.5 + 0.0173 * k as f64, -1.3 + 0.0139 * k as f64];
        assert!((read.at(p) - grid_at(p)).abs() < 1e-12, "{p:?}: {} vs {}", read.at(p), grid_at(p));
    }
    // Written back, it keeps its triangles.
    let again: Relief = serde_json::from_str(&serde_json::to_string(&read).unwrap()).unwrap();
    assert_eq!(again, read);
}

#[test]
fn a_fields_range_over_a_box_is_exact() {
    // A plane reaches its extremes at the box's corners.
    let plane = sampled(|p| 0.2 * p[0] - 0.1 * p[1]);
    let [lo, hi] = plane.range_over([-1.0, -1.0], [1.0, 0.5]);
    assert!(close(lo, -0.2 - 0.05) && close(hi, 0.2 + 0.1), "{lo} {hi}");
    // A bump peaks at the vertex (0.3, −0.7) inside the box, between the samples of any finer
    // sampling, and dips to zero past its foot: the range holds every value and no more.
    let bump = sampled(|p| (1.0 - (p[0] - 0.3).hypot(p[1] + 0.7) / 1.5).max(0.0));
    for (a, b) in [([-0.11, -1.37], [0.93, -0.02]), ([0.8, -0.9], [1.6, 0.4]), ([-3.0, -3.0], [3.0, 3.0])] {
        let [lo, hi] = bump.range_over(a, b);
        let n = 300;
        let (mut min, mut max) = (f64::INFINITY, f64::NEG_INFINITY);
        for j in 0..=n {
            for i in 0..=n {
                let (s, t) = (i as f64 / n as f64, j as f64 / n as f64);
                let v = bump.at([a[0] + s * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
                (min, max) = (min.min(v), max.max(v));
            }
        }
        // Sampled at 1/300 of the box, the bump, 2/3 mm per mm steep, strays less than that.
        let stray = (2.0 / 3.0) * (b[0] - a[0]).hypot(b[1] - a[1]) / n as f64;
        assert!(lo <= min + 1e-12 && min - lo <= stray, "{a:?}–{b:?}: {lo} under {min}");
        assert!(hi >= max - 1e-12 && hi - max <= stray, "{a:?}–{b:?}: {hi} over {max}");
    }
    assert!(close(bump.range_over([-0.11, -1.37], [0.93, -0.02])[1], 1.0));
}

#[test]
fn a_point_is_on_one_side_of_a_line_whoever_asks() {
    // Points within a few units in the last place of the line through two others, where the
    // plain determinant may give three points named in another order another sign: the exact
    // one gives them all one side, and zero only on the line.
    let (a, b) = ([0.5, 0.5], [12.0, 12.0]);
    let mut seen = [0usize; 3];
    for k in 0..64 {
        let x = 0.5 + 0.1 * k as f64;
        for d in -4i64..=4 {
            let c = [x, f64::from_bits((x.to_bits() as i64 + d) as u64)];
            let sign = orientation(a, b, c);
            assert_eq!(orientation(b, c, a), sign, "{c:?}");
            assert_eq!(orientation(c, a, b), sign, "{c:?}");
            assert_eq!(orientation(b, a, c), sign.reverse(), "{c:?}");
            if d == 0 {
                assert_eq!(sign, std::cmp::Ordering::Equal, "{c:?} lies on the line y = x");
            } else {
                assert_eq!(sign, if d > 0 { std::cmp::Ordering::Greater } else { std::cmp::Ordering::Less });
            }
            seen[(sign as i8 + 1) as usize] += 1;
        }
    }
    assert!(seen.iter().all(|&n| n > 0), "{seen:?}");
}

#[test]
fn a_beads_strip_through_its_layer_holds_its_section() {
    for (w, h) in [(0.4, 0.2), (0.45, 0.1), (0.62, 0.3)] {
        assert!(close(bead_spacing(w, h) * h, bead_area(w, h)), "{w} {h}");
    }
}

#[test]
fn a_line_clipped_runs_on_as_one_piece_until_it_leaves_the_region() {
    let a = union_contours(&[square(0.0, 0.0, 10.0)]).unwrap();
    // Turning back leftward then rightward, twice, inside: one piece, the line itself, either way.
    let zigzag = vec![[8.0, 1.0], [2.0, 3.0], [8.0, 5.0], [2.0, 7.0], [8.0, 9.0]];
    let backward: Vec<Point> = zigzag.iter().rev().copied().collect();
    for line in [zigzag, backward] {
        assert_eq!(clip_lines(std::slice::from_ref(&line), &a).unwrap(), vec![line]);
    }
    // Out through the right side and back in: two pieces, each running the way the line runs.
    let out = vec![[8.0, 1.0], [2.0, 3.0], [12.0, 5.0], [2.0, 7.0], [8.0, 9.0]];
    let pieces = clip_lines(&[out], &a).unwrap();
    assert_eq!(pieces.len(), 2, "{pieces:?}");
    assert_eq!((pieces[0][0], pieces[1][pieces[1].len() - 1]), ([8.0, 1.0], [8.0, 9.0]), "{pieces:?}");
    // A loop half outside, closed at a corner inside: what lies inside, in one piece.
    let ring = vec![[5.0, 5.0], [15.0, 5.0], [15.0, 8.0], [5.0, 8.0], [5.0, 5.0]];
    let pieces = clip_lines(&[ring], &a).unwrap();
    assert_eq!(pieces, vec![vec![[10.0, 8.0], [5.0, 8.0], [5.0, 5.0], [10.0, 5.0]]]);
    // Wherever it starts inside, a ring goes on through its start: the same piece.
    let ring = vec![[5.0, 8.0], [5.0, 5.0], [15.0, 5.0], [15.0, 8.0], [5.0, 8.0]];
    let pieces = clip_lines(&[ring], &a).unwrap();
    assert_eq!(pieces, vec![vec![[10.0, 8.0], [5.0, 8.0], [5.0, 5.0], [10.0, 5.0]]]);
}

/// A mirror across the x axis when `mirror`, then a turn by `angle` degrees and a move by `shift`.
fn isometry(angle: f64, mirror: bool, shift: Point) -> impl Fn(Point) -> Point {
    let (s, c) = angle.to_radians().sin_cos();
    move |p: Point| {
        let p = if mirror { [p[0], -p[1]] } else { p };
        [c * p[0] - s * p[1] + shift[0], s * p[0] + c * p[1] + shift[1]]
    }
}

/// A region moved by `g`, each contour reversed where `g` mirrors, so that it keeps its turn.
fn moved(region: &Region, g: &dyn Fn(Point) -> Point, mirror: bool) -> Region {
    let contour = |c: &Contour| {
        let mut c: Contour = c.iter().map(|&p| g(p)).collect();
        if mirror {
            c.reverse();
        }
        c
    };
    region.iter().map(|shape| shape.iter().map(contour).collect()).collect()
}

/// The farthest any vertex of either region lies from the nearest vertex of the other.
fn vertices_apart(a: &Region, b: &Region) -> f64 {
    let far = |from: &Region, to: &Region| {
        let near = |p: &Point| {
            to.iter().flatten().flatten().map(|q| (p[0] - q[0]).hypot(p[1] - q[1])).fold(f64::MAX, f64::min)
        };
        from.iter().flatten().flatten().map(near).fold(0.0, f64::max)
    };
    far(a, b).max(far(b, a))
}

#[test]
fn a_disc_offset_turns_and_mirrors_with_its_region() {
    // A region with no symmetry, with sharp, blunt and reflex corners and a hole: grown or shrunk,
    // then turned or mirrored, it is grown or shrunk once turned or mirrored, each arc cut into
    // the same chords, to the lattice's rounding.
    let outline = vec![[0.0, 0.0], [9.0, 0.0], [9.5, 4.0], [5.0, 3.0], [6.0, 7.5], [1.0, 6.0]];
    let region = union_contours(&[outline, vec![[2.0, 1.5], [2.5, 3.5], [3.6, 2.0]]]).unwrap();
    for (angle, mirror) in [(37.0, false), (110.0, true)] {
        let g = isometry(angle, mirror, [12.5, -7.0]);
        let there = moved(&region, &g, mirror);
        for d in [-1.1, -0.3, 0.2, 0.9, 2.5] {
            let (a, b) =
                (moved(&disc_offset(&region, d).unwrap(), &g, mirror), disc_offset(&there, d).unwrap());
            let count = |r: &Region| r.iter().flatten().map(Vec::len).sum::<usize>();
            assert_eq!((a.len(), count(&a)), (b.len(), count(&b)), "offset {d}, {angle}°");
            let gap = vertices_apart(&a, &b);
            assert!(gap < 1e-5, "offset {d}, {angle}°: vertices {gap} mm apart");
        }
    }
}

/// The distance from `p` to the boundary of `region`.
fn to_boundary(p: Point, region: &Region) -> f64 {
    region
        .iter()
        .flatten()
        .flat_map(|c| edges(c))
        .map(|(a, b)| segment_distance(p, a, b))
        .fold(f64::MAX, f64::min)
}

#[test]
fn a_disc_offset_of_a_square_is_its_sum_with_a_disc() {
    // Grown by r, a square of side a covers a² + 4ar + πr², less the slivers the chords cut off
    // its arcs, under RESOLUTION_MM deep along their 2πr; shrunk by r, the square of side
    // a − 2r. Grown and shrunk back, it is itself but at its corners: the lines of the first
    // and last chords of each arc, as far inside it as a chord's middle, under RESOLUTION_MM,
    // meet back past the corner, √2 times that a quarter turn apart.
    let a = union_contours(&[square(0.0, 0.0, 10.0)]).unwrap();
    for r in [0.05, 0.4, 3.0] {
        let grown = disc_offset(&a, r).unwrap();
        let (got, exact) = (area(&grown), 100.0 + 40.0 * r + std::f64::consts::PI * r * r);
        let cut = 2.0 * std::f64::consts::PI * r * RESOLUTION_MM;
        assert!(got <= exact + 1e-9 && exact - got <= cut, "{r}: {got} of {exact}");
        let shrunk = area(&disc_offset(&a, -r).unwrap());
        assert!(close(shrunk, (10.0 - 2.0 * r).powi(2)), "{r}: {shrunk}");
        let back = disc_offset(&grown, -r).unwrap();
        let off = back.iter().flatten().flatten().map(|&p| to_boundary(p, &a)).fold(0.0, f64::max);
        let corners = a.iter().flatten().flatten().map(|&p| to_boundary(p, &back)).fold(0.0, f64::max);
        assert!(
            off < RESOLUTION_MM && corners <= std::f64::consts::SQRT_2 * RESOLUTION_MM,
            "{r}: {off} {corners}"
        );
    }
}

#[test]
fn a_regions_own_lines_turn_and_mirror_with_it() {
    let polygon = |n: usize, r: f64, phase: f64| -> Region {
        let points = (0..n).map(|k| {
            let t = phase.to_radians() + std::f64::consts::TAU * k as f64 / n as f64;
            [5.0 + r * t.cos(), 3.0 + r * t.sin()]
        });
        union_contours(&[points.collect()]).unwrap()
    };
    let lines = |region: &Region| {
        let moments = Moments::of(region.iter().map(|s| (s.as_slice(), 1.0))).unwrap();
        moments.axes.map(|axes| {
            (axes.count, axes.lines().map(|u| u[1].atan2(u[0]).to_degrees()).collect::<Vec<f64>>())
        })
    };
    let apart = |a: f64, b: f64| {
        let d = (a - b).rem_euclid(180.0);
        d.min(180.0 - d)
    };
    // The long side of a rectangle, turned or mirrored with it.
    let rectangle = union_contours(&[rect(0.0, 0.0, 3.0, 1.0)]).unwrap();
    for (angle, mirror) in [(25.0, false), (110.0, true)] {
        let (count, at) = lines(&moved(&rectangle, &isometry(angle, mirror, [1.0, 2.0]), mirror)).unwrap();
        assert!(count == 1 && apart(at[0], angle) < 1e-4, "{angle}°: {at:?}");
    }
    // A square spreads alike every way: its two sides; an equilateral triangle and a hexagon,
    // their three edges' lines; a polygon of many sides, as a disc, none.
    for (n, phase, count) in [(4, 70.0, 2), (3, 10.0, 3), (6, 10.0, 3)] {
        let (got, at) = lines(&polygon(n, 2.0, phase)).unwrap();
        // The first edge runs from the vertex at `phase` to the next, square to their bisector.
        let edge = phase + 90.0 + 180.0 / n as f64;
        assert_eq!(got, count, "{n}-gon: {at:?}");
        for k in 0..count {
            let want = edge + 180.0 * k as f64 / count as f64;
            assert!(at.iter().any(|&a| apart(a, want) < 1e-4), "{n}-gon: {at:?}, not {want}");
        }
    }
    assert_eq!(lines(&polygon(720, 2.0, 3.0)), None);
    // Flats a hundredth of a millimetre long on a disc are no square's sides.
    assert_eq!(lines(&disc_offset(&union_contours(&[square(0.0, 0.0, 0.01)]).unwrap(), 0.3).unwrap()), None);
    // A right triangle skews towards its sharper corner, along its longer leg.
    let wedge = union_contours(&[vec![[0.0, 0.0], [3.0, 0.0], [0.0, 1.0]]]).unwrap();
    let moments = Moments::of(wedge.iter().map(|s| (s.as_slice(), 1.0))).unwrap();
    assert_eq!(moments.most_skewed([[-1.0, 0.0], [1.0, 0.0]].into_iter()), Some([1.0, 0.0]));
    // Two posts 2 and 1 mm deep under a beam 2 mm deep skew, slightly, toward the deeper post's
    // side: so does the part turned, and the other way mirrored, however little they skew
    // against how far apart they stand.
    let part = |angle: f64, mirror: bool| {
        let (s, c) = angle.to_radians().sin_cos();
        let place = |p: [f64; 2]| {
            let p = if mirror { [p[0], -p[1]] } else { p };
            [c * p[0] - s * p[1] + 3.0, s * p[0] + c * p[1] - 1.0]
        };
        [(rect(0.0, 0.0, 2.0, 2.0), 1.0), (rect(7.0, 0.0, 10.0, 1.0), 1.0), (rect(0.0, 0.0, 10.0, 2.0), 0.6)]
            .map(|(r, w)| {
                let mut outline: Vec<[f64; 2]> = r.into_iter().map(place).collect();
                if mirror {
                    outline.reverse();
                }
                (union_contours(&[outline]).unwrap(), w)
            })
    };
    let side = |angle: f64, mirror: bool| {
        let shapes = part(angle, mirror);
        let moments =
            Moments::of(shapes.iter().flat_map(|(r, w)| r.iter().map(move |s| (s.as_slice(), *w)))).unwrap();
        let (s, c) = angle.to_radians().sin_cos();
        let (up, down) = ([-s, c], [s, -c]);
        moments.most_skewed([up, down].into_iter()) == Some(up)
    };
    let upward = side(0.0, false);
    assert_eq!(side(37.0, false), upward);
    assert_eq!(side(110.0, true), !upward);
    assert!(
        close(moments.centroid[0], 1.0) && close(moments.centroid[1], 1.0 / 3.0) && close(moments.area, 1.5)
    );
}

#[test]
fn a_shape_too_short_for_a_strip_lies_along_its_own_line() {
    // A rounded square half a millimetre across, as the insets of a square leave at its middle:
    // one line along a side through its centre, edge to edge, as thick as its area over that
    // chord; of its two sides the one nearer the part's frame axis, turned or mirrored with it.
    let blob = disc_offset(&union_contours(&[square(-0.15, -0.15, 0.3)]).unwrap(), 0.1).unwrap();
    // A disc has no line of its own: it lies along the frame's axis. A short strip lies along
    // its length, whatever the frame's axis.
    let circle = (0..720).map(|k| {
        let t = std::f64::consts::TAU * k as f64 / 720.0;
        [0.25 * t.cos(), 0.25 * t.sin()]
    });
    let disc = union_contours(&[circle.collect()]).unwrap();
    let strip = disc_offset(&union_contours(&[rect(-0.2, -0.125, 0.2, 0.125)]).unwrap(), 0.1).unwrap();
    for (shape, frame, ends) in [
        (&blob, 10.0f64, [0.25, 0.0]),
        (&disc, 33.0, [0.25 * 33f64.to_radians().cos(), 0.25 * 33f64.to_radians().sin()]),
        (&strip, 80.0, [0.3, 0.0]),
    ] {
        for (angle, mirror) in [(0.0, false), (37.0, false), (110.0, true)] {
            let g = isometry(angle, mirror, [3.0, -2.0]);
            let o = g([0.0, 0.0]);
            let (u, v) = (g([frame.to_radians().cos(), frame.to_radians().sin()]), g(ends));
            let axis = [u[0] - o[0], u[1] - o[1]];
            let lines =
                chordal_axis(&moved(shape, &g, mirror), std::f64::consts::PI * 0.2 / 4.0, 0.012, &|_| axis)
                    .unwrap();
            assert_eq!(lines.len(), 1, "{angle}°: {lines:?}");
            let line = &lines[0];
            let tips = [v, [2.0 * o[0] - v[0], 2.0 * o[1] - v[1]]];
            assert_eq!(line.points.len(), 2, "{angle}°: {line:?}");
            for p in &line.points {
                let off = tips.iter().map(|t| (p[0] - t[0]).hypot(p[1] - t[1])).fold(f64::MAX, f64::min);
                assert!(off < 1e-5, "{angle}°, frame {frame}°: {p:?} is {off} mm from the ends {tips:?}");
            }
            let chord = 2.0 * (ends[0].hypot(ends[1]));
            for t in &line.thickness {
                assert!((t - area(shape) / chord).abs() < 1e-5, "{angle}°: thickness {t}");
            }
        }
    }
}

#[test]
fn a_bridge_across_a_turned_slab_goes_across_the_turned_slab() {
    // A slab 20 × 6 mm, nothing hanging at the strands' ends: the shortest strands go across
    // its width, whichever way the slab lies on the plate, and the lines of any angle lie on
    // the grid through the slab's own centre.
    let rectangle = [[-10.0, -3.0], [10.0, -3.0], [10.0, 3.0], [-10.0, 3.0]];
    for turn in [0.0f64, 37.0, 110.0, -20.0] {
        let (s, c) = turn.to_radians().sin_cos();
        let outline: Vec<Point> =
            rectangle.iter().map(|p| [c * p[0] - s * p[1] + 5.0, s * p[0] + c * p[1] - 2.0]).collect();
        let region = union_contours(&[outline]).unwrap();
        let preferred = turn + 15.0;
        let phase = phase_through(preferred, [5.0, -2.0]);
        let angle = bridge_angle(&region, 0.4, preferred, phase, |_| false).unwrap();
        let across = (turn + 90.0).rem_euclid(180.0);
        assert!((angle - across).abs() < 1e-4, "{turn}°: {angle}° for {across}°");
    }
}

#[test]
fn cells_sharing_edges_are_cut_into_even_triangles_that_meet() {
    // A 4 × 3 rectangle and a 3 × 2 one against the middle of its right side: the smaller
    // one's corners lie on the larger one's side, which must pass through them.
    let cells = |g: &dyn Fn(Point) -> Point, mirror: bool| -> Vec<(Region, bool)> {
        [rect(0.0, 0.0, 4.0, 3.0), rect(4.0, 0.5, 7.0, 2.5)]
            .into_iter()
            .map(|r| {
                let mut c: Vec<Point> = r.into_iter().map(g).collect();
                if mirror {
                    c.reverse();
                }
                (union_contours(&[c]).unwrap(), true)
            })
            .collect()
    };
    let spacing = 0.25;
    let mesh = triangulate_evenly(&cells(&|p| p, false), spacing).unwrap();
    let corner = |i: u32| mesh.points[i as usize];
    let mut uses: std::collections::HashMap<(u32, u32), usize> = Default::default();
    let (mut total, mut longest) = (0.0, 0.0f64);
    for t in &mesh.triangles {
        let [a, b, c] = t.map(corner);
        let twice = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        assert!(twice > 0.0, "a triangle turns clockwise: {a:?} {b:?} {c:?}");
        total += twice / 2.0;
        for (p, q) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *uses.entry((p.min(q), p.max(q))).or_default() += 1;
            longest = longest.max((corner(p)[0] - corner(q)[0]).hypot(corner(p)[1] - corner(q)[1]));
        }
    }
    assert!(close(total, 12.0 + 6.0), "{total}");
    assert!(uses.values().all(|&n| n <= 2), "an edge with three triangles");
    // The edges used once run around the two cells together, 4 + 3 + 4 + 3 + 3 + 2 + 3 + … mm.
    let outline: f64 = uses
        .iter()
        .filter(|(_, n)| **n == 1)
        .map(|(&(p, q), _)| (corner(p)[0] - corner(q)[0]).hypot(corner(p)[1] - corner(q)[1]))
        .sum();
    assert!(close(outline, 2.0 * (4.0 + 3.0) + 2.0 * 3.0), "{outline}");
    assert!(longest <= std::f64::consts::SQRT_2 * spacing + 1e-6, "{longest}");
    // Turned or mirrored, the cells are cut at the turned or mirrored points.
    for (angle, mirror) in [(37.0, false), (110.0, true)] {
        let g = isometry(angle, mirror, [3.0, -5.0]);
        let there = triangulate_evenly(&cells(&g, mirror), spacing).unwrap();
        assert_eq!(there.points.len(), mesh.points.len(), "{angle}°");
        let moved: Vec<Point> = mesh.points.iter().map(|&p| g(p)).collect();
        let far = there
            .points
            .iter()
            .map(|p| moved.iter().map(|q| (p[0] - q[0]).hypot(p[1] - q[1])).fold(f64::MAX, f64::min))
            .fold(0.0, f64::max);
        assert!(far < 1e-4, "{angle}°: a corner {far} mm from where it turned to");
    }
}
