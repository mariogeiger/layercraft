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

/// A field from a function sampled at every vertex of a block.
fn sampled(f: impl Fn(Point) -> f64) -> Relief {
    let grid = Grid { origin: [0.3, -0.7], cell: 0.5 };
    Relief::from_fn(grid, [-4, -3], [9, 8], |i, j| f(grid.vertex(i, j)))
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
    // A bump: values at vertices, zero far away, continuous across every kind of line.
    let bump = sampled(|p| (1.0 - p[0].hypot(p[1]) / 2.0).max(0.0));
    let v = bump.grid.vertex(1, 1);
    assert!(close(bump.at(v), (1.0 - v[0].hypot(v[1]) / 2.0).max(0.0)));
    assert_eq!(bump.at([40.0, 40.0]), 0.0);
    let c = bump.grid.vertex(0, 0);
    for d in [[0.2, 0.0], [0.0, 0.2], [0.13, 0.13], [0.2, 0.2]] {
        let p = [c[0] + d[0], c[1] + d[1]];
        let eps = [1e-9, -1e-9];
        let (a, b) = (bump.at([p[0] + eps[0], p[1] + eps[1]]), bump.at([p[0] + eps[1], p[1] + eps[0]]));
        assert!((a - b).abs() < 1e-8, "a jump at {p:?}: {a} {b}");
    }
}

#[test]
fn a_field_is_linear_between_its_breaks() {
    let bump = sampled(|p| (1.0 - p[0].hypot(p[1]) / 2.0).max(0.0) * 0.7);
    let (a, b) = ([-3.0, -2.2], [2.7, 1.9]);
    let at = |t: f64| bump.at([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
    let mut ts = vec![0.0];
    ts.extend(bump.breaks(a, b));
    ts.push(1.0);
    assert!(ts.len() > 10, "{ts:?}");
    for w in ts.windows(2) {
        assert!(w[0] < w[1]);
        let (m, e) = ((w[0] + w[1]) / 2.0, (at(w[0]) + at(w[1])) / 2.0);
        assert!((at(m) - e).abs() < 1e-12, "bent inside [{}, {}]", w[0], w[1]);
    }
    assert!(close(bump.max_along(a, b), ts.iter().map(|&t| at(t)).fold(f64::MIN, f64::max)));
    // Far from the block the field is flat: no break.
    assert!(bump.breaks([30.0, 30.0], [40.0, 31.0]).is_empty());
}

#[test]
fn fields_combine_trim_and_check() {
    let one = sampled(|p| if p[0] > 0.0 { 0.1 } else { 0.0 });
    let other = sampled(|p| if p[1] > 0.0 { 0.3 } else { 0.0 });
    let mean = Relief::combine(&one, &other, |a, b| (a + b) / 2.0).unwrap();
    for p in [[1.0, 1.0], [-1.0, 1.0], [1.0, -1.0], [0.4, 0.2]] {
        assert!(close(mean.at(p), (one.at(p) + other.at(p)) / 2.0), "{p:?}");
    }
    let zero = Relief::zero(one.grid);
    assert_eq!(Relief::combine(&zero, &zero, |a, b| a + b).unwrap(), zero);
    assert_eq!(Relief::combine(&one, &zero, |a, b| a + b).unwrap(), one);
    assert!(one.size[0] < 9, "trimmed to its non-zero columns: {:?}", one.size);
    assert!(one.check().is_ok());
    let bad = Relief { values: vec![f64::NAN; 4], size: [2, 2], ..one.clone() };
    assert!(bad.check().is_err());
    let short = Relief { values: vec![0.0; 3], size: [2, 2], ..one };
    assert!(short.check().is_err());
    let plane = sampled(|p| 0.25 * p[0]);
    assert!((plane.steepest() - 0.25).abs() < 1e-9 || plane.steepest() > 0.25, "{}", plane.steepest());
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
