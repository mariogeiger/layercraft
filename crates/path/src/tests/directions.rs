//! The plate has no direction: a part turned, moved or mirrored on it is laid turned, moved or
//! mirrored, to the lattice's rounding. Its dense beads, sparse lines and support lines are
//! checked, each against the same part placed another way.

use super::*;

/// Asserts that `lay` lays the beads of `part`, its bodies' outlines, as it lays them placed
/// by `(angle, mirror, shift)` (see `placed`), the beads of the roles `compared` the same
/// curves moved, to `tolerance`.
pub(super) fn assert_beads_move_with_their_part(
    part: &[Vec<Point>],
    (angle, mirror, shift): (f64, bool, Point),
    compared: &dyn Fn(Role) -> bool,
    tolerance: f64,
    lay: &dyn Fn(&[Vec<Point>]) -> Paths,
) {
    let g = placed(&[], angle, mirror, shift).0;
    let moved_part: Vec<Vec<Point>> = part.iter().map(|o| placed(o, angle, mirror, shift).1).collect();
    let (here, there) = (lay(part), lay(&moved_part));
    assert_eq!(here.layers.len(), there.layers.len());
    for (a, b) in here.layers.iter().zip(&there.layers) {
        let curves = |layer: &PathLayer, map: &dyn Fn(Point) -> Point| -> Vec<(Role, Vec<Point>)> {
            layer
                .paths
                .iter()
                .filter(|p| compared(p.role))
                .map(|p| {
                    let mut points: Vec<Point> = p.points.iter().map(|&q| map(q)).collect();
                    points.extend(points.first().copied().filter(|_| p.closed));
                    (p.role, points)
                })
                .collect()
        };
        let (moved, theirs) = (curves(a, &g), curves(b, &|p| p));
        for (from, to) in [(&moved, &theirs), (&theirs, &moved)] {
            for (role, points) in from.iter() {
                let same: Vec<Vec<Point>> =
                    to.iter().filter(|(r, _)| r == role).map(|(_, p)| p.clone()).collect();
                for &p in points {
                    let d = distance_to(p, &same);
                    assert!(d < tolerance, "layer {}, {angle}°: {role:?} at {p:?} is {d} mm off", a.index);
                }
            }
        }
    }
}

#[test]
fn dense_beads_turn_move_and_mirror_with_their_part() {
    // The plate has no direction: the part turned, moved or mirrored on it is laid turned, moved
    // or mirrored, its walls and its dense beads the same curves, to the lattice's rounding. A
    // square is laid alike turned a quarter turn, which nothing of its own tells apart: turned
    // less than an eighth of a turn, its frame turns with it, and with it what the frame
    // settles, as the line of the bead its insets leave at its middle.
    let square = vec![[0.0, 0.0], [6.3, 0.0], [6.3, 6.3], [0.0, 6.3]];
    let parts = [
        (bracket(), [(37.0f64, false, [12.5, -7.0]), (110.0, true, [-30.0, 4.0])]),
        (square, [(37.0, false, [12.5, -7.0]), (20.0, true, [-30.0, 4.0])]),
    ];
    let slab = |outlines: &[Vec<Point>]| slab_beads(&outlines[0]);
    for (part, turn) in parts.iter().flat_map(|(p, turns)| turns.iter().map(move |t| (p, *t))) {
        assert_beads_move_with_their_part(std::slice::from_ref(part), turn, &|_| true, 1e-3, &slab);
    }
}

#[test]
fn sparse_lines_turn_move_and_mirror_with_their_part() {
    // Tall enough for sparse infill: its lines lie in the part's own frame, at the layer's
    // angle from its axis and through its origin, whichever way the part lies on the plate.
    let tall = |outlines: &[Vec<Point>]| {
        let bodies = outlines.iter().map(|o| layercraft_mesh::solids::prism("b", o, 0.0, 6.0)).collect();
        paths_of(&Mesh { bodies, ..Mesh::default() }).1
    };
    for turn in [(37.0, false, [12.5, -7.0]), (110.0, true, [-30.0, 4.0])] {
        let sparse = |role: Role| role == Role::SPARSE;
        assert_beads_move_with_their_part(&[bracket()], turn, &sparse, 1e-3, &tall);
    }
}

#[test]
fn support_lines_turn_move_and_mirror_with_their_part() {
    // A stem under a bar that overhangs on one side only, so the part has no symmetry.
    let supported = |outlines: &[Vec<Point>]| {
        let heights = [(0.0, 4.0), (4.0, 5.0)];
        let bodies = outlines
            .iter()
            .zip(heights)
            .map(|(o, (z0, z1))| layercraft_mesh::solids::prism("b", o, z0, z1))
            .collect();
        let mesh = Mesh { bodies, ..Mesh::default() };
        let sections =
            section(&mesh, &Layering { first_layer_height: 0.2, layer_height: 0.2, foliation: None })
                .unwrap();
        let supports = SupportParams {
            max_bridge: None,
            top_gap: 0.2,
            bottom_gap: 0.2,
            xy_gap: 0.35,
            interface_layers: 2,
            on_plate_only: false,
        };
        let params = RegionParams { supports: Some(supports), ..region_params() };
        let regions = split(&sections, &Field::uniform(params)).unwrap();
        let lines = SupportLines { width: 0.4, gap: 2.0, interface_gap: 0.4 };
        trace(&regions, &Field::uniform(PathParams { support: Some(lines), ..PATHS })).unwrap()
    };
    let part = [
        vec![[0.0, 0.0], [10.0, 0.0], [10.0, 12.0], [0.0, 12.0]],
        vec![[0.0, 0.0], [30.0, 0.0], [18.0, 12.0], [0.0, 12.0]],
    ];
    let compared = |role: Role| role.kind == Kind::Support;
    for turn in [(37.0, false, [12.5, -7.0]), (110.0, true, [-30.0, 4.0])] {
        // A line's end lies where a slanted, lattice-rounded edge cuts it, up to a few lattice
        // steps off along the edge; the lines themselves, 0.6 mm apart at least, lie true.
        assert_beads_move_with_their_part(&part, turn, &compared, 0.05, &supported);
    }
}

#[test]
fn each_part_gets_the_rolls_it_gets_alone() {
    // Two parts on one plate: the pentagon, and a copy turned 40° and moved 40 mm away. Each
    // has the rolls of its own frame, whatever else stands on the plate.
    let (_, copy) = placed(&PENTAGON, 40.0, false, [40.0, 10.0]);
    let both = rolls_over_all(&[PENTAGON.to_vec(), copy.clone()], 30.0, 2);
    let first = rolls_over(&PENTAGON, 30.0, 2);
    let second = rolls_over(&copy, 30.0, 2);
    let lines = |beads: &[Path]| -> Vec<Vec<Point>> { beads.iter().map(|b| b.points.clone()).collect() };
    for k in 0..2 {
        let together = lines(&both[k]);
        for alone in [lines(&first[k]), lines(&second[k])] {
            assert!(!alone.is_empty());
            for p in alone.iter().flatten() {
                assert!(distance_to(*p, &together) < 1e-3, "layer {k}: {p:?} not in the shared plate's");
            }
        }
        for p in together.iter().flatten() {
            let (a, b) = (lines(&first[k]), lines(&second[k]));
            assert!(distance_to(*p, &a).min(distance_to(*p, &b)) < 1e-3, "layer {k}: {p:?} extra");
        }
    }
}
