//! A lift turns the way the nozzle turns, so a program mirrored or turned on the plate lifts
//! mirrored or turned: the plate has no left of its own.

use super::*;
use layercraft_geom::Point;

/// The first helix of a program of one layer of the straight `lines`, in order,
fn first_helix(lines: &[[Point; 2]]) -> Arc {
    let path = |ends: &[Point; 2]| Path {
        role: Role::SPARSE,
        width: 0.4,
        closed: false,
        points: ends.to_vec(),
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let paths = Paths {
        carrier: Default::default(),
        layers: vec![PathLayer {
            index: 0,
            top: 0.2,
            height: 0.2,
            warp: Default::default(),
            paths: lines.iter().map(path).collect(),
            material: 0,
        }],
    };
    let mut p = params();
    p.lift_radius = 1.0;
    p.wipe_distance = 1.0;
    let program = compiled(&paths, &p);
    let lift = chained(&program.layers[0].moves, None).find(|(f, m)| climbs(*f, m)).unwrap().1;
    lift.arc.unwrap()
}

/// Where a map of the plate takes the helix's centre and turn.
fn mapped(arc: Arc, to: &dyn Fn(Point) -> Point, mirror: bool) -> Arc {
    Arc { centre: to(arc.centre), turn: if mirror { -arc.turn } else { arc.turn } }
}

#[test]
fn a_lift_turns_the_way_the_travel_turns_and_mirrors_with_it() {
    let (left, right) = ([[10.0, 10.0], [20.0, 10.0]], [[10.0, -10.0], [20.0, -10.0]]);
    let turning_left = first_helix(&[left, right]);
    assert_eq!(turning_left.turn, std::f64::consts::TAU);
    // Its mirror image across the x axis turns the other way, about the mirrored axis.
    let flip = |p: Point| [p[0], -p[1]];
    let mirrored = |ends: [Point; 2]| ends.map(flip);
    let turning_right = first_helix(&[mirrored(left), mirrored(right)]);
    assert_eq!(turning_right.turn, -std::f64::consts::TAU);
    let image = mapped(turning_left, &flip, true);
    assert!(
        (image.centre[0] - turning_right.centre[0]).abs() < 1e-9
            && (image.centre[1] - turning_right.centre[1]).abs() < 1e-9,
        "{image:?} against {turning_right:?}"
    );
    // Straight on, or straight back, it keeps to the left.
    let straight = first_helix(&[left, [[30.0, 10.0], [40.0, 10.0]]]);
    assert_eq!(straight.turn, std::f64::consts::TAU);
}

#[test]
fn a_program_turned_on_the_plate_lifts_in_the_turned_helix() {
    let (s, c) = 37f64.to_radians().sin_cos();
    let turn = |p: Point| [c * p[0] - s * p[1] + 12.5, s * p[0] + c * p[1] - 7.0];
    let lines = [[[10.0, 10.0], [20.0, 10.0]], [[10.0, 30.0], [20.0, 30.0]]];
    let there = lines.map(|ends| ends.map(turn));
    let (arc, turned) = (first_helix(&lines), first_helix(&there));
    let image = mapped(arc, &turn, false);
    assert_eq!(arc.turn, turned.turn);
    assert!(
        (image.centre[0] - turned.centre[0]).abs() < 1e-9
            && (image.centre[1] - turned.centre[1]).abs() < 1e-9,
        "{image:?} against {turned:?}"
    );
}
