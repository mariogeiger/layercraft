//! Each unsafe move is caught; a safe program passes.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_motion::{Arc, Bead, Layer};
use layercraft_path::Role;

const ENVELOPE: Envelope = Envelope {
    area: [[0.0, 0.0], [256.0, 256.0]],
    max_z: 250.0,
    max_speed: 1000.0,
    filament_diameter: 1.75,
    max_volumetric_speed: 20.0,
};

fn mv(to: [f64; 3], e: f64, speed: f64) -> Move {
    let bead = (e > 0.0).then_some(Bead { role: Role::OUTER_WALL, width: 0.42 });
    Move { to: Some(to), arc: None, e, speed, accel: 1000.0, fan: 0.0, bead }
}

fn arc(to: [f64; 3], centre: [f64; 2], turn: f64) -> Move {
    Move { arc: Some(Arc { centre, turn }), ..mv(to, 0.0, 300.0) }
}

fn program(moves: Vec<Move>) -> Program {
    Program { layers: vec![Layer { index: 0, z: 0.2, height: 0.2, moves }] }
}

/// Over at the hop from wherever the layer starts, down, then a bead.
fn safe() -> Vec<Move> {
    vec![
        mv([10.0, 10.0, 0.6], 0.0, 500.0),
        mv([10.0, 10.0, 0.2], 0.0, 500.0),
        mv([20.0, 10.0, 0.2], 0.3, 50.0),
    ]
}

#[test]
fn a_safe_program_passes() {
    assert_eq!(verify(&program(safe()), &ENVELOPE), vec![]);
}

#[test]
fn each_danger_is_caught() {
    let cases: Vec<(usize, Move)> = vec![
        (2, mv([300.0, 10.0, 0.2], 0.3, 50.0)),
        (2, mv([20.0, 10.0, 0.1], 0.3, 50.0)),
        (1, mv([10.0, 10.0, 0.0], 0.0, 500.0)),
        (2, mv([20.0, 10.0, 0.2], 10.0, 50.0)),
        (2, mv([20.0, 10.0, 0.2], f64::NAN, 50.0)),
        (0, mv([10.0, 10.0, 0.6], 0.0, 5000.0)),
        (0, mv([10.0, 10.0, 999.0], 0.0, 500.0)),
        (2, Move { fan: 2.0, ..mv([20.0, 10.0, 0.2], 0.3, 50.0) }),
        (2, Move { accel: 0.0, ..mv([20.0, 10.0, 0.2], 0.3, 50.0) }),
        // From where the layer starts, unknown, only a travel.
        (0, mv([10.0, 10.0, 0.2], 0.3, 50.0)),
    ];
    for (at, m) in cases {
        let mut moves = safe();
        moves[at] = m.clone();
        assert!(!verify(&program(moves), &ENVELOPE).is_empty(), "{m:?} passed");
    }
    let mut layers = program(safe()).layers;
    layers.push(Layer { index: 1, z: 0.2, height: 0.2, moves: safe() });
    assert!(!verify(&Program { layers: layers.clone() }, &ENVELOPE).is_empty());
    layers[1].z = -1.0;
    assert!(!verify(&Program { layers }, &ENVELOPE).is_empty());
}

#[test]
fn an_arc_turns_inside_the_plate_from_a_known_place_and_ends_on_its_circle() {
    let full = std::f64::consts::TAU;
    let check = |extra: Vec<Move>| {
        let mut moves = safe();
        moves.extend(extra);
        verify(&program(moves), &ENVELOPE)
    };
    // A helix about (20, 11) back over (20, 10) passes.
    assert_eq!(check(vec![arc([20.0, 10.0, 0.6], [20.0, 11.0], full)]), vec![]);
    // A radius of 15 about (20, -5) dips below y = 0.
    assert!(!check(vec![arc([20.0, 10.0, 0.6], [20.0, -5.0], full)]).is_empty());
    // Both ends inside, its belly out: half a turn clockwise about (22, 1) from (24, 1).
    let belly =
        vec![mv([24.0, 1.0, 0.2], 0.0, 300.0), arc([20.0, 1.0, 0.2], [22.0, 1.0], -std::f64::consts::PI)];
    assert!(!check(belly).is_empty());
    let over =
        vec![mv([24.0, 1.0, 0.2], 0.0, 300.0), arc([20.0, 1.0, 0.2], [22.0, 1.0], std::f64::consts::PI)];
    assert_eq!(check(over), vec![]);
    // An end off its circle, or more than a turn.
    assert!(!check(vec![arc([21.0, 10.0, 0.6], [20.0, 11.0], full)]).is_empty());
    assert!(!check(vec![arc([20.0, 10.0, 0.6], [20.0, 11.0], 2.0 * full)]).is_empty());
    // From where the layer starts, unknown, no arc.
    let mut first = safe();
    first.insert(0, arc([10.0, 10.0, 0.8], [10.0, 11.0], full));
    assert!(!verify(&program(first), &ENVELOPE).is_empty());
    // Never below the layer.
    assert!(!check(vec![mv([20.0, 10.0, 0.1], 0.0, 20.0)]).is_empty());
}
