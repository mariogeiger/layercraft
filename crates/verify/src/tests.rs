//! Each unsafe move is caught; a safe program passes, on flat layers and on bent ones.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::{Grid, Relief};
use layercraft_motion::{Arc, Bead, Layer};
use layercraft_path::Role;

const ENVELOPE: Envelope = Envelope {
    area: [[0.0, 0.0], [256.0, 256.0]],
    max_z: 250.0,
    max_speed: 1000.0,
    max_speed_z: 20.0,
    filament_diameter: 1.75,
    max_volumetric_speed: 20.0,
    toolhead: Toolhead {
        tip_radius: 0.2,
        flare: 1.0,
        nozzle_height: 4.0,
        head_radius: 30.0,
        gantry_height: 30.0,
        margin: 1.0,
    },
};

fn mv(to: [f64; 3], e: f64, speed: f64) -> Move {
    let bead = (e > 0.0).then_some(Bead { role: Role::OUTER_WALL, width: 0.42 });
    Move { to: Some(to), arc: None, e, speed, accel: 1000.0, fan: 0.0, bead, wait: 0.0 }
}

fn arc(to: [f64; 3], centre: [f64; 2], turn: f64) -> Move {
    Move { arc: Some(Arc { centre, turn }), ..mv(to, 0.0, 300.0) }
}

fn program(moves: Vec<Move>) -> Program {
    Program { layers: vec![Layer { index: 0, z: 0.2, height: 0.2, warp: Default::default(), moves }] }
}

/// Over at the hop from wherever the layer starts, down as fast as the Z axis goes, then a bead.
fn safe() -> Vec<Move> {
    vec![
        mv([10.0, 10.0, 0.6], 0.0, 500.0),
        mv([10.0, 10.0, 0.2], 0.0, 20.0),
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
        (1, mv([10.0, 10.0, 0.0], 0.0, 20.0)),
        // Down faster than the Z axis goes.
        (1, mv([10.0, 10.0, 0.2], 0.0, 21.0)),
        (2, mv([20.0, 10.0, 0.2], 10.0, 50.0)),
        (2, mv([20.0, 10.0, 0.2], f64::NAN, 50.0)),
        (0, mv([10.0, 10.0, 0.6], 0.0, 5000.0)),
        (0, mv([10.0, 10.0, 999.0], 0.0, 500.0)),
        (2, Move { fan: 2.0, ..mv([20.0, 10.0, 0.2], 0.3, 50.0) }),
        (2, Move { accel: 0.0, ..mv([20.0, 10.0, 0.2], 0.3, 50.0) }),
        (2, Move { wait: f64::NAN, ..mv([20.0, 10.0, 0.2], 0.3, 50.0) }),
        (2, Move { wait: -1.0, ..mv([20.0, 10.0, 0.2], 0.3, 50.0) }),
        // From where the layer starts, unknown, only a travel.
        (0, mv([10.0, 10.0, 0.2], 0.3, 50.0)),
    ];
    for (at, m) in cases {
        let mut moves = safe();
        moves[at] = m.clone();
        assert!(!verify(&program(moves), &ENVELOPE).is_empty(), "{m:?} passed");
    }
    let mut layers = program(safe()).layers;
    layers.push(Layer { index: 1, z: -1.0, height: 0.2, warp: Default::default(), moves: safe() });
    assert!(!verify(&Program { layers }, &ENVELOPE).is_empty());
}

/// A layer of one bead from (10, 10) to (20, 10) at `z`, shifted `dy` across it.
fn bead_layer(index: usize, z: f64, dy: f64) -> Layer {
    let moves = vec![
        mv([10.0, 10.0 + dy, z + 0.4], 0.0, 500.0),
        mv([10.0, 10.0 + dy, z], 0.0, 20.0),
        mv([20.0, 10.0 + dy, z], 0.3, 50.0),
        mv([20.0, 10.0 + dy, z + 0.4], 0.0, 20.0),
    ];
    Layer { index, z, height: 0.2, warp: Default::default(), moves }
}

#[test]
fn the_head_never_comes_down_into_what_is_printed() {
    let tower = |dx: f64| (0..10).map(move |k| bead_layer(k, 0.2 * (k + 1) as f64, dx));
    // A tower 2 mm high, then a layer beside it from its foot: 10 mm away the nozzle clears it
    // under the head, 2 mm away its cone does not, and over it the nozzle goes into it.
    let beside = |dy: f64| {
        let mut layers: Vec<Layer> = tower(0.0).collect();
        layers.push(bead_layer(10, 0.2, dy));
        verify(&Program { layers }, &ENVELOPE)
    };
    assert_eq!(beside(40.0), vec![], "past the head");
    assert_eq!(beside(10.0), vec![], "past the cone");
    assert!(!beside(2.0).is_empty(), "under the cone");
    assert!(!beside(0.0).is_empty(), "into the tower");
    // A tower taller than the nozzle reaches below the head: the head meets it.
    let mut tall: Vec<Layer> = (0..25).map(|k| bead_layer(k, 0.2 * (k + 1) as f64, 0.0)).collect();
    tall.push(bead_layer(25, 0.2, 15.0));
    assert!(!verify(&Program { layers: tall }, &ENVELOPE).is_empty(), "under the head");
    // Two towers side by side, each laid up a layer at a time, pass.
    let mut both: Vec<Layer> = Vec::new();
    for (a, b) in tower(0.0).zip(tower(30.0)) {
        both.extend([a, b]);
    }
    assert_eq!(verify(&Program { layers: both }, &ENVELOPE), vec![]);
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

/// Over a flat first layer's bead, a second layer bent up a ramp 0.02 mm per mm from x = 0
/// to 20, its beads at `z(x)`: up the ramp along y = 10, then back down beside it.
fn ramped(z: impl Fn(f64) -> f64) -> Program {
    let warp = Relief::from_fn(Grid { origin: [0.0, 0.0], cell: 1.0 }, [0, 0], [41, 21], |i, _| {
        0.02 * i.min(20) as f64
    });
    let mut moves = vec![mv([10.0, 10.0, 1.2], 0.0, 500.0), mv([10.0, 10.0, z(10.0)], 0.0, 20.0)];
    moves.extend((11..=20).map(|x| mv([x as f64, 10.0, z(x as f64)], 0.03, 50.0)));
    moves.push(mv([20.0, 10.4, z(20.0)], 0.0, 20.0));
    moves.extend((10..20).rev().map(|x| mv([x as f64, 10.4, z(x as f64)], 0.03, 50.0)));
    moves.push(mv([10.0, 10.4, 1.2], 0.0, 20.0));
    let bent = Layer { index: 1, z: 0.4, height: 0.2, warp, moves };
    Program { layers: vec![bead_layer(0, 0.2, 0.0), bent] }
}

#[test]
fn beads_on_a_bent_layer_pass_on_its_top_and_are_caught_off_it() {
    // On its top, laid against each other: they pass.
    assert_eq!(verify(&ramped(|x| 0.4 + 0.02 * x), &ENVELOPE), vec![]);
    // Within a quarter of the layer's thickness of it, as fitted beads may be: they pass.
    assert_eq!(verify(&ramped(|x| 0.4 + 0.02 * x - 0.04), &ENVELOPE), vec![]);
    // Level at the layer's height, 0.2 to 0.4 mm below its top: caught.
    let level = verify(&ramped(|_| 0.4), &ENVELOPE);
    assert!(level.iter().any(|v| v.reason.starts_with("extrusion at z")), "{level:?}");
}
