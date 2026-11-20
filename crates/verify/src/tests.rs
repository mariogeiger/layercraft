//! Each unsafe move is caught; a safe program passes, on flat layers and on bent ones.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_geom::Relief;
use layercraft_motion::{Arc, Bead, Filament, Pass};
use layercraft_path::Role;

const ENVELOPE: Envelope = Envelope {
    area: [[0.0, 0.0], [256.0, 256.0]],
    max_z: 250.0,
    max_speed: 1000.0,
    max_speed_z: 20.0,
    toolhead: Toolhead {
        face_radius: 0.2,
        flank_slope: 1.0,
        column_radius: 1.5,
        protrusion: 4.0,
        margin: 1.0,
    },
    ironing: 0.2,
    resolution: 0.012,
};

/// The one filament, melted at 20 mm³/s.
const FILAMENTS: [Filament; 1] = [Filament { diameter: 1.75, max_volumetric_speed: 20.0 }];

fn mv(to: [f64; 3], e: f64, speed: f64) -> Move {
    let bead = (e > 0.0).then_some(Bead { role: Role::OUTER_WALL, width: 0.42 });
    Move { to: Some(to), arc: None, e, speed, accel: 1000.0, fan: 0.0, bead, wait: 0.0 }
}

fn arc(to: [f64; 3], centre: [f64; 2], turn: f64) -> Move {
    Move { arc: Some(Arc { centre, turn }), ..mv(to, 0.0, 300.0) }
}

fn program(moves: Vec<Move>) -> Program {
    Program {
        carrier: Default::default(),
        passes: vec![Pass {
            index: 0,
            z: 0.2,
            height: 0.2,
            warp: Default::default(),
            free: false,
            moves,
            material: 0,
        }],
    }
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
    assert_eq!(verify(&program(safe()), &ENVELOPE, &FILAMENTS), vec![]);
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
        assert!(!verify(&program(moves), &ENVELOPE, &FILAMENTS).is_empty(), "{m:?} passed");
    }
    let mut passes = program(safe()).passes;
    passes.push(Pass {
        index: 1,
        z: -1.0,
        height: 0.2,
        warp: Default::default(),
        free: false,
        moves: safe(),
        material: 0,
    });
    assert!(!verify(&Program { carrier: Default::default(), passes }, &ENVELOPE, &FILAMENTS).is_empty());
}

/// A layer of one bead from (10, 10) to (20, 10) at `z`, shifted `dy` across it.
fn bead_layer(index: usize, z: f64, dy: f64) -> Pass {
    let moves = vec![
        mv([10.0, 10.0 + dy, z + 0.4], 0.0, 500.0),
        mv([10.0, 10.0 + dy, z], 0.0, 20.0),
        mv([20.0, 10.0 + dy, z], 0.3, 50.0),
        mv([20.0, 10.0 + dy, z + 0.4], 0.0, 20.0),
    ];
    Pass { index, z, height: 0.2, warp: Default::default(), free: false, moves, material: 0 }
}

#[test]
fn the_head_never_comes_down_into_what_is_printed() {
    let tower = |dx: f64| (0..10).map(move |k| bead_layer(k, 0.2 * (k + 1) as f64, dx));
    // A tower 2 mm high, then a layer beside it from its foot: 3 mm away, the nozzle's column
    // passes it and the tower stands under the head's plane; 1 mm away the nozzle's flank meets
    // it, and over it the nozzle goes into it.
    let beside = |dy: f64| {
        let mut passes: Vec<Pass> = tower(0.0).collect();
        passes.push(bead_layer(10, 0.2, dy));
        verify(&Program { carrier: Default::default(), passes }, &ENVELOPE, &FILAMENTS)
    };
    assert_eq!(beside(40.0), vec![], "far off");
    assert_eq!(beside(3.0), vec![], "past the column");
    assert!(!beside(1.0).is_empty(), "under the flank");
    assert!(!beside(0.0).is_empty(), "into the tower");
    // A tower taller than the head's clearance reaches its plane, however far: the head meets it.
    let mut tall: Vec<Pass> = (0..25).map(|k| bead_layer(k, 0.2 * (k + 1) as f64, 0.0)).collect();
    tall.push(bead_layer(25, 0.2, 60.0));
    assert!(
        !verify(&Program { carrier: Default::default(), passes: tall }, &ENVELOPE, &FILAMENTS).is_empty(),
        "under the head"
    );
    // Two towers side by side, each laid up a layer at a time, pass.
    let mut both: Vec<Pass> = Vec::new();
    for (a, b) in tower(0.0).zip(tower(30.0)) {
        both.extend([a, b]);
    }
    assert_eq!(verify(&Program { carrier: Default::default(), passes: both }, &ENVELOPE, &FILAMENTS), vec![]);
}

#[test]
fn an_arc_turns_inside_the_plate_from_a_known_place_and_ends_on_its_circle() {
    let full = std::f64::consts::TAU;
    let check = |extra: Vec<Move>| {
        let mut moves = safe();
        moves.extend(extra);
        verify(&program(moves), &ENVELOPE, &FILAMENTS)
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
    assert!(!verify(&program(first), &ENVELOPE, &FILAMENTS).is_empty());
    // Never below the layer.
    assert!(!check(vec![mv([20.0, 10.0, 0.1], 0.0, 20.0)]).is_empty());
}

/// Over a flat first layer's bead, a second layer bent up a ramp 0.02 mm per mm from x = 0
/// to 20, its beads at `z(x)`: up the ramp along y = 10, then back down beside it.
fn ramped(z: impl Fn(f64) -> f64) -> Program {
    let values: Vec<f64> = (0..41 * 21).map(|k| 0.02 * (k % 41).min(20) as f64).collect();
    let ramp = Relief::from_grid([0.0, 0.0], 1.0, [0, 0], [41, 21], &values).unwrap();
    let mut moves = vec![mv([10.0, 10.0, 1.2], 0.0, 500.0), mv([10.0, 10.0, z(10.0)], 0.0, 20.0)];
    moves.extend((11..=20).map(|x| mv([x as f64, 10.0, z(x as f64)], 0.03, 50.0)));
    moves.push(mv([20.0, 10.4, z(20.0)], 0.0, 20.0));
    moves.extend((10..20).rev().map(|x| mv([x as f64, 10.4, z(x as f64)], 0.03, 50.0)));
    moves.push(mv([10.0, 10.4, 1.2], 0.0, 20.0));
    let warp = ramp.warp().clone();
    let bent = Pass { index: 1, z: 0.4, height: 0.2, warp, free: false, moves, material: 0 };
    Program { carrier: ramp.carrier().clone(), passes: vec![bead_layer(0, 0.2, 0.0), bent] }
}

#[test]
fn beads_on_a_bent_layer_pass_on_its_top_and_are_caught_off_it() {
    // On its top, laid against each other: they pass.
    assert_eq!(verify(&ramped(|x| 0.4 + 0.02 * x), &ENVELOPE, &FILAMENTS), vec![]);
    // Within a quarter of the layer's thickness of it, as fitted beads may be: they pass.
    assert_eq!(verify(&ramped(|x| 0.4 + 0.02 * x - 0.04), &ENVELOPE, &FILAMENTS), vec![]);
    // Level at the layer's height, 0.2 to 0.4 mm below its top: caught.
    let level = verify(&ramped(|_| 0.4), &ENVELOPE, &FILAMENTS);
    assert!(level.iter().any(|v| v.reason.starts_with("extrusion at z")), "{level:?}");
}

/// Where the nozzle comes down beside a bead: in the bead's own pass, in the next pass of its
/// layer (another material's), or on the next layer.
#[derive(Clone, Copy)]
enum After {
    OwnPass,
    NextPass,
    NextLayer,
}

/// A bead along y = 10 at z = 0.6 over a first layer far off; then, `after` it, the nozzle comes
/// straight down beside it at `(15, y)` to `z`.
fn beside_bead(y: f64, z: f64, after: After) -> Program {
    let bead = vec![
        mv([10.0, 10.0, 1.0], 0.0, 500.0),
        mv([10.0, 10.0, 0.6], 0.0, 20.0),
        mv([20.0, 10.0, 0.6], 0.3, 50.0),
    ];
    let down = vec![mv([15.0, y, 1.0], 0.0, 200.0), mv([15.0, y, z], 0.0, 20.0)];
    let pass = |index: usize, z: f64, height: f64, material: usize, moves: Vec<Move>| Pass {
        index,
        z,
        height,
        warp: Default::default(),
        free: false,
        moves,
        material,
    };
    let then = match after {
        After::OwnPass => vec![pass(1, 0.6, 0.4, 0, [bead, down].concat())],
        After::NextPass => vec![pass(1, 0.6, 0.4, 0, bead), pass(1, 0.6, 0.4, 1, down)],
        After::NextLayer => vec![pass(1, 0.6, 0.4, 0, bead), pass(2, 0.8, 0.2, 0, down)],
    };
    let passes = [vec![bead_layer(0, 0.2, 60.0)], then].concat();
    Program { carrier: Default::default(), passes }
}

#[test]
fn the_face_irons_its_own_layer_as_allowed_and_no_layer_before() {
    // The bead's flat top reaches 0.21 mm from its middle; 0.15 mm past its edge, the tip is
    // within the face of 0.2 mm radius, which meets the top's edge as the tip comes lower.
    let y = 10.0 + 0.21 + 0.15;
    // A layer later: within the rounding of written positions, no deeper.
    assert_eq!(verify(&beside_bead(y, 0.599, After::NextLayer), &ENVELOPE, &FILAMENTS), vec![]);
    let into = verify(&beside_bead(y, 0.57, After::NextLayer), &ENVELOPE, &FILAMENTS);
    assert!(into.iter().any(|v| v.reason.starts_with("the head comes 0.028")), "{into:?}");
    // Past the face's rim, the flank rises faster than the tip sank.
    assert_eq!(verify(&beside_bead(y + 0.1, 0.57, After::NextLayer), &ENVELOPE, &FILAMENTS), vec![]);
    // On its own layer, ironed 0.1 mm of the 0.2 allowed; then 0.25: in the bead's own pass and,
    // the layer's own as well, in the pass of another material after it.
    let two = [FILAMENTS[0], FILAMENTS[0]];
    for after in [After::OwnPass, After::NextPass] {
        assert_eq!(verify(&beside_bead(y, 0.5, after), &ENVELOPE, &two), vec![]);
        let ironed = verify(&beside_bead(y, 0.35, after), &ENVELOPE, &two);
        assert!(
            ironed.iter().any(|v| v.reason.starts_with("the nozzle irons its own layer 0.050")),
            "{ironed:?}"
        );
    }
    // A violation names the layer by its index, and the pass by its place in the program.
    let ironed = verify(&beside_bead(y, 0.35, After::NextPass), &ENVELOPE, &two);
    assert!(ironed.iter().all(|v| (v.layer, v.pass) == (1, 2)), "{ironed:?}");
}

#[test]
fn each_pass_flows_within_its_own_filament_s_melt_limit() {
    // 0.3 mm of 1.75 mm filament over 10 mm at 300 mm/s asks 21.6 mm³/s; one layer, two passes
    // of two materials, side by side.
    let fast = |material: usize| {
        let mut moves = safe();
        for m in &mut moves {
            m.to = m.to.map(|[x, y, z]| [x, y + 20.0 * material as f64, z]);
        }
        moves[2].speed = 300.0;
        Pass { material, ..program(moves).passes[0].clone() }
    };
    let program = Program { carrier: Default::default(), passes: vec![fast(0), fast(1)] };
    let hot = Filament { max_volumetric_speed: 30.0, ..FILAMENTS[0] };
    // The second filament melts it; the first does not.
    let found = verify(&program, &ENVELOPE, &[FILAMENTS[0], hot]);
    assert!(found.iter().all(|v| v.pass == 0 && v.reason.contains("past the melt limit 20")), "{found:?}");
    assert!(!found.is_empty());
    assert_eq!(verify(&program, &ENVELOPE, &[hot, hot]), vec![]);
    // A pass of a material no filament is given for is never sent.
    let found = verify(&program, &ENVELOPE, &[hot]);
    assert!(found.iter().any(|v| v.pass == 1 && v.reason.starts_with("no filament")), "{found:?}");
}

/// A bead along y = 10 on the plate, then a free strand across it along x = 15, from y = 5 to 15,
/// at tip height `z` at its end and `z_start` at its start.
fn strand_across(z_start: f64, z: f64) -> Program {
    let strand = vec![
        mv([15.0, 5.0, 1.0], 0.0, 500.0),
        mv([15.0, 5.0, z_start], 0.0, 20.0),
        mv([15.0, 15.0, z], 0.3, 50.0),
    ];
    let free = Pass {
        index: 1,
        z: 0.4,
        height: 0.2,
        warp: Default::default(),
        free: true,
        moves: strand,
        material: 0,
    };
    Program { carrier: Default::default(), passes: vec![bead_layer(0, 0.2, 0.0), free] }
}

#[test]
fn a_free_bead_rests_on_the_plate_or_on_a_bead_before_it() {
    // Over the bead, one height above its top; on the plate, two: stretched over the gap.
    assert_eq!(verify(&strand_across(0.4, 0.4), &ENVELOPE, &FILAMENTS), vec![]);
    // On the plate at one height, short of the bead.
    let mut short = strand_across(0.2, 0.2);
    short.passes[1].moves[2].to = Some([15.0, 8.0, 0.2]);
    assert_eq!(verify(&short, &ENVELOPE, &FILAMENTS), vec![]);
}

#[test]
fn a_free_bead_is_caught_inside_plastic_or_hanging_too_high() {
    // Into the bead it crosses: at its middle the tip is 0.1 mm over its top.
    let inside = verify(&strand_across(0.4, 0.2), &ENVELOPE, &FILAMENTS);
    assert!(inside.iter().any(|v| v.reason.starts_with("free extrusion at z = 0.3")), "{inside:?}");
    // Past two heights over the plate at its end.
    let hanging = verify(&strand_across(0.4, 0.5), &ENVELOPE, &FILAMENTS);
    assert!(hanging.iter().any(|v| v.reason.starts_with("free extrusion at z = 0.5")), "{hanging:?}");
}
