//! Each unsafe instruction is caught; a safe program passes.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_motion::Summary;

const ENVELOPE: Envelope = Envelope {
    area: [[0.0, 0.0], [256.0, 256.0]],
    max_z: 250.0,
    max_speed: 1000.0,
    filament_diameter: 1.75,
    max_volumetric_speed: 20.0,
};

fn program(ops: Vec<Op>) -> Program {
    Program { ops, summary: Summary::default() }
}

fn safe() -> Vec<Op> {
    vec![
        Op::Layer { index: 0, z: 0.2, height: 0.2 },
        Op::Lift { z: 0.6, speed: 20.0 },
        Op::Travel { to: [10.0, 10.0, 0.6], speed: 500.0 },
        Op::Travel { to: [10.0, 10.0, 0.2], speed: 500.0 },
        Op::Extrude { to: [20.0, 10.0, 0.2], e: 0.3, speed: 50.0 },
    ]
}

#[test]
fn a_safe_program_passes() {
    assert_eq!(verify(&program(safe()), &ENVELOPE), vec![]);
}

#[test]
fn each_danger_is_caught() {
    let cases: Vec<(usize, Op)> = vec![
        (4, Op::Extrude { to: [300.0, 10.0, 0.2], e: 0.3, speed: 50.0 }),
        (4, Op::Extrude { to: [20.0, 10.0, 0.1], e: 0.3, speed: 50.0 }),
        (3, Op::Travel { to: [10.0, 10.0, 0.0], speed: 500.0 }),
        (4, Op::Extrude { to: [20.0, 10.0, 0.2], e: 10.0, speed: 50.0 }),
        (4, Op::Extrude { to: [20.0, 10.0, 0.2], e: f64::NAN, speed: 50.0 }),
        (2, Op::Travel { to: [10.0, 10.0, 0.6], speed: 5000.0 }),
        (1, Op::Lift { z: 999.0, speed: 20.0 }),
        (0, Op::Layer { index: 0, z: -1.0, height: 0.2 }),
    ];
    for (at, op) in cases {
        let mut ops = safe();
        ops[at] = op.clone();
        assert!(!verify(&program(ops), &ENVELOPE).is_empty(), "{op:?} passed");
    }
    let mut falling = safe();
    falling.push(Op::Layer { index: 1, z: 0.2, height: 0.2 });
    assert!(!verify(&program(falling), &ENVELOPE).is_empty());
    let mut fan = safe();
    fan.push(Op::Fan { speed: 2.0 });
    assert!(!verify(&program(fan), &ENVELOPE).is_empty());
}
