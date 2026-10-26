//! The text of small programs.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_motion::Summary;

#[test]
fn numbers_are_short() {
    assert_eq!(number(0.4, 5), "0.4");
    assert_eq!(number(-0.0000001, 3), "0");
    assert_eq!(number(12.0, 3), "12");
    assert_eq!(number(1800.0, 0), "1800");
}

#[test]
fn moves_name_only_what_changes() {
    let ops = vec![
        Op::Raw { text: "M83".into() },
        Op::Layer { index: 0, z: 0.2, height: 0.2 },
        Op::Lift { z: 0.6, around: None, speed: 20.0 },
        Op::Travel { to: [10.0, 20.0, 0.6], speed: 500.0 },
        Op::Travel { to: [10.0, 20.0, 0.2], speed: 500.0 },
        Op::Retract { e: 0.4, speed: 30.0 },
        Op::Feature { role: Role::OuterWall, width: 0.42 },
        Op::Extrude { to: [15.0, 20.0, 0.2], e: 0.123456, speed: 50.0 },
        Op::Extrude { to: [15.0, 25.0, 0.2], e: 0.1, speed: 50.0 },
    ];
    let program = Program { ops, summary: Summary { layers: 1, ..Summary::default() } };
    let header = Header {
        generator: "layercraft test".into(),
        filament_diameter: 1.75,
        filament_density: 1.26,
        settings: vec![("start".into(), "a\nb".into())],
    };
    let text = write(&program, &header);
    let body: Vec<&str> = text.lines().skip_while(|l| *l != "; EXECUTABLE_BLOCK_START").collect();
    assert_eq!(
        body,
        vec![
            "; EXECUTABLE_BLOCK_START",
            "M83",
            "; CHANGE_LAYER",
            "; Z_HEIGHT: 0.2",
            "; LAYER_HEIGHT: 0.2",
            "G1 Z0.6 F1200",
            "G1 X10 Y20 F30000",
            "G1 Z0.2",
            "G1 E0.4 F1800",
            "; FEATURE: Outer wall",
            "; LINE_WIDTH: 0.42",
            "G1 X15 E0.12346 F3000",
            "G1 Y25 E0.1",
            "M73 P100 R0",
            "; EXECUTABLE_BLOCK_END",
        ]
    );
    assert!(text.contains("; start = a\\nb\n"));
}

#[test]
fn helical_lifts_are_full_arcs_and_moves_of_nothing_vanish() {
    let ops = vec![
        Op::Raw { text: "M83".into() },
        Op::Travel { to: [10.0, 20.0, 0.2], speed: 500.0 },
        Op::Lift { z: 0.6, around: Some([10.0, 21.0]), speed: 300.0 },
        Op::Lift { z: 1.0, around: Some([10.0, 19.0]), speed: 300.0 },
        // Less than the written precision in every axis and in filament.
        Op::Extrude { to: [10.0002, 20.0, 1.0], e: 0.000001, speed: 300.0 },
        Op::Raw { text: "M400".into() },
        Op::Travel { to: [10.0, 20.0, 1.0], speed: 500.0 },
        Op::Lift { z: 1.4, around: Some([9.0, 20.0]), speed: 300.0 },
    ];
    let program = Program { ops, summary: Summary::default() };
    let header =
        Header { generator: "t".into(), filament_diameter: 1.75, filament_density: 1.26, settings: vec![] };
    let text = write(&program, &header);
    let body: Vec<&str> =
        text.lines().skip_while(|l| *l != "M83").take_while(|l| !l.starts_with("M73")).collect();
    assert_eq!(
        body,
        vec![
            "M83",
            "G1 X10 Y20 Z0.2 F30000",
            "G17",
            "G3 Z0.6 I0 J1 P1 F18000",
            "G3 Z1 I0 J-1 P1",
            "M400",
            "G1 X10 Y20 Z1 F30000",
            // The machine's own G-code may have changed the plane: named again.
            "G17",
            "G3 Z1.4 I-1 J0 P1 F18000",
        ]
    );
}
