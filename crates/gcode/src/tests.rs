//! The text of small programs.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_motion::{Bead, Layer};

#[test]
fn numbers_are_short() {
    assert_eq!(number(0.4, 5), "0.4");
    assert_eq!(number(-0.0000001, 3), "0");
    assert_eq!(number(12.0, 3), "12");
    assert_eq!(number(1800.0, 0), "1800");
}

fn mv(to: Option<[f64; 3]>, arc: Option<Arc>, e: f64, speed: f64, bead: Option<Bead>) -> Move {
    Move { to, arc, e, speed, accel: 1000.0, fan: 0.0, bead }
}

fn header() -> Header {
    Header {
        generator: "layercraft test".into(),
        filament_diameter: 1.75,
        filament_density: 1.26,
        settings: vec![("start".into(), "a\nb".into())],
    }
}

const LIMITS: Limits = Limits { max_accel: 1000.0, jerk: 10.0, max_speed_xy: 1000.0, max_speed_z: 20.0 };

fn frame(layers: &[&str]) -> Frame {
    Frame { start: "M83".into(), layers: layers.iter().map(|l| l.to_string()).collect(), end: "M400".into() }
}

/// The executable block without the printer's progress, which depends on the planner.
fn body(text: &str) -> Vec<&str> {
    text.lines().skip_while(|l| *l != "; EXECUTABLE_BLOCK_START").filter(|l| !l.starts_with("M73")).collect()
}

#[test]
fn moves_name_only_what_changes() {
    let wall = Bead { role: Role::OUTER_WALL, width: 0.42 };
    let moves = vec![
        mv(Some([10.0, 20.0, 0.6]), None, 0.0, 500.0, None),
        mv(Some([10.0, 20.0, 0.2]), None, 0.0, 500.0, None),
        mv(None, None, 0.4, 30.0, None),
        mv(Some([15.0, 20.0, 0.2]), None, 0.123456, 50.0, Some(wall)),
        mv(Some([15.0, 25.0, 0.2]), None, 0.1, 50.0, Some(wall)),
    ];
    let program = Program { layers: vec![Layer { index: 0, z: 0.2, height: 0.2, moves }] };
    let text = write(&program, &frame(&["; layer"]), &LIMITS, &header());
    assert_eq!(
        body(&text),
        vec![
            "; EXECUTABLE_BLOCK_START",
            "M83",
            "; CHANGE_LAYER",
            "; Z_HEIGHT: 0.2",
            "; LAYER_HEIGHT: 0.2",
            "; layer",
            "M106 S0",
            "M204 S1000",
            "G1 X10 Y20 Z0.6 F30000",
            "G1 Z0.2",
            "G1 E0.4 F1800",
            "; FEATURE: Outer wall",
            "; LINE_WIDTH: 0.42",
            "G1 X15 E0.12346 F3000",
            "G1 Y25 E0.1",
            "M400",
            "; EXECUTABLE_BLOCK_END",
        ]
    );
    assert!(text.contains("; start = a\\nb\n"));
    assert!(text.contains("\nM73 P0 R1\n") && text.contains("\nM73 P100 R0\n"));
}

#[test]
fn arcs_are_written_about_their_centre_and_moves_of_nothing_vanish() {
    let wall = Bead { role: Role::OUTER_WALL, width: 0.42 };
    let turn = |centre: [f64; 2], turn: f64| Some(Arc { centre, turn });
    let full = std::f64::consts::TAU;
    let first = vec![
        mv(Some([10.0, 20.0, 0.2]), None, 0.0, 500.0, None),
        mv(Some([10.0, 20.0, 0.6]), turn([10.0, 21.0], full), 0.0, 300.0, None),
        mv(Some([10.0, 20.0, 1.0]), turn([10.0, 19.0], full), 0.0, 300.0, None),
        // Less than the written precision in every axis and in filament: nothing, no feature.
        mv(Some([10.0002, 20.0, 1.0]), None, 0.000001, 300.0, Some(wall)),
        // Half a turn clockwise about (11, 20).
        mv(Some([12.0, 20.0, 1.0]), turn([11.0, 20.0], -std::f64::consts::PI), 0.05, 10.0, Some(wall)),
    ];
    let second = vec![
        mv(Some([10.0, 20.0, 1.0]), None, 0.0, 500.0, None),
        mv(Some([10.0, 20.0, 1.4]), turn([9.0, 20.0], full), 0.0, 300.0, None),
    ];
    let program = Program {
        layers: vec![
            Layer { index: 0, z: 0.2, height: 0.2, moves: first },
            Layer { index: 1, z: 0.4, height: 0.2, moves: second },
        ],
    };
    let text = write(&program, &frame(&["; one", "; two"]), &LIMITS, &header());
    let lines = body(&text);
    let after = |marker: &str| -> Vec<&str> {
        lines
            .iter()
            .skip_while(|l| **l != marker)
            .skip(1)
            .take_while(|l| !l.starts_with(';'))
            .copied()
            .collect()
    };
    assert_eq!(
        after("; one"),
        vec![
            "M106 S0",
            "M204 S1000",
            "G1 X10 Y20 Z0.2 F30000",
            "G17",
            "G3 Z0.6 I0 J1 P1 F18000",
            "G3 Z1 I0 J-1 P1"
        ]
    );
    assert!(lines.contains(&"G2 X12 Y20 I1 J0 E0.05 F600"), "{lines:?}");
    assert_eq!(lines.iter().filter(|l| l.starts_with("; FEATURE")).count(), 1);
    // The machine's own G-code may have changed the plane and the state: named again.
    assert_eq!(
        after("; two"),
        vec!["M106 S0", "M204 S1000", "G1 X10 Y20 Z1 F30000", "G17", "G3 Z1.4 I-1 J0 P1 F18000", "M400"]
    );
}

#[test]
fn what_is_written_reads_back() {
    let wall = Bead { role: Role::OUTER_WALL, width: 0.42 };
    let top = Bead { role: Role::TOP_SURFACE, width: 0.4 };
    let tau = std::f64::consts::TAU;
    let first = vec![
        mv(None, None, -0.4, 30.0, None),
        mv(Some([10.0, 20.0, 0.6]), None, 0.0, 500.0, None),
        mv(Some([10.0, 20.0, 0.2]), None, 0.0, 500.0, None),
        mv(None, None, 0.4, 30.0, None),
        mv(Some([15.0, 20.0, 0.2]), None, 0.12346, 50.0, Some(wall)),
        mv(
            Some([17.0, 20.0, 0.2]),
            Some(Arc { centre: [16.0, 20.0], turn: -std::f64::consts::PI }),
            0.05,
            20.0,
            Some(top),
        ),
        // Leaving: a wipe back, a pull, a helix up.
        mv(Some([16.5, 20.0, 0.2]), None, -0.3, 40.0, None),
        mv(None, None, -0.1, 30.0, None),
        mv(Some([16.5, 20.0, 0.6]), Some(Arc { centre: [16.5, 21.0], turn: tau }), 0.0, 300.0, None),
    ];
    let second = vec![
        mv(Some([30.0, 30.0, 0.8]), None, 0.0, 500.0, None),
        mv(Some([30.0, 30.0, 0.4]), None, 0.0, 500.0, None),
        mv(None, None, 0.4, 30.0, None),
        mv(Some([35.0, 30.0, 0.4]), None, 0.1, 50.0, Some(wall)),
    ];
    let program = Program {
        layers: vec![
            Layer { index: 0, z: 0.2, height: 0.2, moves: first },
            Layer { index: 1, z: 0.4, height: 0.2, moves: second },
        ],
    };
    let text = write(&program, &frame(&["M991 S0 P0", "M991 S0 P1"]), &LIMITS, &header());
    let back = read(&text).unwrap();
    assert_eq!(back.layers.len(), 2);
    for (a, b) in program.layers.iter().zip(&back.layers) {
        assert_eq!((a.z, a.height), (b.z, b.height));
        assert_eq!(a.moves.len(), b.moves.len(), "{:#?}", b.moves);
        for (m, n) in a.moves.iter().zip(&b.moves) {
            let close = |x: f64, y: f64, eps: f64| (x - y).abs() <= eps;
            assert_eq!(m.to.is_some(), n.to.is_some());
            if let (Some(p), Some(q)) = (m.to, n.to) {
                assert!(p.iter().zip(q).all(|(x, y)| close(*x, y, 1e-3)), "{m:?} read as {n:?}");
            }
            assert!(
                close(m.e, n.e, 1e-5) && close(m.speed, n.speed, 1.0 / 60.0) && m.bead == n.bead,
                "{m:?} read as {n:?}"
            );
            assert_eq!(m.arc.is_some(), n.arc.is_some());
            if let (Some(p), Some(q)) = (m.arc, n.arc) {
                assert!(
                    close(p.turn, q.turn, 1e-9) && close(p.centre[0], q.centre[0], 1e-3),
                    "{p:?} read as {q:?}"
                );
            }
        }
    }
}

#[test]
fn unknown_commands_and_features_are_errors() {
    let body = |line: &str| {
        format!(
            "; CHANGE_LAYER\n; Z_HEIGHT: 0.2\n; LAYER_HEIGHT: 0.2\nM204 S500\nG1 X1 Y1 Z0.2 F600\n{line}\n"
        )
    };
    assert!(read(&body("M999")).unwrap_err().reason.contains("M999"));
    let err = read(&body("; FEATURE: Ironing\n; LINE_WIDTH: 0.4\nG1 X2 E0.1")).unwrap_err();
    assert!(err.reason.contains("Ironing"), "{err}");
    // In the machine's own G-code, before the first layer, anything goes.
    assert!(read(&format!("M999\nG28\n{}", body("G1 X2 Y2"))).is_ok());
}

#[test]
fn filament_too_fine_to_write_is_carried_not_lost() {
    let wall = Bead { role: Role::OUTER_WALL, width: 0.42 };
    // A thousand beads 0.0003 mm long, each pushing less than the fifth decimal.
    let mut moves = vec![mv(Some([10.0, 20.0, 0.2]), None, 0.0, 500.0, None)];
    moves.extend(
        (1..=1000)
            .map(|i| mv(Some([10.0 + 0.0003 * i as f64, 20.0, 0.2]), None, 0.0000123, 50.0, Some(wall))),
    );
    let program = Program { layers: vec![Layer { index: 0, z: 0.2, height: 0.2, moves }] };
    let text = write(&program, &frame(&["; layer"]), &LIMITS, &header());
    let written: f64 = text
        .lines()
        .filter_map(|l| l.split_whitespace().find_map(|w| w.strip_prefix('E'))?.parse::<f64>().ok())
        .sum();
    assert!((written - 0.0123).abs() < 1e-5, "{written}");
    // A line per written step of position, about one in three or four beads, never a push in place.
    let lines = text.lines().filter(|l| l.starts_with("G1 X")).count();
    assert!((250..=400).contains(&lines), "{lines}");
    assert!(!text.lines().any(|l| l.starts_with("G1 E")));
}

#[test]
fn a_layer_starts_at_its_first_travel_and_its_prelude_leaves_the_one_below() {
    // Bambu Studio's way: the marker, then the wipe and pull of the layer below, its frame, a
    // timelapse block, a spiral lift, and only then the travel of the new layer.
    let text = "\
M204 S10000
G1 X0 Y0 Z5 F600
; CHANGE_LAYER
; Z_HEIGHT: 0.2
; LAYER_HEIGHT: 0.2
G1 E-.4 F1800
M73 L1
G1 X10 Y10 F60000
G1 Z.2
G1 E.4 F1800
; FEATURE: Outer wall
; LINE_WIDTH: 0.42
G1 X20 Y10 E.5 F3000
; CHANGE_LAYER
; Z_HEIGHT: 0.4
; LAYER_HEIGHT: 0.2
; WIPE_START
G1 X19 Y10 E-.38 F15000
; WIPE_END
G1 E-.02 F1800
M991 S0 P1 ;notify layer change
M622.1 S1
M622 J1
    G1 Z9 F1200
M623
G17
G3 Z.6 I0 J1 P1 F60000
G1 X10 Y20 Z.6
G1 Z.4
G1 E.4 F1800
G1 X20 Y20 E.5 F3000
; MACHINE_END_GCODE_START
G1 Z50
";
    let program = read(text).unwrap();
    let [first, second] = program.layers.as_slice() else { panic!("{program:?}") };
    // The start's move is the frame's, not followed; the first layer starts with its pull in
    // place, then its travel, whose height it never named takes the next one it is given.
    assert!(first.moves[0].to.is_none() && first.moves[0].e < 0.0);
    assert_eq!(first.moves[1].to, Some([10.0, 10.0, 0.2]));
    // The layer below ends with the wipe, the pull and the spiral lift, at its own height.
    let tail: Vec<&Move> = first.moves.iter().rev().take(3).collect();
    assert!(
        tail[0].arc.is_some_and(|a| a.turn == std::f64::consts::TAU) && tail[0].to == Some([19.0, 10.0, 0.6])
    );
    assert!(tail[1].to.is_none() && (tail[1].e + 0.02).abs() < 1e-12);
    assert_eq!(tail[2].to, Some([19.0, 10.0, 0.2]));
    // The new layer starts at its travel; the conditional block's move is not there.
    assert_eq!(second.moves[0].to, Some([10.0, 20.0, 0.6]));
    assert!(program.layers.iter().flat_map(|l| &l.moves).all(|m| m.to.is_none_or(|p| p[2] < 9.0)));
    assert_eq!(second.moves.last().and_then(|m| m.bead).map(|b| b.role), Some(Role::OUTER_WALL));
}
