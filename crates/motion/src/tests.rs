//! Filament by volume, times against closed forms, and the order of a travel.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_path::{Path, PathLayer, Paths};

fn role(speed: f64) -> RoleMotion {
    RoleMotion { speed, accel: 1000.0 }
}

fn params() -> MotionParams {
    MotionParams {
        filament_diameter: 1.75,
        flow_ratio: 0.98,
        max_volumetric_speed: 1000.0,
        outer_wall: role(100.0),
        inner_wall: role(100.0),
        top_surface: role(100.0),
        solid: role(100.0),
        sparse: role(100.0),
        bridge: role(50.0),
        first_layer_wall_speed: 50.0,
        first_layer_infill_speed: 50.0,
        first_layer_accel: 500.0,
        travel: role(300.0),
        retract_length: 0.4,
        retract_speed: 30.0,
        deretract_speed: 30.0,
        retract_min_travel: 1.0,
        seam_gap: 0.0,
        wipe_distance: 0.0,
        retract_before_wipe: 0.0,
        z_hop: 0.4,
        slow_down_layer_time: 0.0,
        slow_down_min_speed: 10.0,
        fan_speed: 1.0,
        fan_off_layers: 1,
        limits: Limits { max_accel: 1000.0, jerk: 0.0, max_speed_xy: 1000.0, max_speed_z: 20.0 },
        templates: Templates { start: "; start [who]\n".into(), ..Templates::default() },
    }
}

fn line(len: f64) -> Paths {
    let path = Path {
        role: Role::Sparse,
        width: 0.4,
        closed: false,
        points: vec![[10.0, 10.0], [10.0 + len, 10.0]],
        travel_inside: false,
    };
    Paths {
        layers: vec![
            PathLayer { index: 0, top: 0.2, height: 0.2, paths: vec![path.clone()] },
            PathLayer { index: 1, top: 0.4, height: 0.2, paths: vec![path] },
        ],
    }
}

fn scope() -> layercraft_machine::Scope {
    [("who".to_string(), layercraft_machine::Value::Str("me".into()))].into_iter().collect()
}

#[test]
fn filament_follows_the_bead_volume() {
    let program = compile(&line(10.0), &params(), &scope()).unwrap();
    let area = (0.4 - 0.2) * 0.2 + std::f64::consts::PI * 0.01;
    let filament_area = std::f64::consts::PI * 0.875 * 0.875;
    let expected = 2.0 * 0.98 * area * 10.0 / filament_area;
    assert!((program.summary.filament_mm - expected).abs() < 1e-12);
    assert!((program.summary.volume_per_layer[0] - area * 10.0).abs() < 1e-12);
    assert!(matches!(&program.ops[0], Op::Raw { text } if text.starts_with("; start me\n")));
}

#[test]
fn a_lone_move_takes_its_trapezoid() {
    let limits = Limits { max_accel: 1000.0, jerk: 0.0, max_speed_xy: 1000.0, max_speed_z: 20.0 };
    // 100 mm at 100 mm/s with 1000 mm/s²: 0.1 s up and 0.1 s down over 5 mm each, 90 mm cruising.
    let ops = vec![Op::Travel { to: [100.0, 0.0, 0.0], speed: 100.0 }];
    let t = estimate(&ops, &limits, [0.0; 3]);
    assert!((t[1] - 1.1).abs() < 1e-9, "{t:?}");
    // 1 mm never reaches cruise: a triangle of peak √(a·L) = 31.6 mm/s.
    let ops = vec![Op::Travel { to: [1.0, 0.0, 0.0], speed: 100.0 }];
    let t = estimate(&ops, &limits, [0.0; 3]);
    assert!((t[1] - 2.0 * (1.0f64 / 1000.0).sqrt()).abs() < 1e-9);
}

#[test]
fn straight_runs_keep_their_speed_through_joints() {
    let limits = Limits { max_accel: 1000.0, jerk: 0.0, max_speed_xy: 1000.0, max_speed_z: 20.0 };
    let split: Vec<Op> =
        (1..=10).map(|i| Op::Travel { to: [10.0 * i as f64, 0.0, 0.0], speed: 100.0 }).collect();
    let t = estimate(&split, &limits, [0.0; 3]);
    assert!((t.last().unwrap() - 1.1).abs() < 1e-9);
}

#[test]
fn travels_lift_straight_up_then_cross_then_land() {
    let program = compile(&line(10.0), &params(), &scope()).unwrap();
    let moves: Vec<&Op> = program
        .ops
        .iter()
        .filter(|op| matches!(op, Op::Lift { .. } | Op::Travel { .. } | Op::Retract { .. }))
        .take(5)
        .collect();
    assert!(matches!(moves[0], Op::Retract { e, .. } if *e < 0.0));
    assert!(matches!(moves[1], Op::Lift { z, .. } if (*z - 0.6).abs() < 1e-9));
    assert!(matches!(moves[2], Op::Travel { to, .. } if (to[2] - 0.6).abs() < 1e-9 && to[0] == 10.0));
    assert!(matches!(moves[3], Op::Travel { to, .. } if (to[2] - 0.2).abs() < 1e-12));
    assert!(matches!(moves[4], Op::Retract { e, .. } if *e > 0.0));
}

#[test]
fn short_layers_slow_down_to_cool() {
    let mut p = params();
    p.slow_down_layer_time = 4.0;
    let program = compile(&line(10.0), &p, &scope()).unwrap();
    let speeds: Vec<f64> = program
        .ops
        .iter()
        .filter_map(|op| if let Op::Extrude { speed, .. } = op { Some(*speed) } else { None })
        .collect();
    // 10 mm at 100 mm/s is 0.1 s: slowed to 2.5 mm/s, held at the 10 mm/s floor.
    assert_eq!(speeds[1], 10.0);
}

#[test]
fn missing_template_variables_are_named() {
    let mut p = params();
    p.templates.end = "M140 S[bed]".into();
    let err = compile(&line(1.0), &p, &scope()).unwrap_err().to_string();
    assert!(err.contains("bed"), "{err}");
}

#[test]
fn retraction_wipes_back_over_the_bead() {
    let mut p = params();
    p.wipe_distance = 1.0;
    let program = compile(&line(10.0), &p, &scope()).unwrap();
    let ops = &program.ops;
    let first_wipe = ops.iter().position(|op| matches!(op, Op::Wipe { .. })).unwrap();
    let wipes: Vec<(&[f64; 3], f64, f64)> = ops[first_wipe..]
        .iter()
        .map_while(|op| if let Op::Wipe { to, e, speed } = op { Some((to, *e, *speed)) } else { None })
        .collect();
    // Back from the line's end at x = 20 towards its start, 1 mm, pulling the whole 0.4 mm.
    let last = wipes.last().unwrap();
    assert!((last.0[0] - 19.0).abs() < 1e-9, "{wipes:?}");
    let pulled: f64 = wipes.iter().map(|w| w.1).sum();
    assert!((pulled + 0.4).abs() < 1e-12);
    // The filament never moves faster than the retraction speed: 0.4 mm over 1 mm at v ≤ 75 mm/s.
    assert!(wipes.iter().all(|w| w.2 * 0.4 / 1.0 <= p.retract_speed + 1e-9));
    // And the print ends lifted off the part before the machine's own end.
    let end = ops.iter().rposition(|op| matches!(op, Op::Raw { .. })).unwrap();
    assert!(matches!(ops[end - 1], Op::Lift { z, .. } if (z - 0.8).abs() < 1e-9), "{:?}", ops[end - 1]);
}

#[test]
fn loops_stop_short_of_their_seam_and_bridges_lay_round_strands() {
    let square = Path {
        role: Role::OuterWall,
        width: 0.4,
        closed: true,
        points: vec![[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]],
        travel_inside: false,
    };
    let bridge = Path {
        role: Role::Bridge,
        width: 0.4,
        closed: false,
        points: vec![[10.0, 10.0], [20.0, 10.0]],
        travel_inside: false,
    };
    let paths = Paths {
        layers: vec![
            PathLayer { index: 0, top: 0.2, height: 0.2, paths: vec![square.clone()] },
            PathLayer { index: 1, top: 0.4, height: 0.2, paths: vec![square, bridge] },
        ],
    };
    let mut p = params();
    p.seam_gap = 0.06;
    let program = compile(&paths, &p, &scope()).unwrap();
    let extrusions: Vec<[f64; 3]> = program
        .ops
        .iter()
        .filter_map(|op| if let Op::Extrude { to, .. } = op { Some(*to) } else { None })
        .collect();
    // The loop's last move ends 0.06 mm before its start, on the closing side.
    assert!(
        (extrusions[3][0] - 10.0).abs() < 1e-12 && (extrusions[3][1] - 10.06).abs() < 1e-12,
        "{:?}",
        extrusions[3]
    );
    let filament_area = std::f64::consts::PI * 0.875 * 0.875;
    let round = std::f64::consts::PI * 0.04;
    let flat = (0.4 - 0.2) * 0.2 + std::f64::consts::PI * 0.01;
    let expected = 0.98 * (2.0 * flat * 39.94 + round * 10.0) / filament_area;
    assert!((program.summary.filament_mm - expected).abs() < 1e-9, "{}", program.summary.filament_mm);
}

#[test]
fn a_move_inside_the_part_neither_retracts_nor_lifts() {
    let line = |y: f64, inside: bool| Path {
        role: Role::Solid,
        width: 0.4,
        closed: false,
        points: vec![[10.0, y], [20.0, y]],
        travel_inside: inside,
    };
    let paths = Paths {
        layers: vec![PathLayer {
            index: 1,
            top: 0.2,
            height: 0.2,
            paths: vec![line(10.0, false), line(15.0, true)],
        }],
    };
    let program = compile(&paths, &params(), &scope()).unwrap();
    let after_first = program.ops.iter().position(|op| matches!(op, Op::Extrude { .. })).unwrap();
    let between: Vec<&Op> = program.ops[after_first..]
        .iter()
        .skip_while(|op| matches!(op, Op::Extrude { .. }))
        .take_while(|op| !matches!(op, Op::Extrude { .. }))
        .collect();
    assert!(
        between.iter().all(|op| !matches!(op, Op::Retract { .. } | Op::Lift { .. } | Op::Wipe { .. })),
        "{between:?}"
    );
}
