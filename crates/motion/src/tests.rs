//! Filament by volume, times against closed forms, the order of a travel, the shape of a lift,
//! the geometry of an arc, cooling, and the frame.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use crate::moves::chained;
use layercraft_path::{Path, PathLayer, Paths};
use layercraft_tune::Field;

fn role(speed: f64) -> RoleMotion {
    RoleMotion { speed, accel: 1000.0, flow: 1.0 }
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
        overhang_speeds: Vec::new(),
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
        lift_radius: 0.0,
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
        role: Role::SPARSE,
        width: 0.4,
        closed: false,
        points: vec![[10.0, 10.0], [10.0 + len, 10.0]],
        travel_inside: false,
        overhang: Vec::new(),
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

fn compiled(paths: &Paths, p: &MotionParams) -> Program {
    compile(paths, &Field::uniform(p.clone())).unwrap()
}

/// Every move of the program, in order.
fn all(program: &Program) -> Vec<&Move> {
    program.layers.iter().flat_map(|l| &l.moves).collect()
}

/// A program of one layer at 0.2 mm of these moves.
fn single(moves: Vec<Move>) -> Program {
    Program { layers: vec![Layer { index: 0, z: 0.2, height: 0.2, moves }] }
}

fn travel(to: [f64; 3], speed: f64) -> Move {
    Move { to: Some(to), arc: None, e: 0.0, speed, accel: 1000.0, fan: 0.0, bead: None }
}

/// Whether a move climbs from where the nozzle is, straight or turning, laying nothing.
fn climbs(from: Option<[f64; 3]>, m: &Move) -> bool {
    match (from, m.to) {
        (Some(f), Some(t)) => {
            m.e == 0.0 && t[2] > f[2] && (m.arc.is_some() || (t[0] == f[0] && t[1] == f[1]))
        }
        _ => false,
    }
}

#[test]
fn filament_follows_the_bead_volume() {
    let p = params();
    let program = compiled(&line(10.0), &p);
    let summary = finish(&program, &p.limits, p.filament_diameter);
    let area = (0.4 - 0.2) * 0.2 + std::f64::consts::PI * 0.01;
    let filament_area = std::f64::consts::PI * 0.875 * 0.875;
    let expected = 2.0 * 0.98 * area * 10.0 / filament_area;
    assert!((summary.filament_mm - expected).abs() < 1e-12);
    // The plastic pushed: the flow ratio's share of the bead's volume.
    assert!((summary.volume_per_layer[0] - 0.98 * area * 10.0).abs() < 1e-12);
    let frame = frame(&program, &p.templates, &scope()).unwrap();
    assert!(frame.start.starts_with("; start me\n"), "{}", frame.start);
    assert_eq!(frame.layers.len(), 2);
}

#[test]
fn a_lone_move_takes_its_trapezoid() {
    let limits = Limits { max_accel: 1000.0, jerk: 0.0, max_speed_xy: 1000.0, max_speed_z: 20.0 };
    // 100 mm at 100 mm/s with 1000 mm/s²: 0.1 s up and 0.1 s down over 5 mm each, 90 mm cruising.
    let t = estimate(&single(vec![travel([100.0, 0.0, 0.0], 100.0)]), &limits);
    assert!((t[0] - 1.1).abs() < 1e-9, "{t:?}");
    // 1 mm never reaches cruise: a triangle of peak √(a·L) = 31.6 mm/s.
    let t = estimate(&single(vec![travel([1.0, 0.0, 0.0], 100.0)]), &limits);
    assert!((t[0] - 2.0 * (1.0f64 / 1000.0).sqrt()).abs() < 1e-9);
}

#[test]
fn straight_runs_keep_their_speed_through_joints() {
    let limits = Limits { max_accel: 1000.0, jerk: 0.0, max_speed_xy: 1000.0, max_speed_z: 20.0 };
    let split = (1..=10).map(|i| travel([10.0 * i as f64, 0.0, 0.0], 100.0)).collect();
    let t = estimate(&single(split), &limits);
    assert!((t.last().unwrap() - 1.1).abs() < 1e-9);
}

#[test]
fn a_layer_starts_crossing_over_at_the_hop_then_lands() {
    let program = compiled(&line(10.0), &params());
    let moves = &program.layers[0].moves;
    // From the machine's start, wherever it leaves the nozzle: over at the hop, then down.
    assert!(moves[0].to.is_none() && moves[0].e < 0.0);
    assert_eq!(moves[1].to, Some([10.0, 10.0, 0.6000000000000001]));
    assert_eq!(moves[2].to, Some([10.0, 10.0, 0.2]));
    assert!(moves[3].to.is_none() && moves[3].e > 0.0);
    assert!(moves[4].bead.is_some());
}

#[test]
fn short_layers_slow_down_to_cool() {
    let mut p = params();
    p.slow_down_layer_time = 4.0;
    let mut program = compiled(&line(10.0), &p);
    let speeds = |program: &Program| -> Vec<f64> {
        all(program).iter().filter(|m| m.bead.is_some()).map(|m| m.speed).collect()
    };
    assert_eq!(speeds(&program)[1], 100.0);
    let before = program.clone();
    cool(&mut program, &Field::uniform(p)).unwrap();
    // 10 mm at 100 mm/s is 0.1 s: slowed to 2.5 mm/s, held at the 10 mm/s floor.
    assert_eq!(speeds(&program)[1], 10.0);
    // Travels, wipes and lifts keep their speed.
    let others = |program: &Program| -> Vec<Move> {
        all(program).into_iter().filter(|m| m.bead.is_none()).cloned().collect()
    };
    assert_eq!(others(&program), others(&before));
}

#[test]
fn missing_template_variables_are_named() {
    let mut p = params();
    p.templates.end = "M140 S[bed]".into();
    let program = compiled(&line(1.0), &p);
    let err = frame(&program, &p.templates, &scope()).unwrap_err().to_string();
    assert!(err.contains("bed"), "{err}");
}

#[test]
fn retraction_wipes_back_over_the_bead() {
    let mut p = params();
    p.wipe_distance = 1.0;
    let program = compiled(&line(10.0), &p);
    let moves = &program.layers[0].moves;
    let wipes: Vec<&Move> = moves.iter().filter(|m| m.to.is_some() && m.e < 0.0).collect();
    // Back from the line's end at x = 20 towards its start, 1 mm, pulling the whole 0.4 mm.
    let last = wipes.last().unwrap();
    assert!((last.to.unwrap()[0] - 19.0).abs() < 1e-9, "{wipes:?}");
    let pulled: f64 = wipes.iter().map(|w| w.e).sum();
    assert!((pulled + 0.4).abs() < 1e-12);
    // The filament never moves faster than the retraction speed: 0.4 mm over 1 mm at v ≤ 75 mm/s.
    assert!(wipes.iter().all(|w| w.speed * 0.4 / 1.0 <= p.retract_speed + 1e-9));
    // And the print ends lifted off the part, before the machine's own end.
    let end = program.layers[1].moves.last().unwrap();
    assert!((end.to.unwrap()[2] - 0.8).abs() < 1e-9, "{end:?}");
}

#[test]
fn loops_stop_short_of_their_seam_and_bridges_lay_round_strands() {
    let square = Path {
        role: Role::OUTER_WALL,
        width: 0.4,
        closed: true,
        points: vec![[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]],
        travel_inside: false,
        overhang: Vec::new(),
    };
    let bridge = Path {
        role: Role::BRIDGE,
        width: 0.4,
        closed: false,
        points: vec![[10.0, 10.0], [20.0, 10.0]],
        travel_inside: false,
        overhang: Vec::new(),
    };
    let paths = Paths {
        layers: vec![
            PathLayer { index: 0, top: 0.2, height: 0.2, paths: vec![square.clone()] },
            PathLayer { index: 1, top: 0.4, height: 0.2, paths: vec![square, bridge] },
        ],
    };
    let mut p = params();
    p.seam_gap = 0.06;
    let program = compiled(&paths, &p);
    let extrusions: Vec<[f64; 3]> =
        all(&program).iter().filter(|m| m.bead.is_some()).filter_map(|m| m.to).collect();
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
    let summary = finish(&program, &p.limits, p.filament_diameter);
    assert!((summary.filament_mm - expected).abs() < 1e-9, "{}", summary.filament_mm);
}

#[test]
fn a_move_inside_the_part_neither_retracts_nor_lifts() {
    let line = |y: f64, inside: bool| Path {
        role: Role::SOLID,
        width: 0.4,
        closed: false,
        points: vec![[10.0, y], [20.0, y]],
        travel_inside: inside,
        overhang: Vec::new(),
    };
    let paths = Paths {
        layers: vec![PathLayer {
            index: 1,
            top: 0.2,
            height: 0.2,
            paths: vec![line(10.0, false), line(15.0, true)],
        }],
    };
    let program = compiled(&paths, &params());
    let moves = &program.layers[0].moves;
    let after_first = moves.iter().position(|m| m.bead.is_some()).unwrap();
    let between: Vec<&Move> = moves[after_first..]
        .iter()
        .skip_while(|m| m.bead.is_some())
        .take_while(|m| m.bead.is_none())
        .collect();
    assert!(
        between.iter().all(|m| m.e == 0.0 && m.arc.is_none() && m.to.is_some_and(|t| t[2] == 0.2)),
        "{between:?}"
    );
}

#[test]
fn lifts_turn_a_helix_where_the_nozzle_is_known() {
    let line = |y: f64| Path {
        role: Role::SPARSE,
        width: 0.4,
        closed: false,
        points: vec![[10.0, y], [20.0, y]],
        travel_inside: false,
        overhang: Vec::new(),
    };
    let paths = Paths {
        layers: vec![PathLayer { index: 0, top: 0.2, height: 0.2, paths: vec![line(10.0), line(30.0)] }],
    };
    let mut p = params();
    p.lift_radius = 1.0;
    p.wipe_distance = 1.0;
    let program = compiled(&paths, &p);
    let lifts: Vec<&Move> =
        chained(&program.layers[0].moves, None).filter(|(f, m)| climbs(*f, m)).map(|(_, m)| m).collect();
    // Off the first line, then off the second at the end; from the start, the nozzle crossed over.
    assert_eq!(lifts.len(), 2, "{lifts:?}");
    // Off the first line, wiped back from (20, 10) to (19, 10), setting off towards the second
    // line's start at (10, 30): counter-clockwise, the axis lies a radius left of that heading.
    let h = [-9.0 / 481f64.sqrt(), 20.0 / 481f64.sqrt()];
    let arc = lifts[0].arc.unwrap();
    assert_eq!(arc.turn, std::f64::consts::TAU);
    assert!((arc.centre[0] - (19.0 - h[1])).abs() < 1e-12 && (arc.centre[1] - (10.0 + h[0])).abs() < 1e-12);
    // Flown at the travel speed, the Z axis below its limit: 20 mm/s · √((2π)² + 0.4²) / 0.4 > 300.
    assert_eq!(lifts[0].speed, 300.0);
    // The print ends climbing off the wipe's end at (19, 30), setting off the way it wiped, -x.
    assert_eq!(lifts[1].arc.unwrap().centre, [19.0, 29.0]);
}

#[test]
fn a_helix_runs_at_its_centripetal_limit() {
    let limits = Limits { max_accel: 1000.0, jerk: 0.0, max_speed_xy: 1000.0, max_speed_z: 20.0 };
    // One turn of radius 1 climbing 0.4 mm: v ≤ √(a·r) = 31.6 mm/s, from rest to rest.
    let helix = Move {
        arc: Some(Arc { centre: [0.0, 1.0], turn: std::f64::consts::TAU }),
        ..travel([0.0, 0.0, 0.4], 300.0)
    };
    let t = estimate(&single(vec![helix]), &limits);
    let length = std::f64::consts::TAU.hypot(0.4);
    let v = 1000f64.sqrt();
    let expected = 2.0 * v / 1000.0 + (length - v * v / 1000.0) / v;
    assert!((t[0] - expected).abs() < 1e-9, "{t:?} against {expected}");
}

#[test]
fn an_arc_sweeps_its_extremes_and_turns_its_tangent() {
    // Half a turn counter-clockwise from (1, 0) about the origin, to (-1, 0), climbing 1 mm.
    let half = Move {
        arc: Some(Arc { centre: [0.0, 0.0], turn: std::f64::consts::PI }),
        ..travel([-1.0, 0.0, 1.0], 10.0)
    };
    let from = [1.0, 0.0, 0.0];
    assert!((half.length(from) - std::f64::consts::PI.hypot(1.0)).abs() < 1e-12);
    let [lo, hi] = half.bounds(from);
    assert!((hi[1] - 1.0).abs() < 1e-12 && lo[1].abs() < 1e-12 && lo[0] == -1.0 && hi[0] == 1.0);
    let (enter, leave) = half.directions(from);
    assert!(enter[0].abs() < 1e-12 && enter[1] > 0.0 && leave[1] < 0.0 && enter[2] > 0.0);
    let p = half.point(from, 0.5);
    assert!(p[0].abs() < 1e-12 && (p[1] - 1.0).abs() < 1e-12 && (p[2] - 0.5).abs() < 1e-12);
    // Clockwise the other way round: it sweeps the bottom.
    let under = Move { arc: Some(Arc { centre: [0.0, 0.0], turn: -std::f64::consts::PI }), ..half };
    assert!((under.bounds(from)[0][1] + 1.0).abs() < 1e-12);
}

fn tuned(text: &str) -> Field<MotionParams> {
    let tuning: layercraft_tune::Tuning = serde_json::from_str(text).unwrap();
    Field::new(params(), &tuning, "motion").unwrap()
}

#[test]
fn each_bead_takes_the_parameters_at_its_place() {
    // Above z = 0.3, a lift turns a 1 mm helix; below, it goes straight up.
    let field = tuned(r#"[{"within": {"z": [0.3, 1]}, "set": {"motion": {"lift_radius": 1.0}}}]"#);
    let program = compile(&line(10.0), &field).unwrap();
    let lifts: Vec<&Move> = program
        .layers
        .iter()
        .flat_map(|l| chained(&l.moves, None).filter(|(f, m)| climbs(*f, m)).map(|(_, m)| m))
        .collect();
    // Off the first layer's bead, below 0.3, and off the last one, above.
    assert_eq!(lifts.len(), 2, "{lifts:?}");
    assert!(lifts[0].arc.is_none() && lifts[1].arc.is_some());
}

#[test]
fn what_is_pulled_back_is_pushed_again() {
    // The second layer's bead pulls 1.5 mm, the first's 0.4: each push matches its pull.
    let field = tuned(r#"[{"within": {"z": [0.3, 1]}, "set": {"motion": {"retract_length": 1.5}}}]"#);
    let program = compile(&line(10.0), &field).unwrap();
    let mut pulled = 0.0;
    let mut pushes = Vec::new();
    for m in all(&program) {
        if m.bead.is_some() {
            continue;
        }
        if m.to.is_none() && m.e > 0.0 {
            pushes.push(m.e);
            assert!((pulled - m.e).abs() < 1e-12, "pushed {} after pulling {pulled}", m.e);
            pulled = 0.0;
        } else {
            pulled -= m.e;
        }
    }
    assert_eq!(pushes.len(), 2);
    // The print ends with the last bead's pull, never pushed.
    assert!((pulled - 1.5).abs() < 1e-12);
}

#[test]
fn an_edited_program_is_summarised_from_its_moves() {
    let p = params();
    let mut program = compiled(&line(10.0), &p);
    let before = finish(&program, &p.limits, 1.75);
    for m in program.layers.iter_mut().flat_map(|l| &mut l.moves) {
        if m.bead.is_some() {
            m.e *= 2.0;
            m.speed /= 2.0;
        }
    }
    let after = finish(&program, &p.limits, 1.75);
    assert!((after.filament_mm - 2.0 * before.filament_mm).abs() < 1e-12);
    assert!(after.motion_seconds > before.motion_seconds);
}

#[test]
fn programs_merge_layer_by_layer_nearest_first_with_their_filament_as_they_left_it() {
    let bead = Some(Bead { role: Role::SOLID, width: 0.4 });
    // Over, down, push back what the layer below pulled, a bead, pull.
    let piece = |x: f64, pull: f64, first: bool| {
        vec![
            travel([x, 0.0, 0.6], 300.0),
            Move { to: None, e: if first { 0.0 } else { pull }, ..travel([0.0; 3], 30.0) },
            Move { e: 0.1, bead, ..travel([x + 5.0, 0.0, 0.2], 50.0) },
            Move { to: None, e: -pull, ..travel([0.0; 3], 30.0) },
        ]
    };
    let program = |x: f64, pull: f64, layers: usize| Program {
        layers: (0..layers)
            .map(|i| Layer { index: i, z: 0.2 * (i + 1) as f64, height: 0.2, moves: piece(x, pull, i == 0) })
            .collect(),
    };
    // A far one pulling 0.4, a near one pulling 0.8 and one layer taller.
    let merged = merge(&[program(100.0, 0.4, 2), program(10.0, 0.8, 3)], 25.0);
    assert_eq!(merged.layers.len(), 3);
    assert_eq!(merged.layers.iter().map(|l| l.z).collect::<Vec<_>>(), vec![0.2, 0.4, 0.6000000000000001]);
    // At one height, the first given first, then the next nearest to where it ended.
    let starts: Vec<f64> = merged.layers[1]
        .moves
        .iter()
        .filter(|m| m.bead.is_none() && m.e == 0.0)
        .filter_map(|m| m.to)
        .map(|p| p[0])
        .collect();
    assert_eq!(starts, vec![10.0, 100.0]);
    // The filament balances: every push gives back exactly what is pulled before it.
    let mut pulled = 0.0;
    for m in all(&merged).into_iter().filter(|m| m.bead.is_none()) {
        pulled -= m.e;
        assert!(pulled > -1e-9, "pushed more than was pulled: {pulled}");
    }
    // The near one leaves 0.8 pulled; the far one expects 0.4: 0.4 is given back first, at 25 mm/s.
    let fix = merged.layers[1].moves.iter().find(|m| m.speed == 25.0).unwrap();
    assert!((fix.e - 0.4).abs() < 1e-12 && fix.to.is_none());
}

#[test]
fn beads_are_held_to_what_the_hot_end_melts() {
    let bead = Some(Bead { role: Role::SOLID, width: 0.4 });
    // 1 mm pushing 1 mm of 1.75 mm filament, 2.405 mm³, asks 240 mm³/s at 100 mm/s: held to 24 mm³/s.
    let mut program =
        single(vec![travel([0.0; 3], 300.0), Move { e: 1.0, bead, ..travel([1.0, 0.0, 0.0], 100.0) }]);
    assert_eq!(melt(&mut program, 24.0, 1.75), 1);
    let area = std::f64::consts::PI * 0.875 * 0.875;
    assert!((program.layers[0].moves[1].speed - 24.0 / area).abs() < 1e-9);
    // Held already: left alone.
    assert_eq!(melt(&mut program, 24.0, 1.75), 0);
}

#[test]
fn a_wall_slows_by_how_far_it_hangs() {
    // Four segments of an outer wall, on solid, a third over air, three quarters, wholly over.
    let wall = Path {
        role: Role::OUTER_WALL,
        width: 0.4,
        closed: false,
        points: vec![[10.0, 10.0], [12.0, 10.0], [14.0, 10.0], [16.0, 10.0], [18.0, 10.0]],
        travel_inside: false,
        overhang: vec![0.0, 0.3, 0.75, 1.0],
    };
    let paths = Paths { layers: vec![PathLayer { index: 1, top: 0.4, height: 0.2, paths: vec![wall] }] };
    let mut p = params();
    // By quarters, each held at its band's middle, then wholly over air; the first quarter
    // keeps the wall's speed. Between, linear: 0.3 lies 0.7 of the way from 1/8 (100) to 3/8
    // (50), 0.75 halfway from 5/8 (30) to 7/8 (10).
    p.overhang_speeds = vec![0.0, 50.0, 30.0, 10.0, 5.0];
    let program = compiled(&paths, &p);
    let speeds: Vec<f64> = all(&program).iter().filter(|m| m.bead.is_some()).map(|m| m.speed).collect();
    let expected = [100.0, 65.0, 20.0, 5.0];
    assert!(speeds.iter().zip(expected).all(|(s, e)| (s - e).abs() < 1e-9), "{speeds:?}");
}
