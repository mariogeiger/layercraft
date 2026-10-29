//! Filament by volume, times against closed forms, the order of a travel, the shape of a lift,
//! the geometry of an arc, beads fitted as lines and arcs, cooling, and the frame; beads on a
//! bent layer: on its top, laying the exact plastic of the slab under them, less what the face
//! planes off, which the next layer gets back, written as few moves each within the resolution of
//! what it replaces, travels hopping over its highest point.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use crate::moves::chained;
use layercraft_geom::Grid;
use layercraft_path::{Path, PathLayer, Paths};
use layercraft_tune::Field;
use std::f64::consts::{FRAC_PI_4, PI};

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
        face_radius: 0.0,
        slow_down_layer_time: 0.0,
        slow_down_min_speed: 10.0,
        fan_speed: 1.0,
        overhang_fan_speed: 0.0,
        overhang_fan_share: 0.0,
        fan_off_layers: 1,
        resolution: 0.0,
        arc_fitting: false,
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
            PathLayer {
                index: 0,
                top: 0.2,
                height: 0.2,
                warp: Default::default(),
                paths: vec![path.clone()],
            },
            PathLayer { index: 1, top: 0.4, height: 0.2, warp: Default::default(), paths: vec![path] },
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
    Program { layers: vec![Layer { index: 0, z: 0.2, height: 0.2, warp: Default::default(), moves }] }
}

fn travel(to: [f64; 3], speed: f64) -> Move {
    Move { to: Some(to), arc: None, e: 0.0, speed, accel: 1000.0, fan: 0.0, bead: None, wait: 0.0 }
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

/// A plate 256 mm square.
const PLATE: [[f64; 2]; 2] = [[0.0, 0.0], [256.0, 256.0]];

#[test]
fn a_layer_too_short_at_its_floor_waits_off_the_part() {
    let mut p = params();
    p.slow_down_layer_time = 4.0;
    let mut program = compiled(&line(10.0), &p);
    let speeds = |program: &Program| -> Vec<f64> {
        all(program).iter().filter(|m| m.bead.is_some()).map(|m| m.speed).collect()
    };
    assert_eq!(speeds(&program), [50.0, 100.0]);
    let before = program.clone();
    cool(&mut program, &Field::uniform(p.clone()), PLATE).unwrap();
    // 10 mm at 50 mm/s, then at 100 mm/s, is 0.2 s and 0.1 s: held at the 10 mm/s floor.
    assert_eq!(speeds(&program), [10.0, 10.0]);
    // At its floor the first layer takes 1 s. Lifted off its bead's end at (20, 10), it goes
    // 10 mm past the bead the nearest way, along x, and waits the 3 s left.
    let n = before.layers[0].moves.len();
    let added = &program.layers[0].moves[n..];
    assert_eq!(added.len(), 2, "{added:?}");
    assert_eq!(
        (added[0].to, added[0].e, added[0].speed),
        (Some([30.0, 10.0, 0.6000000000000001]), 0.0, 300.0)
    );
    assert_eq!((added[1].to, added[1].e, added[1].wait), (None, 0.0, 3.0));
    // Nothing lands on the last layer: it does not wait.
    assert_eq!(program.layers[1].moves.len(), before.layers[1].moves.len());
    // Travels, wipes and lifts keep their speed.
    let others =
        |moves: &[Move]| -> Vec<Move> { moves.iter().filter(|m| m.bead.is_none()).cloned().collect() };
    assert_eq!(others(&program.layers[0].moves[..n]), others(&before.layers[0].moves));
    assert_eq!(others(&program.layers[1].moves), others(&before.layers[1].moves));
    // The wait is in the program's time.
    let (t0, t1) = (finish(&before, &p.limits, 1.75), finish(&program, &p.limits, 1.75));
    assert!(t1.motion_seconds - t0.motion_seconds > 3.0 + 2.0 * 0.9, "{t0:?} {t1:?}");
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
            PathLayer {
                index: 0,
                top: 0.2,
                height: 0.2,
                warp: Default::default(),
                paths: vec![square.clone()],
            },
            PathLayer {
                index: 1,
                top: 0.4,
                height: 0.2,
                warp: Default::default(),
                paths: vec![square, bridge],
            },
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
            warp: Default::default(),
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
        layers: vec![PathLayer {
            index: 0,
            top: 0.2,
            height: 0.2,
            warp: Default::default(),
            paths: vec![line(10.0), line(30.0)],
        }],
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
            .map(|i| Layer {
                index: i,
                z: 0.2 * (i + 1) as f64,
                height: 0.2,
                warp: Default::default(),
                moves: piece(x, pull, i == 0),
            })
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
fn moves_are_held_to_what_the_hot_end_melts_and_the_z_axis_moves() {
    let bead = Some(Bead { role: Role::SOLID, width: 0.4 });
    let limits = MoveLimits { max_volumetric_speed: 24.0, filament_diameter: 1.75, max_speed_z: 20.0 };
    // 1 mm pushing 1 mm of 1.75 mm filament, 2.405 mm³, asks 240 mm³/s at 100 mm/s: held to 24 mm³/s.
    // A travel down 3 mm over 4 across, 5 long, at 300 mm/s drives Z at 180 mm/s: held to 20.
    let mut program = single(vec![
        travel([0.0; 3], 300.0),
        Move { e: 1.0, bead, ..travel([1.0, 0.0, 0.0], 100.0) },
        travel([5.0, 0.0, -3.0], 300.0),
    ]);
    assert_eq!(limit(&mut program, &limits), 1);
    let area = std::f64::consts::PI * 0.875 * 0.875;
    assert!((program.layers[0].moves[1].speed - 24.0 / area).abs() < 1e-9);
    assert!((program.layers[0].moves[2].speed - 20.0 * 5.0 / 3.0).abs() < 1e-9);
    // Held already: left alone.
    let held = program.clone();
    assert_eq!(limit(&mut program, &limits), 0);
    assert_eq!(program, held);
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
    let paths = Paths {
        layers: vec![PathLayer {
            index: 1,
            top: 0.4,
            height: 0.2,
            warp: Default::default(),
            paths: vec![wall],
        }],
    };
    let mut p = params();
    // By quarters, each closed above, then wholly over air; a bead on solid keeps its speed.
    p.overhang_speeds = vec![0.0, 50.0, 30.0, 10.0, 5.0];
    // From half a bead over air on, the overhang fan replaces the layer's weaker one.
    p.fan_speed = 0.2;
    p.overhang_fan_speed = 0.5;
    p.overhang_fan_share = 0.5;
    let program = compiled(&paths, &p);
    let beads: Vec<&Move> = all(&program).into_iter().filter(|m| m.bead.is_some()).collect();
    let speeds: Vec<f64> = beads.iter().map(|m| m.speed).collect();
    let fans: Vec<f64> = beads.iter().map(|m| m.fan).collect();
    assert!(speeds.iter().zip([100.0, 50.0, 30.0, 5.0]).all(|(s, e)| (s - e).abs() < 1e-9), "{speeds:?}");
    assert_eq!(fans, [0.2, 0.2, 0.5, 0.5]);
}

#[test]
fn a_layer_cools_on_its_hidden_beads_before_its_surface() {
    // 10 mm of outer wall and 10 mm of sparse infill at 100 mm/s: 0.2 s, to last 0.5 s. The
    // infill alone takes the 0.3 s more, at 25 mm/s; the wall keeps its 100 mm/s. To last 1.5 s,
    // the infill falls to its 10 mm/s floor (1 s), and the wall slows to 20 mm/s for the rest.
    let line = |role: Role, y: f64| Path {
        role,
        width: 0.4,
        closed: false,
        points: vec![[10.0, y], [20.0, y]],
        travel_inside: true,
        overhang: Vec::new(),
    };
    let paths = Paths {
        layers: vec![PathLayer {
            index: 1,
            top: 0.4,
            height: 0.2,
            warp: Default::default(),
            paths: vec![line(Role::OUTER_WALL, 10.0), line(Role::SPARSE, 10.5)],
        }],
    };
    for (goal, expected) in [(0.5, [100.0, 25.0]), (1.5, [20.0, 10.0])] {
        let p = MotionParams { slow_down_layer_time: goal, ..params() };
        let mut program = compiled(&paths, &p);
        cool(&mut program, &Field::uniform(p), PLATE).unwrap();
        let speeds: Vec<f64> = all(&program).iter().filter(|m| m.bead.is_some()).map(|m| m.speed).collect();
        assert!(speeds.iter().zip(expected).all(|(s, e)| (s - e).abs() < 1e-6), "goal {goal}: {speeds:?}");
    }
}

#[test]
fn beads_are_written_as_the_fewest_lines_and_arcs_within_the_resolution() {
    // Three quarters of a circle of 10 mm in 128 chords, then 10 mm straight on along its tangent
    // in 9 pieces, then a zigzag 0.05 mm high, more than the resolution: one arc, one line, and
    // the zigzag as it is. Without arcs, the circle takes chords of two pieces, each within
    // the resolution.
    let (c, r, eps) = ([50.0, 50.0], 10.0, 0.012);
    let quarter = std::f64::consts::FRAC_PI_2;
    let mut points: Vec<[f64; 2]> = (0..=128)
        .map(|k| 3.0 * quarter * k as f64 / 128.0)
        .map(|a: f64| [c[0] + r * a.cos(), c[1] + r * a.sin()])
        .collect();
    let end = *points.last().unwrap();
    points.extend((1..=9).map(|k| [end[0] + 10.0 * k as f64 / 9.0, end[1]]));
    let end = *points.last().unwrap();
    points.extend((1..=6).map(|k| [end[0] + k as f64, end[1] + 0.05 * (k % 2) as f64]));
    let wall = Path {
        role: Role::OUTER_WALL,
        width: 0.4,
        closed: false,
        points: points.clone(),
        travel_inside: false,
        overhang: Vec::new(),
    };
    let paths = Paths {
        layers: vec![PathLayer {
            index: 1,
            top: 0.4,
            height: 0.2,
            warp: Default::default(),
            paths: vec![wall],
        }],
    };
    let traced = compiled(&paths, &params());
    let beads = |program: &Program| -> Vec<(Option<[f64; 3]>, Move)> {
        chained(&program.layers[0].moves, None)
            .filter(|(_, m)| m.bead.is_some())
            .map(|(f, m)| (f, m.clone()))
            .collect()
    };
    let (from, first) = &beads(&traced)[0];
    let per_mm = first.e / first.length(from.unwrap());

    let mut arcs = traced.clone();
    fit(&mut arcs, eps, true);
    let fitted = beads(&arcs);
    assert_eq!(fitted.len(), 1 + 1 + 6, "{fitted:?}");
    let arc = fitted[0].1.arc.unwrap();
    assert!((arc.centre[0] - c[0]).abs() < 1e-9 && (arc.centre[1] - c[1]).abs() < 1e-9, "{arc:?}");
    assert!((arc.turn - 3.0 * quarter).abs() < 1e-9);
    assert!((fitted[0].1.e - per_mm * r * 3.0 * quarter).abs() < 1e-12);
    assert!(fitted[1].1.arc.is_none() && fitted[1].1.to.unwrap()[0] == end[0]);
    for (f, m) in &fitted {
        assert!((m.e - per_mm * m.length(f.unwrap())).abs() < 1e-12);
        assert_eq!((m.speed, m.fan, m.bead), (first.speed, first.fan, first.bead));
    }

    let mut lines = traced.clone();
    fit(&mut lines, eps, false);
    let fitted = beads(&lines);
    assert_eq!(fitted.len(), 64 + 1 + 6);
    let ends: Vec<[f64; 2]> = std::iter::once(points[0])
        .chain(fitted.iter().map(|(_, m)| [m.to.unwrap()[0], m.to.unwrap()[1]]))
        .collect();
    let to_polyline = |p: [f64; 2]| {
        ends.windows(2)
            .map(|w| {
                let (a, b) = (w[0], w[1]);
                let e = [b[0] - a[0], b[1] - a[1]];
                let s = (((p[0] - a[0]) * e[0] + (p[1] - a[1]) * e[1]) / (e[0] * e[0] + e[1] * e[1]))
                    .clamp(0.0, 1.0);
                (p[0] - a[0] - s * e[0]).hypot(p[1] - a[1] - s * e[1])
            })
            .fold(f64::INFINITY, f64::min)
    };
    assert!(points.iter().all(|&p| to_polyline(p) <= eps));

    // A resolution of nothing leaves the beads as traced.
    let mut same = traced.clone();
    fit(&mut same, 0.0, true);
    assert_eq!(same, traced);
}

/// A ramp on a grid of millimetre squares: rising 0.05 mm per mm from x = 0 to x = 10, then
/// level at 0.5 mm out to x = 20, for y from 0 to 20.
fn ramp() -> Relief {
    let grid = Grid { origin: [0.0, 0.0], cell: 1.0 };
    Relief::from_fn(grid, [0, 0], [21, 21], |i, _| 0.05 * i.min(10) as f64)
}

fn bead(points: Vec<[f64; 2]>) -> Path {
    Path { role: Role::SOLID, width: 0.4, closed: false, points, travel_inside: false, overhang: Vec::new() }
}

/// A flat first layer, laying nothing, and over it a second bent by `ramp`, laying `paths`.
fn bent(paths: Vec<Path>) -> Paths {
    Paths {
        layers: vec![
            PathLayer { index: 0, top: 0.2, height: 0.2, warp: Default::default(), paths: Vec::new() },
            PathLayer { index: 1, top: 0.4, height: 0.2, warp: ramp(), paths },
        ],
    }
}

/// The bead moves of the bent layer, each with where it starts.
fn beads(program: &Program) -> Vec<([f64; 3], Move)> {
    chained(&program.layers[1].moves, None)
        .filter(|(_, m)| m.bead.is_some())
        .map(|(f, m)| (f.unwrap(), m.clone()))
        .collect()
}

/// A move's length across the plate.
fn plate(from: [f64; 3], m: &Move) -> f64 {
    let to = m.to.unwrap();
    (to[0] - from[0]).hypot(to[1] - from[1])
}

/// One bead across the ramp and on along its level top, from x = 2 to x = 18.
fn across() -> Paths {
    bent(vec![bead(vec![[2.0, 5.5], [18.0, 5.5]])])
}

#[test]
fn a_bead_on_a_bent_layer_lies_on_its_top_and_lays_its_slab() {
    let p = params();
    let traced = beads(&compiled(&across(), &p));
    // Cut wherever the ramp may bend: at x = 3 … 18, and where the squares' diagonals cross.
    assert_eq!(traced.len(), 32, "{traced:?}");
    for (_, m) in &traced {
        let to = m.to.unwrap();
        assert!((to[2] - (0.4 + ramp().at([to[0], to[1]]))).abs() < 1e-12, "{to:?} off the layer");
    }
    // The slab, 0.2 mm plus the ramp, runs from 0.3 to 0.7 mm thick over 8 mm, then stays for
    // 8 more: the bead fills its strip through it, as wide as beads 0.4 mm wide on a layer 0.2 mm
    // high lie apart, `0.4 − 0.2·(1 − π/4)`.
    let volume = (0.4 - 0.2 * (1.0 - FRAC_PI_4)) * (8.0 * 0.5 + 8.0 * 0.7);
    let pushed: f64 = traced.iter().map(|(_, m)| m.e).sum();
    let expected = p.flow_ratio * volume / disc_area(p.filament_diameter);
    assert!((pushed - expected).abs() < 1e-12 * expected, "{pushed} against {expected}");
}

#[test]
fn a_bent_bead_is_written_as_few_moves_each_laying_within_its_share_of_the_slab() {
    let (p, eps) = (params(), 0.012);
    let traced = compiled(&across(), &p);
    let mut program = traced.clone();
    fit(&mut program, eps, true);
    let (before, after) = (beads(&traced), beads(&program));
    // Straight in space on the ramp and on the level, but the slab thickening on the ramp: a
    // few moves up it, one along the level.
    assert!(after.len() <= 8, "{after:?}");
    assert_eq!(after.iter().filter(|(from, _)| from[0] >= 10.0).count(), 1, "{after:?}");
    let total = |b: &[([f64; 3], Move)]| b.iter().map(|(_, m)| m.e).sum::<f64>();
    assert!((total(&after) - total(&before)).abs() < 1e-12 * total(&before));
    // Each piece traced lays within 2ε/w of the plastic per mm of the move laying it.
    for (from, m) in &after {
        let (x0, x1, per_mm) = (from[0], m.to.unwrap()[0], m.e / plate(*from, m));
        for (f, t) in before.iter().filter(|(f, t)| f[0] >= x0 - 1e-9 && t.to.unwrap()[0] <= x1 + 1e-9) {
            let own = t.e / plate(*f, t);
            assert!((own - per_mm).abs() <= 2.0 * eps / 0.4 * own, "{own} under {per_mm}");
        }
    }
}

#[test]
fn chords_of_a_helix_climbing_evenly_are_one_arc_and_of_a_vast_circle_lines() {
    let bead = Some(Bead { role: Role::OUTER_WALL, width: 0.4 });
    let chain = |points: Vec<[f64; 3]>| {
        let mut moves = vec![travel(points[0], 300.0)];
        moves.extend(points.windows(2).map(|w| {
            let l = (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
            Move { e: 0.01 * l, bead, ..travel(w[1], 100.0) }
        }));
        single(moves)
    };
    // Half a turn of radius 10 about (50, 50) in 64 chords, climbing 0.5 mm evenly: one arc
    // laying 0.01 mm of filament per mm across the plate.
    let helix: Vec<[f64; 3]> = (0..=64)
        .map(|k| (PI * k as f64 / 64.0, k as f64 / 64.0))
        .map(|(a, s)| [50.0 + 10.0 * a.cos(), 50.0 + 10.0 * a.sin(), 1.0 + 0.5 * s])
        .collect();
    let mut program = chain(helix.clone());
    fit(&mut program, 0.012, true);
    let moves = &program.layers[0].moves;
    assert_eq!(moves.len(), 2, "{moves:?}");
    let arc = moves[1].arc.unwrap();
    assert!((arc.centre[0] - 50.0).abs() < 1e-9 && (arc.centre[1] - 50.0).abs() < 1e-9, "{arc:?}");
    assert!((arc.turn - PI).abs() < 1e-9 && moves[1].to == Some(helix[64]));
    assert!((moves[1].e - 0.01 * 10.0 * PI).abs() < 1e-12);
    // 20 mm of a circle 2 m across, bowing 0.025 mm from its chord, past the resolution: an arc
    // about a centre that far is lost to the machine's rounding, so lines.
    let vast: Vec<[f64; 3]> = (0..=40)
        .map(|k| (k as f64 - 20.0) / 2.0)
        .map(|x| [100.0 + x, 50.0 + 2000.0 - (2000.0f64 * 2000.0 - x * x).sqrt(), 1.0])
        .collect();
    let mut program = chain(vast);
    fit(&mut program, 0.012, true);
    let moves = &program.layers[0].moves;
    assert!(moves.len() > 2 && moves.iter().all(|m| m.arc.is_none()), "{moves:?}");
}

#[test]
fn a_travel_across_a_bent_layer_hops_over_its_highest_point_on_the_way() {
    // From a bead at the ramp's foot, to one beside it, then to one on its level top.
    let paths = bent(vec![
        bead(vec![[0.5, 2.5], [1.5, 2.5]]),
        bead(vec![[0.5, 7.5], [1.5, 7.5]]),
        bead(vec![[15.0, 12.5], [16.0, 12.5]]),
    ]);
    let program = compiled(&paths, &params());
    let moves = &program.layers[1].moves;
    // The highest each travel between beads goes.
    let mut hops = Vec::new();
    let mut top = f64::NEG_INFINITY;
    for (k, m) in moves.iter().enumerate() {
        if m.bead.is_some() {
            if top > f64::NEG_INFINITY && k > 0 && moves[k - 1].bead.is_none() {
                hops.push(top);
            }
            top = f64::NEG_INFINITY;
        } else if let Some(to) = m.to {
            top = top.max(to[2]);
        }
    }
    // Over the foot, the ramp 0.075 mm high at x = 1.5; over the top, 0.5 mm; the hop 0.4 mm on.
    assert_eq!(hops.len(), 3, "{moves:?}");
    assert!((hops[1] - (0.4 + 0.075 + 0.4)).abs() < 1e-12, "{hops:?}");
    assert!((hops[2] - (0.4 + 0.5 + 0.4)).abs() < 1e-12, "{hops:?}");
}

/// The plastic each layer pushes, with the nozzle's face `face_radius` round.
fn pushed_by_layer(paths: &Paths, face_radius: f64) -> Vec<f64> {
    let program = compiled(paths, &MotionParams { face_radius, ..params() });
    program.layers.iter().map(|l| l.moves.iter().filter(|m| m.bead.is_some()).map(|m| m.e).sum()).collect()
}

#[test]
fn a_bead_down_the_ramp_pushes_only_what_its_face_leaves() {
    // Down from x = 8 to x = 2, 0.05 mm per mm: a point δ short of the end keeps the face, 0.5 mm
    // round, `0.05·min(δ, √(ρ² − o²))` lower over it, `o` across, as the bead goes on down and
    // climbs off its end. Over the strip's thirds at 0 and ±s/3, that takes `0.05·(6r − r²/2)`
    // off the slab's 2.7 mm² along it, `r` the reach along of each third.
    let down = bent(vec![bead(vec![[8.0, 5.5], [2.0, 5.5]])]);
    let s = 0.4 - 0.2 * (1.0 - FRAC_PI_4);
    let cut = |r: f64| 0.05 * (6.0 * r - r * r / 2.0);
    let r = (0.25 - (s / 3.0) * (s / 3.0)).sqrt();
    let kept = 1.0 - (cut(0.5) + 2.0 * cut(r)) / 3.0 / 2.7;
    let (bare, planed) = (pushed_by_layer(&down, 0.0), pushed_by_layer(&down, 0.5));
    assert!((planed[1] / bare[1] - kept).abs() < 1e-4, "{} against {kept}", planed[1] / bare[1]);
    // Up the ramp, the face only ever stands higher: every bead keeps its plastic.
    let up = bent(vec![bead(vec![[2.0, 5.5], [8.0, 5.5]])]);
    assert_eq!(pushed_by_layer(&up, 0.5), pushed_by_layer(&up, 0.0));
    // Level, nothing changes.
    let program = |face_radius| compiled(&line(10.0), &MotionParams { face_radius, ..params() });
    assert_eq!(program(0.5), program(0.0));
}

#[test]
fn what_the_face_takes_off_a_layer_the_next_gets_back() {
    // Down the ramp, then up it again on a layer bent alike over it: the second lays its slab
    // and what the first lost, the plastic of both as without the face.
    let mut paths = bent(vec![bead(vec![[8.0, 5.5], [2.0, 5.5]])]);
    let next = PathLayer {
        index: 2,
        top: 0.6,
        paths: vec![bead(vec![[2.0, 5.5], [8.0, 5.5]])],
        ..paths.layers[1].clone()
    };
    paths.layers.push(next);
    let (bare, planed) = (pushed_by_layer(&paths, 0.0), pushed_by_layer(&paths, 0.5));
    assert!(planed[1] < 0.95 * bare[1] && planed[2] > 1.05 * bare[2], "{planed:?} against {bare:?}");
    let total = |v: &[f64]| v.iter().sum::<f64>();
    assert!((total(&planed) - total(&bare)).abs() < 1e-12 * total(&bare), "{planed:?} against {bare:?}");
}
