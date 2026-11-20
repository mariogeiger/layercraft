//! Filament by volume, times against closed forms, the order of a travel, the shape of a lift,
//! the geometry of an arc, each bead's parameters, the moves held to the machine's limits and the
//! parameters checked; the other passes have their own files.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use crate::moves::chained;
use layercraft_path::{Path, PathPass, Paths};
use layercraft_tune::Field;
use std::f64::consts::PI;

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
        support: role(100.0),
        support_interface: role(80.0),
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
        travel_crosses: false,
        overhang: Vec::new(),
    };
    Paths {
        carrier: Default::default(),
        passes: vec![
            PathPass {
                index: 0,
                top: 0.2,
                height: 0.2,
                warp: Default::default(),
                paths: vec![path.clone()],
                material: 0,
            },
            PathPass {
                index: 1,
                top: 0.4,
                height: 0.2,
                warp: Default::default(),
                paths: vec![path],
                material: 0,
            },
        ],
    }
}

fn scope() -> layercraft_template::Scope {
    [("who".to_string(), layercraft_template::Value::Str("me".into()))].into_iter().collect()
}

/// One material held at 220 °C on a 60 °C bed, every layer alike: the frame sets nothing.
fn steady() -> Heat {
    Heat { nozzle: vec![[220.0, 220.0]], bed: [60.0, 60.0] }
}

fn compiled(paths: &Paths, p: &MotionParams) -> Program {
    compile(paths, &[Field::uniform(p.clone())]).unwrap()
}

/// What `program` does, timed as the machine `p` runs it, every material pushing `p`'s filament.
fn summary_of(program: &Program, p: &MotionParams) -> Summary {
    summarize(program, &estimate(program, &p.limits), &[p.filament()])
}

/// Every move of the program, in order.
fn all(program: &Program) -> Vec<&Move> {
    program.passes.iter().flat_map(|l| &l.moves).collect()
}

/// A program of one layer at 0.2 mm of these moves.
fn single(moves: Vec<Move>) -> Program {
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
    let summary = summary_of(&program, &p);
    let area = (0.4 - 0.2) * 0.2 + std::f64::consts::PI * 0.01;
    let filament_area = std::f64::consts::PI * 0.875 * 0.875;
    let expected = 2.0 * 0.98 * area * 10.0 / filament_area;
    assert!((summary.filament_mm - expected).abs() < 1e-12);
    // The plastic pushed: the flow ratio's share of the bead's volume.
    assert!((summary.volume_per_layer[0] - 0.98 * area * 10.0).abs() < 1e-12);
    let frame = frame(&program, &p.templates, &steady(), &scope()).unwrap();
    assert!(frame.start.starts_with("; start me\n"), "{}", frame.start);
    assert_eq!(frame.passes.len(), 2);
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
    let moves = &program.passes[0].moves;
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
fn retraction_wipes_back_over_the_bead() {
    let mut p = params();
    p.wipe_distance = 1.0;
    let program = compiled(&line(10.0), &p);
    let moves = &program.passes[0].moves;
    let wipes: Vec<&Move> = moves.iter().filter(|m| m.to.is_some() && m.e < 0.0).collect();
    // Back from the line's end at x = 20 towards its start, 1 mm, pulling the whole 0.4 mm.
    let last = wipes.last().unwrap();
    assert!((last.to.unwrap()[0] - 19.0).abs() < 1e-9, "{wipes:?}");
    let pulled: f64 = wipes.iter().map(|w| w.e).sum();
    assert!((pulled + 0.4).abs() < 1e-12);
    // The filament never moves faster than the retraction speed: 0.4 mm over 1 mm at v ≤ 75 mm/s.
    assert!(wipes.iter().all(|w| w.speed * 0.4 / 1.0 <= p.retract_speed + 1e-9));
    // And the print ends lifted off the part, before the machine's own end.
    let end = program.passes[1].moves.last().unwrap();
    assert!((end.to.unwrap()[2] - 0.8).abs() < 1e-9, "{end:?}");
}

#[test]
fn loops_stop_short_of_their_seam_and_bridges_lay_round_strands() {
    let square = Path {
        role: Role::OUTER_WALL,
        width: 0.4,
        closed: true,
        points: vec![[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]],
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let bridge = Path {
        role: Role::BRIDGE,
        width: 0.4,
        closed: false,
        points: vec![[10.0, 10.0], [20.0, 10.0]],
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let paths = Paths {
        carrier: Default::default(),
        passes: vec![
            PathPass {
                index: 0,
                top: 0.2,
                height: 0.2,
                warp: Default::default(),
                paths: vec![square.clone()],
                material: 0,
            },
            PathPass {
                index: 1,
                top: 0.4,
                height: 0.2,
                warp: Default::default(),
                paths: vec![square, bridge],
                material: 0,
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
    let summary = summary_of(&program, &p);
    assert!((summary.filament_mm - expected).abs() < 1e-9, "{}", summary.filament_mm);
}

#[test]
fn a_short_move_slides_and_a_long_one_retracts_and_lifts() {
    let line = |a: [f64; 2], b: [f64; 2]| Path::new(Role::SOLID, 0.4, false, vec![a, b]);
    let between = |second: Path| {
        let paths = Paths {
            carrier: Default::default(),
            passes: vec![PathPass {
                index: 1,
                top: 0.2,
                height: 0.2,
                warp: Default::default(),
                paths: vec![line([10.0, 10.0], [20.0, 10.0]), second],
                material: 0,
            }],
        };
        let program = compiled(&paths, &params());
        let moves = &program.passes[0].moves;
        let after_first = moves.iter().position(|m| m.bead.is_some()).unwrap();
        let between: Vec<Move> = moves[after_first..]
            .iter()
            .skip_while(|m| m.bead.is_some())
            .take_while(|m| m.bead.is_none())
            .cloned()
            .collect();
        between
    };
    let short = between(line([20.0, 10.5], [10.0, 10.5]));
    assert!(
        short.iter().all(|m| m.e == 0.0 && m.arc.is_none() && m.to.is_some_and(|t| t[2] == 0.2)),
        "{short:?}"
    );
    // However well the layer above would hide it: a string is drawn over whatever lies below.
    let long = between(line([20.0, 15.0], [10.0, 15.0]));
    assert!(long.iter().any(|m| m.e < 0.0), "{long:?}");
    assert!(long.iter().any(|m| m.to.is_some_and(|t| t[2] > 0.2)), "{long:?}");
}

#[test]
fn lifts_turn_a_helix_where_the_nozzle_is_known() {
    let line = |y: f64| Path {
        role: Role::SPARSE,
        width: 0.4,
        closed: false,
        points: vec![[10.0, y], [20.0, y]],
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let paths = Paths {
        carrier: Default::default(),
        passes: vec![PathPass {
            index: 0,
            top: 0.2,
            height: 0.2,
            warp: Default::default(),
            paths: vec![line(10.0), line(30.0)],
            material: 0,
        }],
    };
    let mut p = params();
    p.lift_radius = 1.0;
    p.wipe_distance = 1.0;
    let program = compiled(&paths, &p);
    let lifts: Vec<&Move> =
        chained(&program.passes[0].moves, None).filter(|(f, m)| climbs(*f, m)).map(|(_, m)| m).collect();
    // Off the first line, then off the second at the end; from the start, the nozzle crossed over.
    assert_eq!(lifts.len(), 2, "{lifts:?}");
    // Off the first line, wiped back from (20, 10) to (19, 10), setting off towards the second
    // line's start at (10, 30): the nozzle turns clockwise from its wipe (-x) to that heading, and
    // so does the helix, its axis a radius right of the heading.
    let h = [-9.0 / 481f64.sqrt(), 20.0 / 481f64.sqrt()];
    let arc = lifts[0].arc.unwrap();
    assert_eq!(arc.turn, -std::f64::consts::TAU);
    assert!((arc.centre[0] - (19.0 + h[1])).abs() < 1e-12 && (arc.centre[1] - (10.0 - h[0])).abs() < 1e-12);
    // Flown at the travel speed, the Z axis below its limit: 20 mm/s · √((2π)² + 0.4²) / 0.4 > 300.
    assert_eq!(lifts[0].speed, 300.0);
    // The print ends climbing off the wipe's end at (19, 30), setting off the way it wiped, -x.
    assert_eq!(lifts[1].arc.unwrap().centre, [19.0, 29.0]);
}

mod bent;
mod cool;
mod fit;
mod frame;
mod lifts;
mod merge;

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
    let program = compile(&line(10.0), std::slice::from_ref(&field)).unwrap();
    let lifts: Vec<&Move> = program
        .passes
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
    let program = compile(&line(10.0), std::slice::from_ref(&field)).unwrap();
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
    let before = summary_of(&program, &p);
    for m in program.passes.iter_mut().flat_map(|l| &mut l.moves) {
        if m.bead.is_some() {
            m.e *= 2.0;
            m.speed /= 2.0;
        }
    }
    let after = summary_of(&program, &p);
    assert!((after.filament_mm - 2.0 * before.filament_mm).abs() < 1e-12);
    assert!(after.motion_seconds > before.motion_seconds);
}

#[test]
fn moves_are_held_to_what_the_hot_end_melts_and_the_z_axis_moves() {
    let bead = Some(Bead { role: Role::SOLID, width: 0.4 });
    let filament = Filament { diameter: 1.75, max_volumetric_speed: 24.0 };
    // 1 mm pushing 1 mm of 1.75 mm filament, 2.405 mm³, asks 240 mm³/s at 100 mm/s: held to 24 mm³/s.
    // A travel down 3 mm over 4 across, 5 long, at 300 mm/s drives Z at 180 mm/s: held to 20.
    let mut program = single(vec![
        travel([0.0; 3], 300.0),
        Move { e: 1.0, bead, ..travel([1.0, 0.0, 0.0], 100.0) },
        travel([5.0, 0.0, -3.0], 300.0),
    ]);
    assert_eq!(limit(&mut program, &[filament], 20.0), 1);
    let area = std::f64::consts::PI * 0.875 * 0.875;
    assert!((program.passes[0].moves[1].speed - 24.0 / area).abs() < 1e-9);
    assert!((program.passes[0].moves[2].speed - 20.0 * 5.0 / 3.0).abs() < 1e-9);
    // Held already: left alone.
    let held = program.clone();
    assert_eq!(limit(&mut program, &[filament], 20.0), 0);
    assert_eq!(program, held);
}

#[test]
fn a_bead_slows_by_how_far_it_hangs() {
    // Four segments of an outer wall, on solid, a third over air, three quarters, wholly over;
    // then a bridge's strand, wholly over air, which crosses as slowly as that last segment
    // although the bridge's own speed is 50 mm/s.
    let wall = Path {
        role: Role::OUTER_WALL,
        width: 0.4,
        closed: false,
        points: vec![[10.0, 10.0], [12.0, 10.0], [14.0, 10.0], [16.0, 10.0], [18.0, 10.0]],
        travel_crosses: false,
        overhang: vec![0.0, 0.3, 0.75, 1.0],
    };
    let strand = Path::new(Role::BRIDGE, 0.4, false, vec![[18.0, 10.4], [10.0, 10.4]]);
    let paths = Paths {
        carrier: Default::default(),
        passes: vec![PathPass {
            index: 1,
            top: 0.4,
            height: 0.2,
            warp: Default::default(),
            paths: vec![wall, strand],
            material: 0,
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
    let expected = [100.0, 50.0, 30.0, 5.0, 5.0];
    assert_eq!(speeds.len(), expected.len(), "{speeds:?}");
    assert!(speeds.iter().zip(expected).all(|(s, e)| (s - e).abs() < 1e-9), "{speeds:?}");
    assert_eq!(fans, [0.2, 0.2, 0.5, 0.5, 0.5]);
}

#[test]
fn the_pass_after_a_filament_change_goes_without_the_fan() {
    // Layer 2 in two materials, layer 3 in the second: the second material's first pass lies on
    // plastic the change left to cool, and melts into it without the fan; the next layer has it.
    let pass = |index: usize, material: usize, y: f64| PathPass {
        index,
        top: 0.2 * (index + 1) as f64,
        height: 0.2,
        warp: Default::default(),
        paths: vec![Path::new(Role::INNER_WALL, 0.4, false, vec![[10.0, y], [20.0, y]])],
        material,
    };
    let paths = Paths {
        carrier: Default::default(),
        passes: vec![pass(2, 0, 10.0), pass(2, 1, 12.0), pass(3, 1, 14.0)],
    };
    let mut p = params();
    p.fan_speed = 0.4;
    let program = compiled(&paths, &p);
    let fans: Vec<Vec<f64>> = program
        .passes
        .iter()
        .map(|l| l.moves.iter().filter(|m| m.bead.is_some()).map(|m| m.fan).collect())
        .collect();
    assert_eq!(fans, [vec![0.4], vec![0.0], vec![0.4]]);
}

/// A short move inside the part slides along the layer, unless it crosses a laid bead: then it
/// lifts, and it pulls no filament back.
#[test]
fn a_short_move_lifts_over_a_laid_bead_it_crosses() {
    let p = params();
    let bead = |points: Vec<[f64; 2]>, crosses: bool| Path {
        role: Role::SPARSE,
        width: 0.4,
        closed: false,
        points,
        travel_crosses: crosses,
        overhang: Vec::new(),
    };
    let hop = (p.retract_min_travel / 2.0).max(0.1);
    let layer = |crosses: bool| Paths {
        carrier: Default::default(),
        passes: vec![PathPass {
            index: 0,
            top: 0.2,
            height: 0.2,
            warp: Default::default(),
            paths: vec![
                bead(vec![[10.0, 10.0], [20.0, 10.0]], false),
                bead(vec![[20.0 + hop, 10.0], [30.0, 10.0]], crosses),
            ],
            material: 0,
        }],
    };
    let highest_between = |program: &Program| {
        let moves = &program.passes[0].moves;
        let first = moves.iter().position(|m| m.e > 0.0).unwrap();
        let second = moves.iter().rposition(|m| m.e > 0.0).unwrap();
        let (mut z, mut pulled) = (0.0f64, false);
        for m in &moves[first..second] {
            z = m.to.map_or(z, |t| z.max(t[2]));
            pulled |= m.e < 0.0;
        }
        (z, pulled)
    };
    let (slid, _) = highest_between(&compiled(&layer(false), &p));
    let (lifted, pulled) = highest_between(&compiled(&layer(true), &p));
    assert!((slid - 0.2).abs() < 1e-9, "stays on the layer: {slid}");
    assert!((lifted - (0.2 + p.z_hop)).abs() < 1e-9, "over the bead by the hop: {lifted}");
    assert!(!pulled, "a hidden move keeps its filament");
}

#[test]
fn a_parameter_out_of_range_is_refused_by_name() {
    assert!(params().check().is_ok());
    let refused = |change: fn(&mut MotionParams), named: &str| {
        let mut p = params();
        change(&mut p);
        let err = compile(&line(10.0), &[Field::uniform(p)]).unwrap_err().to_string();
        assert!(err.contains(named), "{err}");
    };
    // Every role's speed and acceleration, the bridge's too, which strands take.
    refused(|p| p.bridge.speed = 0.0, "bridge");
    refused(|p| p.support_interface.accel = f64::NAN, "support interface");
    refused(|p| p.first_layer_accel = -1.0, "first layer's acceleration");
    refused(|p| p.overhang_fan_speed = 1.5, "overhang fan");
    refused(|p| p.retract_before_wipe = 2.0, "retracted before the wipe");
    refused(|p| p.wipe_distance = -1.0, "wipe");
    refused(|p| p.z_hop = 11.0, "hop");
    refused(|p| p.overhang_speeds = vec![-1.0], "overhang speeds");
}

#[test]
fn positions_are_written_to_their_unit() {
    assert_eq!(POSITION_UNIT, 10f64.powi(-(POSITION_DECIMALS as i32)));
}
