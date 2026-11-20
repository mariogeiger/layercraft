//! A layer too short to cool, all its passes: slowed on its hidden beads before its surface, and
//! waiting off the part, clear of its beads, the same turned, moved or mirrored.

use super::*;
use layercraft_path::{Path, PathLayer, Paths};
use layercraft_tune::Field;

/// Where the nozzle waits after the first layer of a bead along `points`, held to its floor.
fn waiting_point(points: &[[f64; 2]]) -> [f64; 2] {
    let path = Path {
        role: Role::SPARSE,
        width: 0.4,
        closed: false,
        points: points.to_vec(),
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let layer = |index: usize| PathLayer {
        index,
        top: 0.2 * (index + 1) as f64,
        height: 0.2,
        warp: Default::default(),
        paths: vec![path.clone()],
        material: 0,
    };
    let mut p = params();
    p.slow_down_layer_time = 40.0;
    let mut program = compiled(&Paths { carrier: Default::default(), layers: vec![layer(0), layer(1)] }, &p);
    let n = program.passes[0].moves.len();
    cool(&mut program, &[Field::uniform(p)], PLATE).unwrap();
    let to = program.passes[0].moves[n..].iter().find_map(|m| m.to).unwrap();
    [to[0], to[1]]
}

#[test]
fn a_waiting_nozzle_goes_clear_of_the_beads_whichever_way_they_lie() {
    // A bent bead on a plate 256 mm across, turned 37° or mirrored and turned 110° about its
    // middle: the nozzle waits at the turned (and mirrored) place, never nearer a bead than the
    // clearance, and not a long way farther.
    let bead = [[100.0, 100.0], [130.0, 100.0], [130.0, 120.0], [110.0, 140.0]];
    let here = waiting_point(&bead);
    let from_beads = |q: [f64; 2], beads: &[[f64; 2]]| {
        beads
            .windows(2)
            .map(|w| {
                let (d, f) = ([w[1][0] - w[0][0], w[1][1] - w[0][1]], [q[0] - w[0][0], q[1] - w[0][1]]);
                let t = ((f[0] * d[0] + f[1] * d[1]) / (d[0] * d[0] + d[1] * d[1])).clamp(0.0, 1.0);
                (f[0] - t * d[0]).hypot(f[1] - t * d[1])
            })
            .fold(f64::INFINITY, f64::min)
    };
    assert!((from_beads(here, &bead) - 10.0).abs() < 1e-6, "{here:?}");
    for (angle, mirror) in [(37.0f64, false), (110.0, true)] {
        let (s, c) = angle.to_radians().sin_cos();
        let g = |p: [f64; 2]| {
            let (x, y) = (p[0] - 128.0, if mirror { 128.0 - p[1] } else { p[1] - 128.0 });
            [128.0 + c * x - s * y + 5.0, 128.0 + s * x + c * y - 3.0]
        };
        let moved: Vec<[f64; 2]> = bead.iter().map(|&p| g(p)).collect();
        let there = waiting_point(&moved);
        let want = g(here);
        assert!((there[0] - want[0]).hypot(there[1] - want[1]) < 1e-3, "{angle}°: {there:?} for {want:?}");
    }
}

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
    cool(&mut program, &[Field::uniform(p.clone())], PLATE).unwrap();
    // 10 mm at 50 mm/s, then at 100 mm/s, is 0.2 s and 0.1 s: held at the 10 mm/s floor.
    assert_eq!(speeds(&program), [10.0, 10.0]);
    // At its floor the first layer takes 1 s. Lifted off its bead's end at (20, 10), it goes
    // 10 mm past the bead the nearest way, along x, and waits the 3 s left.
    let n = before.passes[0].moves.len();
    let added = &program.passes[0].moves[n..];
    assert_eq!(added.len(), 2, "{added:?}");
    assert_eq!(
        (added[0].to, added[0].e, added[0].speed),
        (Some([30.0, 10.0, 0.6000000000000001]), 0.0, 300.0)
    );
    assert_eq!((added[1].to, added[1].e, added[1].wait), (None, 0.0, 3.0));
    // Nothing lands on the last layer: it does not wait.
    assert_eq!(program.passes[1].moves.len(), before.passes[1].moves.len());
    // Travels, wipes and lifts keep their speed.
    let others =
        |moves: &[Move]| -> Vec<Move> { moves.iter().filter(|m| m.bead.is_none()).cloned().collect() };
    assert_eq!(others(&program.passes[0].moves[..n]), others(&before.passes[0].moves));
    assert_eq!(others(&program.passes[1].moves), others(&before.passes[1].moves));
    // The wait is in the program's time.
    let (t0, t1) = (summary_of(&before, &p), summary_of(&program, &p));
    assert!(t1.motion_seconds - t0.motion_seconds > 3.0 + 2.0 * 0.9, "{t0:?} {t1:?}");
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
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let paths = Paths {
        carrier: Default::default(),
        layers: vec![PathLayer {
            index: 1,
            top: 0.4,
            height: 0.2,
            warp: Default::default(),
            paths: vec![line(Role::OUTER_WALL, 10.0), {
                let mut back = line(Role::SPARSE, 10.5);
                back.reverse();
                back
            }],
            material: 0,
        }],
    };
    for (goal, expected) in [(0.5, [100.0, 25.0]), (1.5, [20.0, 10.0])] {
        let p = MotionParams { slow_down_layer_time: goal, ..params() };
        let mut program = compiled(&paths, &p);
        cool(&mut program, &[Field::uniform(p)], PLATE).unwrap();
        let speeds: Vec<f64> = all(&program).iter().filter(|m| m.bead.is_some()).map(|m| m.speed).collect();
        assert!(speeds.iter().zip(expected).all(|(s, e)| (s - e).abs() < 1e-6), "goal {goal}: {speeds:?}");
    }
}

#[test]
fn a_layer_in_two_passes_cools_as_one_and_waits_after_its_last() {
    // Layer 0 in two materials, a 10 mm bead each at 100 mm/s, 0.2 s in all; then a layer on it.
    let bead = |y: f64| Move {
        e: 0.1,
        bead: Some(Bead { role: Role::SPARSE, width: 0.4 }),
        ..travel([20.0, y, 0.2], 100.0)
    };
    let pass = |index: usize, material: usize, y: f64| Pass {
        index,
        z: 0.2 * (index + 1) as f64,
        height: 0.2,
        warp: Default::default(),
        free: false,
        moves: vec![travel([10.0, y, 0.2 * (index + 1) as f64], 300.0), bead(y)],
        material,
    };
    let cooled = |time: f64| {
        let p = MotionParams { slow_down_layer_time: time, slow_down_min_speed: 10.0, ..params() };
        let mut program = Program {
            carrier: Default::default(),
            passes: vec![pass(0, 0, 10.0), pass(0, 1, 20.0), pass(1, 0, 10.0)],
        };
        cool(&mut program, &[Field::uniform(p.clone()), Field::uniform(p)], PLATE).unwrap();
        program
    };
    let speeds = |program: &Program| -> Vec<f64> {
        program
            .passes
            .iter()
            .take(2)
            .flat_map(|p| &p.moves)
            .filter(|m| m.bead.is_some())
            .map(|m| m.speed)
            .collect()
    };
    let waits = |program: &Program| -> Vec<f64> {
        program.passes.iter().map(|p| p.moves.iter().map(|m| m.wait).sum()).collect()
    };
    // To take 1 s, the layer's two beads go at 20 mm/s, not each pass alone at 10.
    let one = cooled(1.0);
    assert!(speeds(&one).iter().all(|v| (v - 20.0).abs() < 1e-9), "{:?}", speeds(&one));
    assert_eq!(waits(&one), [0.0, 0.0, 0.0]);
    // To take 3 s, both at their floor, 2 s, then 1 s off the part after the layer's last pass.
    let three = cooled(3.0);
    assert!(speeds(&three).iter().all(|v| (v - 10.0).abs() < 1e-9), "{:?}", speeds(&three));
    let waited = waits(&three);
    assert!(waited[0] == 0.0 && (waited[1] - 1.0).abs() < 1e-9 && waited[2] == 0.0, "{waited:?}");
}
