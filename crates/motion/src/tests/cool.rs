//! Where a layer too short to cool waits: clear of its beads, the same turned, moved or mirrored.

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
        travel_inside: false,
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
    let mut program = compiled(&Paths { layers: vec![layer(0), layer(1)] }, &p);
    let n = program.layers[0].moves.len();
    cool(&mut program, &Field::uniform(p), PLATE).unwrap();
    let to = program.layers[0].moves[n..].iter().find_map(|m| m.to).unwrap();
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
