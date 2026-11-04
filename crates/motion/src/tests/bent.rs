//! Beads on a bent layer: on its top, laying the exact plastic of the slab under them, less what
//! the face planes off, which the next layer gets back, written as few moves each within the
//! resolution of what it replaces, travels hopping over its highest point; and bent programs
//! merged on their carriers joined.

use super::*;
use layercraft_geom::Relief;
use std::f64::consts::FRAC_PI_4;

/// A ramp on a grid of millimetre squares from `x0`: rising 0.05 mm per mm out to `x0 + 10`, then
/// level at 0.5 mm out to `x0 + 20`, for y from 0 to 20.
fn ramp_from(x0: f64) -> Relief {
    let values: Vec<f64> = (0..21 * 21).map(|k| 0.05 * (k % 21).min(10) as f64).collect();
    Relief::from_grid([x0, 0.0], 1.0, [0, 0], [21, 21], &values).unwrap()
}

fn bead(points: Vec<[f64; 2]>) -> Path {
    Path { role: Role::SOLID, width: 0.4, closed: false, points, travel_crosses: false, overhang: Vec::new() }
}

/// A flat first layer, laying nothing, and over it a second bent by the ramp from x = 0, laying
/// `paths`.
fn bent(paths: Vec<Path>) -> Paths {
    let ramp = ramp_from(0.0);
    Paths {
        carrier: ramp.carrier().clone(),
        layers: vec![
            PathLayer {
                index: 0,
                top: 0.2,
                height: 0.2,
                warp: Default::default(),
                paths: Vec::new(),
                material: 0,
            },
            PathLayer { index: 1, top: 0.4, height: 0.2, warp: ramp.warp().clone(), paths, material: 0 },
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
        assert!((to[2] - (0.4 + ramp_from(0.0).at([to[0], to[1]]))).abs() < 1e-12, "{to:?} off the layer");
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

#[test]
fn bent_programs_merge_on_their_carriers_joined() {
    // A layer bent up a ramp in two programs, the second 30 mm along x: merged, the layer is bent
    // where either is, on their carriers joined. A program on triangles of its own over the
    // first's keeps its bends out.
    let program = |ramp: &Relief| Program {
        carrier: ramp.carrier().clone(),
        layers: vec![Layer {
            index: 0,
            z: 0.4,
            height: 0.4,
            warp: ramp.warp().clone(),
            free: false,
            moves: Vec::new(),
            material: 0,
        }],
    };
    let top = |merged: &Program| merged.carrier.relief(&merged.layers[0].warp).unwrap();
    let (near, far) = (ramp_from(0.0), ramp_from(30.0));
    let both = top(&merge(&[program(&near), program(&far)], 25.0));
    for p in [[5.0, 5.0], [35.0, 5.0], [12.0, 3.0], [42.0, 3.0], [25.0, 5.0]] {
        assert!((both.at(p) - (near.at(p) + far.at(p))).abs() < 1e-12, "{p:?}");
    }
    let kept = top(&merge(&[program(&near), program(&ramp_from(0.5))], 25.0));
    for p in [[5.0, 5.0], [12.3, 3.7]] {
        assert!((kept.at(p) - near.at(p)).abs() < 1e-12, "{p:?}");
    }
}
