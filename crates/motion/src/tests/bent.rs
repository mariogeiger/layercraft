//! Beads on a bent layer: on its top, laying the exact plastic of the slab under them, written
//! as few moves each within the resolution of what it replaces, and travels hopping over the
//! layer's highest point on their way.

use super::*;
use layercraft_geom::Grid;
use std::f64::consts::{FRAC_PI_4, PI};

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
