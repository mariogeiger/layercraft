//! Beads written as the fewest lines and arcs within the resolution, each laying the plastic of
//! the moves it replaces.

use super::*;

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
        travel_crosses: false,
        overhang: Vec::new(),
    };
    let paths = Paths {
        carrier: Default::default(),
        passes: vec![PathPass {
            index: 1,
            top: 0.4,
            height: 0.2,
            warp: Default::default(),
            paths: vec![wall],
            material: 0,
        }],
    };
    let traced = compiled(&paths, &params());
    let beads = |program: &Program| -> Vec<(Option<[f64; 3]>, Move)> {
        chained(&program.passes[0].moves, None)
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
    let moves = &program.passes[0].moves;
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
    let moves = &program.passes[0].moves;
    assert!(moves.len() > 2 && moves.iter().all(|m| m.arc.is_none()), "{moves:?}");
}
