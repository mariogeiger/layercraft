//! The order of a layer's paths: the fewest moves that leave what hides them, then the least
//! travel, every reversal of a run and every move of a short run elsewhere tried and found no
//! better.

use super::*;
use crate::link::{Cover, Fill};
use crate::tour::tour;
use layercraft_geom::{Bands, difference, union_contours};
use std::collections::HashMap;

fn dot(x: f64) -> Path {
    Path::new(Role::SOLID, 0.4, false, vec![[x, 0.0]])
}

fn square(lo: Point, hi: Point) -> Vec<Point> {
    vec![lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]]
}

/// A cover that hides every move between points of the square from `lo` to `hi`.
fn everywhere(lo: Point, hi: Point) -> Bands {
    Bands::new(&union_contours(&[square(lo, hi)]).unwrap())
}

#[test]
fn a_tour_moves_a_path_rather_than_zigzag() {
    // Nearest first goes 1, −2, 4, −8: 22 mm; the least, one way or another, is 16.
    let hidden = everywhere([-10.0, -1.0], [10.0, 1.0]);
    let (mut at, mut out) = ([0.0, 0.0], Vec::new());
    tour([1.0, -2.0, 4.0, -8.0].map(dot).to_vec(), &mut at, &mut out, &Cover { hidden: &hidden, join: None });
    let travel = out
        .iter()
        .fold((0.0, 0.0), |(sum, x): (f64, f64), p| (sum + (p.points[0][0] - x).abs(), p.points[0][0]));
    assert_eq!(travel, (16.0, at[0]));
}

#[test]
fn a_tour_goes_farther_to_keep_its_moves_hidden() {
    // A slit across the hidden square parts (1, 0) from (1, 1): from (0, 0), going round its end
    // by (2, 0.5) takes 3.24 mm, against 3.12 mm across it, but no retraction.
    let slit = union_contours(&[square([0.2, 0.45], [1.5, 0.55])]).unwrap();
    let region = difference(&union_contours(&[square([-5.0, -5.0], [5.0, 5.0])]).unwrap(), &slit).unwrap();
    let hidden = Bands::new(&region);
    let dots: Vec<Path> =
        [[1.0, 0.0], [1.0, 1.0], [2.0, 0.5]].map(|p| Path::new(Role::SOLID, 0.4, false, vec![p])).to_vec();
    let (mut at, mut out) = ([0.0, 0.0], Vec::new());
    tour(dots, &mut at, &mut out, &Cover { hidden: &hidden, join: None });
    let laid: Vec<Point> = out.iter().map(|p| p.points[0]).collect();
    assert_eq!(laid, [[1.0, 0.0], [2.0, 0.5], [1.0, 1.0]]);
}

#[test]
fn a_tour_lays_open_paths_either_way_and_loops_from_their_start() {
    // Two strokes side by side: the second laid back toward the first's end.
    let stroke = |y: f64| Path::new(Role::SOLID, 0.4, false, vec![[0.0, y], [10.0, y]]);
    let hidden = everywhere([-1.0, -1.0], [11.0, 2.0]);
    let (mut at, mut out) = ([0.0, 0.0], Vec::new());
    tour(vec![stroke(1.0), stroke(0.0)], &mut at, &mut out, &Cover { hidden: &hidden, join: None });
    assert_eq!(out[0].points, [[0.0, 0.0], [10.0, 0.0]]);
    assert_eq!(out[1].points, [[10.0, 1.0], [0.0, 1.0]]);
    assert_eq!(at, [0.0, 1.0]);
    let mut ring = Path::new(Role::OUTER_WALL, 0.4, true, vec![[5.0, 0.0], [6.0, 1.0], [5.0, 1.0]]);
    ring.reverse();
    assert_eq!(ring.points, [[5.0, 0.0], [6.0, 1.0], [5.0, 1.0]]);
}

/// A number in [0, 1) from a xorshift state.
fn uniform(state: &mut u64) -> f64 {
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    (*state >> 11) as f64 / (1u64 << 53) as f64
}

/// `count` paths over a 20 mm square: strokes of two or three points, small loops and dots.
fn scattered(count: usize, state: &mut u64) -> Vec<Path> {
    (0..count)
        .map(|_| {
            let (x, y, kind) = (20.0 * uniform(state), 20.0 * uniform(state), uniform(state));
            let (s, c) = (std::f64::consts::TAU * uniform(state)).sin_cos();
            let l = 0.5 + 3.5 * uniform(state);
            if kind < 0.6 {
                let bend =
                    [x + l * c - 0.5 * l * s * uniform(state), y + l * s + 0.5 * l * c * uniform(state)];
                Path::new(Role::SOLID, 0.4, false, vec![[x, y], [x + 0.5 * l * c, y + 0.5 * l * s], bend])
            } else if kind < 0.8 {
                Path::new(
                    Role::INNER_WALL,
                    0.4,
                    true,
                    vec![[x, y], [x + l * c, y + l * s], [x - l * s, y + l * c]],
                )
            } else {
                Path::new(Role::SOLID, 0.4, false, vec![[x, y]])
            }
        })
        .collect()
}

/// What laying `paths` in order from `from` costs: the moves that leave `cover`, then the
/// length of all the moves.
fn cost_of(paths: &[Path], from: Point, stays: &mut dyn FnMut(Point, Point) -> bool) -> (usize, f64) {
    let (mut at, mut leaves, mut length) = (from, 0, 0.0);
    for path in paths {
        let start = path.points[0];
        leaves += usize::from(!stays(at, start));
        length += (start[0] - at[0]).hypot(start[1] - at[1]);
        at = path.end().unwrap();
    }
    (leaves, length)
}

/// Asserts that no reversal of a run of `paths`, and no move of a run of up to three of them
/// elsewhere either way round, lays them from `from` with fewer moves leaving `cover`, or as
/// few and less travel.
fn assert_no_change_shortens(paths: &[Path], from: Point, cover: &Cover) {
    let mut known: HashMap<[u64; 4], bool> = HashMap::new();
    let mut stays = |a: Point, b: Point| {
        *known
            .entry([a[0].to_bits(), a[1].to_bits(), b[0].to_bits(), b[1].to_bits()])
            .or_insert_with(|| cover.stays(a, b))
    };
    let now = cost_of(paths, from, &mut stays);
    let below = |c: (usize, f64)| c.0 < now.0 || (c.0 == now.0 && c.1 < now.1 - 1e-9);
    let n = paths.len();
    for i in 0..n {
        for j in i..n {
            let mut changed = paths.to_vec();
            changed[i..=j].reverse();
            changed[i..=j].iter_mut().for_each(Path::reverse);
            let cost = cost_of(&changed, from, &mut stays);
            assert!(!below(cost), "reversing {i}..={j}: {cost:?} against {now:?}");
        }
    }
    for len in 1..=3.min(n) {
        for k in 0..=n - len {
            for t in 0..=n - len {
                for turned in [false, true] {
                    let mut changed = paths.to_vec();
                    let mut run: Vec<Path> = changed.drain(k..k + len).collect();
                    if turned {
                        run.reverse();
                        run.iter_mut().for_each(Path::reverse);
                    }
                    changed.splice(t..t, run);
                    let cost = cost_of(&changed, from, &mut stays);
                    assert!(
                        !below(cost),
                        "moving {k}..{} to {t}, turned {turned}: {cost:?} against {now:?}",
                        k + len
                    );
                }
            }
        }
    }
}

#[test]
fn no_reversal_or_move_of_a_run_shortens_a_tour() {
    // Over a hidden square with two holes, and over lines joined only where short inside an
    // area: the tour is as short as the full search of both changes leaves it.
    let outer = union_contours(&[square([-1.0, -1.0], [21.0, 21.0])]).unwrap();
    let holes =
        union_contours(&[square([5.0, 3.0], [9.0, 15.0]), square([12.0, 8.0], [16.0, 19.0])]).unwrap();
    let holed = Bands::new(&difference(&outer, &holes).unwrap());
    let corner = Bands::new(&union_contours(&[square([0.0, 0.0], [6.0, 6.0])]).unwrap());
    let fill = Fill {
        role: Role::SOLID,
        reach: difference(&outer, &holes).unwrap(),
        max_link: 2.0,
        monotone: false,
    };
    let reach = Bands::new(&fill.reach);
    let covers =
        [Cover { hidden: &holed, join: None }, Cover { hidden: &corner, join: Some((&fill, &reach)) }];
    let mut state = 0x2545_f491_4f6c_dd1d;
    for round in 0..40 {
        let cover = &covers[round % 2];
        let paths = scattered(12 + round % 30, &mut state);
        let from = [20.0 * uniform(&mut state), 20.0 * uniform(&mut state)];
        let (mut at, mut out) = (from, Vec::new());
        tour(paths.clone(), &mut at, &mut out, cover);
        assert_eq!(out.len(), paths.len());
        for path in &paths {
            let mut back = path.clone();
            back.reverse();
            assert!(out.iter().any(|p| p == path || *p == back), "round {round}: a path is lost");
        }
        assert_eq!(Some(at), out.last().and_then(Path::end));
        assert_no_change_shortens(&out, from, cover);
    }
}
