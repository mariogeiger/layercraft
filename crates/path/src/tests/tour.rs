//! The order of a layer's paths: the fewest moves that cross a laid bead, then the least
//! travel, every reversal of a run and every move of a short run elsewhere tried and found no
//! better.

use super::*;
use crate::crossing::{Cover, Laid};
use crate::tour::tour;
use std::collections::HashMap;

fn dot(x: f64) -> Path {
    Path::new(Role::SOLID, 0.4, false, vec![[x, 0.0]])
}

/// The edges of the rectangles from `lo` to `hi`, as laid beads.
fn rings(squares: &[(Point, Point)]) -> Laid {
    let mut all = Vec::new();
    for &(lo, hi) in squares {
        let c = [lo, [hi[0], lo[1]], hi, [lo[0], hi[1]], lo];
        all.extend(c.windows(2).map(|w| (w[0], w[1], 0)));
    }
    Laid::new(&all)
}

#[test]
fn a_tour_moves_a_path_rather_than_zigzag() {
    // Nearest first goes 1, −2, 4, −8: 22 mm; the least, one way or another, is 16.
    let laid = rings(&[]);
    let (mut at, mut out) = ([0.0, 0.0], Vec::new());
    tour([1.0, -2.0, 4.0, -8.0].map(dot).to_vec(), &mut at, &mut out, &Cover { laid: &laid });
    let travel = out
        .iter()
        .fold((0.0, 0.0), |(sum, x): (f64, f64), p| (sum + (p.points[0][0] - x).abs(), p.points[0][0]));
    assert_eq!(travel, (16.0, at[0]));
}

#[test]
fn a_tour_goes_farther_to_stay_off_a_laid_bead() {
    // A bead parts (1, 0) from (1, 1): from (0, 0), going round its end by (2, 0.5) takes 3.24 mm,
    // against 3.12 mm across it, but no lift.
    let laid = rings(&[([0.2, 0.45], [1.5, 0.55])]);
    let dots: Vec<Path> =
        [[1.0, 0.0], [1.0, 1.0], [2.0, 0.5]].map(|p| Path::new(Role::SOLID, 0.4, false, vec![p])).to_vec();
    let (mut at, mut out) = ([0.0, 0.0], Vec::new());
    tour(dots, &mut at, &mut out, &Cover { laid: &laid });
    let laid: Vec<Point> = out.iter().map(|p| p.points[0]).collect();
    assert_eq!(laid, [[1.0, 0.0], [2.0, 0.5], [1.0, 1.0]]);
}

#[test]
fn a_tour_lays_open_paths_either_way_and_loops_from_their_start() {
    // Two strokes side by side: the second laid back toward the first's end.
    let stroke = |y: f64| Path::new(Role::SOLID, 0.4, false, vec![[0.0, y], [10.0, y]]);
    let laid = rings(&[]);
    let (mut at, mut out) = ([0.0, 0.0], Vec::new());
    tour(vec![stroke(1.0), stroke(0.0)], &mut at, &mut out, &Cover { laid: &laid });
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
    // Among no beads, and around two rings of them: the tour is as short as the full search of
    // both changes leaves it.
    let (open, ringed) = (rings(&[]), rings(&[([5.0, 3.0], [9.0, 15.0]), ([12.0, 8.0], [16.0, 19.0])]));
    let covers = [Cover { laid: &ringed }, Cover { laid: &open }];
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

/// A tour takes the pieces on each side of a laid bead in runs, crossing it as seldom as it can.
#[test]
fn a_tour_crosses_a_laid_bead_as_seldom_as_it_can() {
    let lines: Vec<Path> = (0..12)
        .flat_map(|k| {
            let x = k as f64;
            [
                Path::new(Role::SPARSE, 0.4, false, vec![[x, 0.0], [x, 4.0]]),
                Path::new(Role::SPARSE, 0.4, false, vec![[x, 6.0], [x, 10.0]]),
            ]
        })
        .collect();
    let wall = Laid::new(&[([-1.0, 5.0], [13.0, 5.0], 0)]);
    let mut out = Vec::new();
    let mut at = [0.0, 0.0];
    tour(lines, &mut at, &mut out, &Cover { laid: &wall });
    let crossings = out.windows(2).filter(|w| wall.crossed(w[0].end().unwrap(), w[1].points[0], 1)).count();
    assert_eq!(crossings, 1);
}
