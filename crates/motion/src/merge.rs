//! Programs printed together on one plate: compiled, read from another slicer's G-code, or
//! both.
//!
//! The layers at one height, to the micrometre, become one layer; its pieces are printed one
//! program after another, each next the one nearest where the last ended. Each program's moves
//! expect the filament as that program left it, pulled back so far: where the piece before
//! left it otherwise, the difference is pushed or pulled in place first. A layer is as thick as
//! the step from the one below it.

use crate::{Layer, Move, Program};

/// How far a program's filament is pulled back after each of its moves: retractions and wipes
/// pull, pushes in place give back, beads leave it as it is.
fn pulled(moves: &[Move], before: f64) -> f64 {
    moves.iter().filter(|m| m.bead.is_none()).fold(before, |p, m| p - m.e)
}

fn distance2(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

/// One layer of one program, with the filament it expects pulled before it and leaves after.
struct Piece<'a> {
    layer: &'a Layer,
    before: f64,
    after: f64,
}

/// `programs` printed together; a filament difference between pieces is made up at `speed`.
pub fn merge(programs: &[Program], speed: f64) -> Program {
    let mut pieces: Vec<(i64, Piece)> = Vec::new();
    for program in programs {
        let mut state = 0.0;
        for layer in &program.layers {
            let after = pulled(&layer.moves, state);
            pieces.push(((layer.z * 1e6).round() as i64, Piece { layer, before: state, after }));
            state = after;
        }
    }
    // Stable: at one height, the programs in the order given.
    pieces.sort_by_key(|(z, _)| *z);
    let mut layers: Vec<Layer> = Vec::new();
    let (mut at, mut state, mut below) = (None::<[f64; 3]>, 0.0, 0.0);
    let mut rest = pieces.as_slice();
    while let Some((z, _)) = rest.first() {
        let count = rest.iter().take_while(|(k, _)| k == z).count();
        let (here, next) = rest.split_at(count);
        rest = next;
        let mut left: Vec<&Piece> = here.iter().map(|(_, p)| p).collect();
        let height = left.first().map_or(0.0, |p| p.layer.z);
        let mut layer = Layer { index: layers.len(), z: height, height: height - below, moves: Vec::new() };
        below = height;
        while !left.is_empty() {
            let start = |p: &Piece| p.layer.moves.iter().find_map(|m| m.to);
            let pick = match at {
                Some(a) => (0..left.len())
                    .min_by(|i, j| {
                        let d = |k: &usize| {
                            left.get(*k).and_then(|p| start(p)).map_or(f64::MAX, |s| distance2(a, s))
                        };
                        d(i).total_cmp(&d(j))
                    })
                    .unwrap_or(0),
                None => 0,
            };
            let piece = left.remove(pick.min(left.len().saturating_sub(1)));
            if (state - piece.before).abs() > 1e-9
                && let Some(first) = piece.layer.moves.first()
            {
                layer.moves.push(Move {
                    to: None,
                    arc: None,
                    e: state - piece.before,
                    speed,
                    bead: None,
                    wait: 0.0,
                    ..first.clone()
                });
            }
            layer.moves.extend(piece.layer.moves.iter().cloned());
            state = piece.after;
            at = piece.layer.moves.iter().rev().find_map(|m| m.to).or(at);
        }
        layers.push(layer);
    }
    Program { layers }
}
