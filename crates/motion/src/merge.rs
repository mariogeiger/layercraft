//! Programs printed together on one plate: compiled, read from another slicer's G-code, or
//! both.
//!
//! The layers at one height, to the micrometre, become one layer; its pieces are printed one
//! program after another, each next the one nearest where the last ended. Each program's moves
//! expect the filament as that program left it, pulled back so far: where the piece before
//! left it otherwise, the difference is pushed or pulled in place first. A layer is as thick as
//! the step from the one below it, and bent where any of its pieces is: their warps add up on
//! the programs' carriers joined, where their triangles are the same or lie apart on the plate;
//! a program whose triangles overlap another's otherwise keeps its bends out of the plate, and
//! its beads then fail verification.

use crate::{Layer, Move, Program};
use layercraft_geom::{Carrier, Warp};

/// How far a program's filament is pulled back after each of its moves: retractions and wipes
/// pull, pushes in place give back, beads leave it as it is.
fn pulled(moves: &[Move], before: f64) -> f64 {
    moves.iter().filter(|m| m.bead.is_none()).fold(before, |p, m| p - m.e)
}

fn distance2(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

/// One layer of one program, with the filament it expects pulled before it and leaves after,
/// and where its program's corners lie on the plate's carrier, unless kept out.
struct Piece<'a> {
    layer: &'a Layer,
    before: f64,
    after: f64,
    places: Option<&'a [u32]>,
}

/// `programs` printed together; a filament difference between pieces is made up at `speed`.
pub fn merge(programs: &[Program], speed: f64) -> Program {
    // The programs' carriers joined, the corners of each where the union put them.
    let mut carrier = Carrier::default();
    let mut places: Vec<Option<Vec<u32>>> = Vec::with_capacity(programs.len());
    for program in programs {
        match carrier.union(&program.carrier) {
            Ok((joined, moved)) => {
                carrier = joined;
                places.push(Some(moved));
            }
            Err(_) => places.push(None),
        }
    }
    let mut pieces: Vec<(i64, Piece)> = Vec::new();
    for (program, places) in programs.iter().zip(&places) {
        let mut state = 0.0;
        for layer in &program.layers {
            let after = pulled(&layer.moves, state);
            let piece = Piece { layer, before: state, after, places: places.as_deref() };
            pieces.push(((layer.z * 1e6).round() as i64, piece));
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
        let mut layer = Layer {
            index: layers.len(),
            z: height,
            height: height - below,
            warp: Default::default(),
            free: false,
            moves: Vec::new(),
            material: 0,
        };
        below = height;
        let mut values: Option<Vec<f64>> = None;
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
            let warp = piece.layer.warp.values();
            if let Some(places) = piece.places.filter(|p| !warp.is_empty() && p.len() == warp.len()) {
                let sum = values.get_or_insert_with(|| vec![0.0; carrier.len()]);
                for (&to, &v) in places.iter().zip(warp) {
                    if let Some(slot) = sum.get_mut(to as usize) {
                        *slot += v;
                    }
                }
            }
            state = piece.after;
            at = piece.layer.moves.iter().rev().find_map(|m| m.to).or(at);
        }
        layer.warp = values.map_or(Ok(Warp::default()), Warp::new).unwrap_or_default();
        layers.push(layer);
    }
    Program { carrier, layers }
}
