//! Programs printed together on one plate: compiled, read from another slicer's G-code, or both.
//!
//! The passes at one height, to the micrometre, become one layer, printed in passes of one
//! material, on the layer's top or free: material by material, the filament loaded first, each
//! material's beads on the top before its free strands; within a pass, one program's piece after
//! another, each next the one nearest where the last ended. Each program's moves expect the
//! filament as that program left it, pulled back so far: where the piece before left it otherwise,
//! the difference is pushed or pulled in place first. A layer is as thick as the step from the one
//! below it, and bent where any of its pieces is: their warps add up on the programs' carriers
//! joined, where their triangles are the same or lie apart on the plate; a program whose triangles
//! overlap another's otherwise keeps its bends out of the plate, and its beads then fail
//! verification.

use crate::moves::{Point3, end};
use crate::{MotionError, Move, Pass, Program};
use layercraft_geom::{Carrier, Warp};

/// How far a program's filament is pulled back after each of its moves: retractions and wipes
/// pull, pushes in place give back, beads leave it as it is.
fn pulled(moves: &[Move], before: f64) -> f64 {
    moves.iter().filter(|m| m.bead.is_none()).fold(before, |p, m| p - m.e)
}

fn distance2(a: Point3, b: Point3) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

/// Micrometres in a millimetre: passes whose heights round to the same micrometre are printed
/// together.
const MICROMETRES: f64 = 1e6;

/// A pass's height in micrometres, which the passes printed together share.
fn z_key(z: f64) -> i64 {
    (z * MICROMETRES).round() as i64
}

/// One pass of one program, with the filament it expects pulled before it and leaves after,
/// and where its program's corners lie on the plate's carrier, unless kept out.
struct Piece<'a> {
    pass: &'a Pass,
    before: f64,
    after: f64,
    places: Option<&'a [u32]>,
}

/// `programs` printed together; a filament difference between pieces is made up at `speed`.
pub fn merge(programs: &[Program], speed: f64) -> Result<Program, MotionError> {
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
    let mut pieces: Vec<Piece> = Vec::new();
    for (program, places) in programs.iter().zip(&places) {
        let mut state = 0.0;
        for pass in &program.passes {
            let after = pulled(&pass.moves, state);
            pieces.push(Piece { pass, before: state, after, places: places.as_deref() });
            state = after;
        }
    }
    // Stable: at one height, the programs in the order given.
    pieces.sort_by_key(|p| z_key(p.pass.z));
    let mut passes: Vec<Pass> = Vec::new();
    let (mut at, mut state, mut below, mut loaded) = (None::<Point3>, 0.0, 0.0, None::<usize>);
    for (index, here) in pieces.chunk_by(|a, b| z_key(a.pass.z) == z_key(b.pass.z)).enumerate() {
        let z = here.first().map_or(0.0, |p| p.pass.z);
        let warp = summed_warp(here, carrier.len())?;
        // The passes at this height, the filament loaded first, then by material, laid before free.
        let mut kinds: Vec<(usize, bool)> = here.iter().map(|p| (p.pass.material, p.pass.free)).collect();
        kinds.sort_by_key(|&(material, free)| (Some(material) != loaded, material, free));
        kinds.dedup();
        for (material, free) in kinds {
            let mut left: Vec<&Piece> =
                here.iter().filter(|p| (p.pass.material, p.pass.free) == (material, free)).collect();
            let mut pass =
                Pass { index, z, height: z - below, material, warp: warp.clone(), free, moves: Vec::new() };
            while !left.is_empty() {
                let start = |k: usize| left.get(k).and_then(|p| p.pass.moves.iter().find_map(|m| m.to));
                let near = |k: usize| at.zip(start(k)).map_or(f64::MAX, |(a, s)| distance2(a, s));
                let pick = (0..left.len()).min_by(|&i, &j| near(i).total_cmp(&near(j))).unwrap_or(0);
                let piece = left.remove(pick);
                if (state - piece.before).abs() > 1e-9
                    && let Some(first) = piece.pass.moves.first()
                {
                    pass.moves.push(Move {
                        to: None,
                        arc: None,
                        e: state - piece.before,
                        speed,
                        bead: None,
                        wait: 0.0,
                        ..first.clone()
                    });
                }
                pass.moves.extend(piece.pass.moves.iter().cloned());
                state = piece.after;
                at = end(&piece.pass.moves, at);
            }
            loaded = Some(material);
            passes.push(pass);
        }
        below = z;
    }
    Ok(Program { carrier, passes })
}

/// The warp of a layer printed from `pieces`: their warps added on the joined carrier of `corners`
/// corners, each where its program's corners lie on it; flat where none bends.
fn summed_warp(pieces: &[Piece], corners: usize) -> Result<Warp, MotionError> {
    let mut values: Option<Vec<f64>> = None;
    for piece in pieces {
        let warp = piece.pass.warp.values();
        if let Some(places) = piece.places.filter(|p| !warp.is_empty() && p.len() == warp.len()) {
            let sum = values.get_or_insert_with(|| vec![0.0; corners]);
            for (&to, &v) in places.iter().zip(warp) {
                if let Some(slot) = sum.get_mut(to as usize) {
                    *slot += v;
                }
            }
        }
    }
    Ok(values.map(Warp::new).transpose()?.unwrap_or_default())
}
