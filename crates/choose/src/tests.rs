//! Choosing by the objective, and the geometry it is costed by.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;
use layercraft_cost::Quanta;

#[test]
fn each_piece_takes_its_cheapest_way() {
    let q = Quanta::new(0.01);
    // Three pieces; the second way of piece 0, the first of 1, the third of 2 are best.
    let table = [[2.0, 1.0, 3.0], [0.5, 0.7, 0.9], [4.0, 5.0, 1.0]];
    let (chosen, cost) = choose_fills(&[3, 3, 3], |p, a| q.cost(0.0, table[p][a], 1.0, 0.0)).unwrap();
    assert_eq!(chosen, vec![1, 0, 2]);
    assert_eq!(cost, q.cost(0.0, 2.5, 3.0, 0.0));
}

#[test]
fn hatch_lines_leave_their_void_at_the_ends() {
    // A 10 × 4 mm rectangle hatched along x at the spacing: lines full length, so the void is
    // the strip their round ends leave, and nothing is pushed outside.
    let q = Quanta::new(0.001);
    let piece: layercraft_geom::Region = vec![vec![vec![[0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [0.0, 4.0]]]];
    let s = 0.4;
    let lines: Vec<Vec<[f64; 2]>> =
        (0..10).map(|k| vec![[0.0, s / 2.0 + s * k as f64], [10.0, s / 2.0 + s * k as f64]]).collect();
    let cost = quality::lines_cost(&piece, &lines, s, 0.2, &q).unwrap();
    // At each end, the cap's disc of radius r = s/2, centred c = π·s/8 inside: of the strip
    // [0, c] × s it covers the half disc less the segment beyond the edge, which it pushes out.
    let (r, c) = (s / 2.0, std::f64::consts::PI * s / 8.0);
    let beyond = r * r * (c / r).acos() - c * (r * r - c * c).sqrt();
    let covered = std::f64::consts::PI * r * r / 2.0 - beyond;
    let expected = 2.0 * 10.0 * ((c * s - covered) + beyond);
    let interior = cost.interior as f64 * 1e-9 / 0.2;
    assert!((interior - expected).abs() < 0.02 * expected + 1e-3, "{interior} for {expected}");
}
