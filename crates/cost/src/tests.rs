//! The order of costs: quality first, within the printer's quanta, then filament, then time.
#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use super::*;

const Q: Quanta = Quanta { length: 0.01 };

#[test]
fn quality_comes_first_then_filament_then_time() {
    // A hundredth of a millimetre closer to the solid beats a gram less and an hour less.
    let close = Q.cost(0.0, 0.0, 9.0e3, 7200.0);
    let far = Q.cost(0.011, 0.0, 1.0e3, 60.0);
    assert!(close < far);
    // Within one resolution, the surfaces print alike: the filament decides.
    assert!(Q.cost(0.009, 0.0, 1.0e3, 7200.0) < Q.cost(0.0, 0.0, 2.0e3, 60.0));
    // Same quality, same filament to its quantum: the time decides.
    assert!(Q.cost(0.0, 0.0, 1.0e3, 60.0) < Q.cost(0.0, 0.0, 1.0e3 + 1e-7, 61.0));
    // The interior, in resolutions cubed, comes before the filament.
    assert!(Q.cost(0.0, 2e-6, 9.0e3, 60.0) < Q.cost(0.0, 3e-6, 1.0e3, 60.0));
}

#[test]
fn parts_combine_by_the_worst_surface_and_the_summed_rest() {
    let a = Q.cost(0.05, 1e-3, 10.0, 1.0);
    let b = Q.cost(0.02, 2e-3, 20.0, 2.0);
    let both = a.and(b);
    assert_eq!((both.surface, both.interior, both.filament), (5, 3000, 30_000_000));
    assert_eq!(both.seconds, 3.0);
    // Monotone: a cheaper part never makes the whole dearer.
    let cheaper = Q.cost(0.05, 1e-3, 10.0, 0.5);
    assert!(cheaper.and(b) < both);
    assert!(Cost::default() < Cost::INFINITE);
}
