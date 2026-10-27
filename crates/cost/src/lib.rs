//! What a print costs, the objective every choice of layercraft is ranked by.
//!
//! A plan `P` for a solid `T` prints, as its physics predicts (`layercraft_physics`), the solid
//! `S(P)`. Its quality comes first, then its filament, then its time, each compared only to what
//! the printer can tell apart: `ε`, its `resolution`, as a length, and `ε³` as a volume.
//!
//! - The surface: the farthest the printed surface strays from the solid's, `dₛ`, a Hausdorff
//!   distance, counted in whole `ε`: two surfaces within one `ε` of each other print alike.
//! - The interior: the volume where `S(P)` and `T` differ inside, voids and the plastic pushed
//!   where there is no room for it, `Vᵢ`, counted in `ε³`.
//! - The filament: the plastic pushed, `F`, in `ε³`.
//! - The time the moves take, `t`, exact: nothing comes after it.
//!
//! The key `(⌊dₛ/ε⌋, [Vᵢ/ε³], [F/ε³], t)` is compared lexicographically: any gain of a quantum
//! of quality is worth any filament and any time; filament, any time. No weight trades one
//! for the other: there is no exchange rate between a millimetre of error and a gram. Each
//! part of a plan is counted whole, rounded to its nearest quantum, and parts combine exactly:
//! the surface by the larger deviation, the rest by sums. Combining is then monotone, as an
//! extraction by parts needs: a cheaper part never makes the whole dearer.

use std::cmp::Ordering;

/// The quanta a printer tells apart: its resolution, in mm, and that cubed, in mm³.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quanta {
    pub length: f64,
}

impl Quanta {
    /// The quanta of a printer of resolution `resolution`, in mm.
    pub fn new(resolution: f64) -> Self {
        Self { length: resolution }
    }

    fn volume(&self) -> f64 {
        self.length.powi(3)
    }

    /// The cost of one part of a plan: its surface deviation `surface_mm`, its interior error
    /// `interior_mm3`, its filament `filament_mm3` and its time `seconds`.
    pub fn cost(&self, surface_mm: f64, interior_mm3: f64, filament_mm3: f64, seconds: f64) -> Cost {
        let count =
            |v: f64, q: f64| if v > 0.0 && q > 0.0 { (v / q).round().min(u64::MAX as f64) as u64 } else { 0 };
        let floor =
            |v: f64, q: f64| if v > 0.0 && q > 0.0 { (v / q).floor().min(u64::MAX as f64) as u64 } else { 0 };
        Cost {
            surface: floor(surface_mm, self.length),
            interior: count(interior_mm3, self.volume()),
            filament: count(filament_mm3, self.volume()),
            seconds: seconds.max(0.0),
        }
    }
}

/// A plan's cost, in quanta: lower is better, compared field by field in order.
#[derive(Debug, Clone, Copy, Default)]
pub struct Cost {
    /// The surface's deviation, in whole resolutions.
    pub surface: u64,
    /// The interior's error, in resolutions cubed.
    pub interior: u64,
    /// The plastic pushed, in resolutions cubed.
    pub filament: u64,
    pub seconds: f64,
}

impl Cost {
    /// The cost of two parts of one plan: the larger deviation, the summed rest.
    pub fn and(self, other: Cost) -> Cost {
        Cost {
            surface: self.surface.max(other.surface),
            interior: self.interior.saturating_add(other.interior),
            filament: self.filament.saturating_add(other.filament),
            seconds: self.seconds + other.seconds,
        }
    }

    /// A cost no plan reaches: what cannot be printed.
    pub const INFINITE: Cost =
        Cost { surface: u64::MAX, interior: u64::MAX, filament: u64::MAX, seconds: f64::INFINITY };
}

impl PartialEq for Cost {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Cost {}

impl PartialOrd for Cost {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Cost {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.surface, self.interior, self.filament)
            .cmp(&(other.surface, other.interior, other.filament))
            .then(self.seconds.total_cmp(&other.seconds))
    }
}

impl std::iter::Sum for Cost {
    fn sum<I: Iterator<Item = Cost>>(iter: I) -> Cost {
        iter.fold(Cost::default(), Cost::and)
    }
}

#[cfg(test)]
mod tests;
