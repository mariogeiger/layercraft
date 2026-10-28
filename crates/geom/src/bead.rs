//! The cross-section of a bead: what a nozzle lays down `w` wide on a layer `h` high.
//!
//! Squeezed between the nozzle and the layer below, the plastic takes a rectangle with a
//! half-disc on each side: area `(w − h)·h + π·h²/4`. Beads tile a surface without voids or
//! overlaps when their centres are that area over `h` apart, `w − h·(1 − π/4)`, a little closer
//! than their width, because the rounded sides leave room. A strand, hanging round from the
//! nozzle, is a disc.

/// The cross-section of a bead `width` wide on a layer `height` high, in mm².
pub fn bead_area(width: f64, height: f64) -> f64 {
    (width - height) * height + std::f64::consts::PI * height * height / 4.0
}

/// The distance between the centres of side-by-side beads that fill their layer exactly, in mm.
pub fn bead_spacing(width: f64, height: f64) -> f64 {
    width - height * (1.0 - std::f64::consts::FRAC_PI_4)
}

/// The area of a disc `diameter` across, in mm²: a strand's section, or the filament's.
pub fn disc_area(diameter: f64) -> f64 {
    std::f64::consts::PI * (diameter / 2.0).powi(2)
}
