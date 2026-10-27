//! The viscosity of the plastic at each temperature, by the Vogel–Fulcher–Tammann law, through
//! its glass transition and its melt at the nozzle.

use crate::Physics;

/// Below the glass transition by this, in K, the viscosity diverges: the universal constant
/// `C₂` of Williams, Landel and Ferry.
const WLF_C2: f64 = 51.6;
/// The viscosity at the glass transition, in Pa·s: what the transition is, by definition.
const GLASS_VISCOSITY: f64 = 1e12;

/// `η(T) = η∞·exp(B/(T − T₀))`.
#[derive(Debug, Clone, Copy)]
pub struct Viscosity {
    ln_infinite: f64,
    b: f64,
    t0: f64,
}

impl Viscosity {
    /// The law through `η(T_g) = 10¹² Pa·s` and the melt's viscosity at the nozzle.
    pub fn of(p: &Physics) -> Self {
        let t0 = p.glass_temperature - WLF_C2;
        let (x_glass, x_melt) = (1.0 / (p.glass_temperature - t0), 1.0 / (p.melt_temperature - t0));
        let b = (GLASS_VISCOSITY.ln() - p.melt_viscosity.ln()) / (x_glass - x_melt);
        Viscosity { ln_infinite: p.melt_viscosity.ln() - b * x_melt, b, t0 }
    }

    /// In Pa·s, at `t` °C; past the divergence, glass.
    pub fn at(&self, t: f64) -> f64 {
        let above = t - self.t0;
        if above <= 0.0 { f64::INFINITY } else { (self.ln_infinite + self.b / above).exp() }
    }
}
