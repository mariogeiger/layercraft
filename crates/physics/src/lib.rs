//! What the plastic does once laid: how stiff it is at each temperature, and how a print cools.
//!
//! The plastic is a liquid that stiffens as it cools: its viscosity follows the Vogel–Fulcher–
//! Tammann law `η = η∞·exp(B/(T − T₀))`, with `T₀` the universal `51.6 K` of Williams, Landel
//! and Ferry below its glass transition `T_g`, where `η` is `10¹² Pa·s` (the transition's
//! definition), and `B`, `η∞` set by its melt's viscosity at the nozzle (`viscosity`).
//! A print cools as a stack of slabs, one per layer: each holds its heat, passes it by
//! conduction to the slabs beside it and the bed, and loses it to the air from its sides and
//! the top it leaves bare, by forced convection under the fan (a laminar flat plate) or free
//! convection (`column`). A layer laid on one still soft squeezes it: under the stress that
//! flattens the new bead, of the melt's viscosity, the slab below yields as much as it is less
//! stiff, `δ = h·η(T_nozzle)/η(T)`.
//!
//! The material's constants without a key in the machine description are literature values
//! for PLA; the fan's air speed at the part is the one constant to calibrate on the printer.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod column;
pub mod viscosity;

use serde::{Deserialize, Serialize};

/// The plastic, the air about it and the machine's heat, in SI units and °C.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Physics {
    /// In kg/m³.
    pub density: f64,
    /// In J/(kg·K).
    pub specific_heat: f64,
    /// In W/(m·K).
    pub conductivity: f64,
    /// Where the plastic turns to glass, in °C.
    pub glass_temperature: f64,
    /// The nozzle's, in °C.
    pub melt_temperature: f64,
    /// The melt's viscosity at the nozzle, in Pa·s.
    pub melt_viscosity: f64,
    /// The air about the print, in °C.
    pub air_temperature: f64,
    /// The bed's, in °C.
    pub bed_temperature: f64,
    /// The air's speed at the part with the fan full on, in m/s.
    pub fan_air_speed: f64,
}

impl Default for Physics {
    /// PLA: 1240 kg/m³, 1800 J/(kg·K), 0.13 W/(m·K), glass at 60 °C, a melt of 1000 Pa·s at
    /// 220 °C; air at 25 °C, a bed at 55 °C, and 5 m/s of air from the fan, to calibrate.
    fn default() -> Self {
        Physics {
            density: 1240.0,
            specific_heat: 1800.0,
            conductivity: 0.13,
            glass_temperature: 60.0,
            melt_temperature: 220.0,
            melt_viscosity: 1000.0,
            air_temperature: 25.0,
            bed_temperature: 55.0,
            fan_air_speed: 5.0,
        }
    }
}

impl Physics {
    /// How deep, in m, a slab `h` high at `temperature` yields under the next layer.
    pub fn squeeze(&self, h: f64, temperature: f64) -> f64 {
        let law = viscosity::Viscosity::of(self);
        h * law.at(self.melt_temperature) / law.at(temperature)
    }
}

#[cfg(test)]
mod tests;
