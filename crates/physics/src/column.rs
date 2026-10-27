//! How a print cools, as a stack of slabs, one per layer, each at one temperature.
//!
//! A slab of volume `V`, area `S = V/h` and perimeter `P` holds `ρ·c·V` per kelvin. It passes
//! heat by conduction through the area it shares with the slab below, `k·S/h̄` across their
//! mean height, the first one to the bed across half its own; it loses heat to the air from its
//! sides, `P·h`, and from the top it leaves bare, under the larger, in Churchill's way, of
//! forced convection (a laminar flat plate, `Nu = 0.664·Re^½·Pr^⅓`) and free convection
//! (`Nu = 0.54·Ra^¼`), each over the slab's breadth `√S`. While a layer is laid, its slab grows
//! as the plastic comes, at the nozzle's temperature. Each step is implicit: the stack's
//! temperatures solve one tridiagonal system, stable at any step.

use crate::Physics;

/// Air near room temperature: its conductivity in W/(m·K), kinematic viscosity and thermal
/// diffusivity in m²/s, and Prandtl number.
const AIR_CONDUCTIVITY: f64 = 0.026;
const AIR_VISCOSITY: f64 = 1.6e-5;
const AIR_DIFFUSIVITY: f64 = 2.2e-5;
const AIR_PRANDTL: f64 = 0.71;
const GRAVITY: f64 = 9.81;
const CELSIUS: f64 = 273.15;

/// The longest step of the integration, in s: a tenth of the time a slab two tenths of a
/// millimetre thick takes to pass its heat on, `ρ·c·h²/k ≈ 0.7 s`.
const STEP: f64 = 0.05;

/// One layer as it is laid: its plastic in mm³, its height and its perimeter in mm, the fan
/// from 0 to 1.
#[derive(Debug, Clone, Copy)]
pub struct Layer {
    pub volume: f64,
    pub height: f64,
    pub perimeter: f64,
    pub fan: f64,
}

#[derive(Debug, Clone, Copy)]
struct Slab {
    /// In m², m and m.
    area: f64,
    height: f64,
    perimeter: f64,
    temperature: f64,
}

/// A print as it cools: its slabs, from the bed up.
#[derive(Debug, Clone)]
pub struct Column {
    physics: Physics,
    slabs: Vec<Slab>,
}

impl Column {
    pub fn new(physics: Physics) -> Self {
        Column { physics, slabs: Vec::new() }
    }

    /// The heat transfer coefficient, in W/(m²·K), over a breadth `l` at `t` °C, the fan at
    /// `fan` of its full air speed.
    fn convection(&self, l: f64, t: f64, fan: f64) -> f64 {
        let p = &self.physics;
        if l <= 0.0 {
            return 0.0;
        }
        let u = p.fan_air_speed * fan.clamp(0.0, 1.0);
        let forced = 0.664 * (u * l / AIR_VISCOSITY).sqrt() * AIR_PRANDTL.cbrt() * AIR_CONDUCTIVITY / l;
        let film = (t + p.air_temperature) / 2.0 + CELSIUS;
        let rayleigh =
            GRAVITY * (t - p.air_temperature).abs() / film * l.powi(3) / (AIR_VISCOSITY * AIR_DIFFUSIVITY);
        let free = 0.54 * rayleigh.powf(0.25) * AIR_CONDUCTIVITY / l;
        (forced.powi(3) + free.powi(3)).cbrt()
    }

    /// Lays `layer` over `seconds`: the temperature, in °C, of its slab when it is done.
    pub fn lay(&mut self, layer: &Layer, seconds: f64) -> f64 {
        let p = self.physics.clone();
        let (h, perimeter) = (layer.height * 1e-3, layer.perimeter * 1e-3);
        let volume = layer.volume * 1e-9;
        if !(h > 0.0 && volume > 0.0) {
            return p.air_temperature;
        }
        self.slabs.push(Slab { area: volume / h, height: h, perimeter, temperature: p.melt_temperature });
        let steps = (seconds / STEP).ceil().max(1.0) as usize;
        let dt = seconds.max(0.0) / steps as f64;
        let heat = p.density * p.specific_heat;
        let n = self.slabs.len();
        let mut laid = 0.0;
        for k in 0..steps {
            // The share of the top slab laid by the end of the step.
            let share = (k + 1) as f64 / steps as f64;
            let grown = |j: usize| if j + 1 == n { share } else { 1.0 };
            // Each slab's conductance to the one below (to the bed, for the first).
            let below: Vec<f64> = self
                .slabs
                .iter()
                .enumerate()
                .map(|(j, s)| match j.checked_sub(1).and_then(|i| self.slabs.get(i)) {
                    Some(b) => p.conductivity * b.area.min(s.area) * grown(j) / ((b.height + s.height) / 2.0),
                    None => p.conductivity * s.area * grown(j) / (s.height / 2.0),
                })
                .collect();
            let above = below.iter().skip(1).copied().chain(std::iter::once(0.0));
            let rows: Vec<Row> = self
                .slabs
                .iter()
                .enumerate()
                .zip(below.iter().zip(above))
                .map(|((j, s), (&g_below, g_above))| {
                    let top = j + 1 == n;
                    let f = grown(j);
                    let full = heat * s.area * s.height;
                    let (before, came) = if top { (full * laid, full * (share - laid)) } else { (full, 0.0) };
                    let covered = self.slabs.get(j + 1).map_or(0.0, |a| a.area.min(s.area));
                    let bare = (s.area * f - covered).max(0.0);
                    let g_air = self.convection(s.area.sqrt(), s.temperature, layer.fan)
                        * (s.perimeter * s.height * f + bare);
                    let bed = if j == 0 { g_below * p.bed_temperature } else { 0.0 };
                    Row {
                        lower: if j == 0 { 0.0 } else { -g_below },
                        diagonal: full * f / dt + g_air + g_below + g_above,
                        upper: -g_above,
                        rhs: before * s.temperature / dt
                            + came * p.melt_temperature / dt
                            + g_air * p.air_temperature
                            + bed,
                    }
                })
                .collect();
            for (s, t) in self.slabs.iter_mut().zip(solve(&rows)) {
                s.temperature = t;
            }
            laid = share;
        }
        self.slabs.last().map_or(p.air_temperature, |s| s.temperature)
    }
}

/// One row of a tridiagonal system: `lower·x[i−1] + diagonal·x[i] + upper·x[i+1] = rhs`.
struct Row {
    lower: f64,
    diagonal: f64,
    upper: f64,
    rhs: f64,
}

/// The solution of a tridiagonal system, by Thomas's elimination: the matrix is diagonally
/// dominant, so it needs no pivot.
fn solve(rows: &[Row]) -> Vec<f64> {
    let mut swept: Vec<(f64, f64)> = Vec::with_capacity(rows.len());
    let mut previous = (0.0, 0.0);
    for r in rows {
        let m = r.diagonal - r.lower * previous.0;
        previous = if m != 0.0 { (r.upper / m, (r.rhs - r.lower * previous.1) / m) } else { (0.0, 0.0) };
        swept.push(previous);
    }
    let mut x = vec![0.0; rows.len()];
    let mut next = 0.0;
    for (xi, (c, d)) in x.iter_mut().zip(&swept).rev() {
        next = d - c * next;
        *xi = next;
    }
    x
}
