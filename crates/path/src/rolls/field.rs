//! The rolls' field and its flow down its energy, on a periodic grid `h` apart. In lengths of the
//! rolls' own scale, `1/q0`, the energy is
//!
//! `E = Σ h² [½((1 + Δ)u)² − ε/2·u² + ¼u⁴ + κ/2·w·(m·Du)² + μ/2·χ·(u − u↓)² + ½·M·u²]`,
//!
//! Swift–Hohenberg's: rolls a wavelength `2π/q0` wide, `Δ` exact in Fourier space; the anchoring
//! by the walls, `D` the central difference, whose adjoint is `−D`; the pull of the layer below
//! where it lay (`χ`); the damping beyond the walls (`M`). Each step is semi-implicit: the
//! linear operator exactly, with as much of the anchoring's own `−κ·D·D` as it may reach, the
//! rest and the cube explicitly, the pull and the damping pointwise and implicitly.

use super::Nodes;
use super::walls::Walls;
use rustfft::num_complex::Complex;
use rustfft::{Fft, FftPlanner};
use std::f64::consts::TAU;
use std::sync::Arc;

/// How far above onset the rolls are driven: strong enough to saturate, weak enough to stay
/// smooth.
const EPSILON: f64 = 0.35;
/// How strongly the walls turn the rolls.
const KAPPA: f64 = 1.5;
/// How strongly a layer's rolls follow those below.
const MU: f64 = 0.4;
/// The step in the rolls' own time.
const DT: f64 = 0.5;

/// The transforms of the grid's rows and columns, and what the flow keeps between steps.
pub(super) struct Flow {
    nodes: Nodes,
    row_forward: Arc<dyn Fft<f64>>,
    row_inverse: Arc<dyn Fft<f64>>,
    column_forward: Arc<dyn Fft<f64>>,
    column_inverse: Arc<dyn Fft<f64>>,
    /// `|k|²`, and `(sin²(kx·h) + sin²(ky·h))/h²`, the symbol of `−D·D`: column after column.
    k2: Vec<f64>,
    d2: Vec<f64>,
    buffer: Vec<Complex<f64>>,
    turned: Vec<Complex<f64>>,
    scratch: Vec<Complex<f64>>,
}

/// The angular frequency of the `i`-th of `n` samples `h` apart.
fn frequency(i: usize, n: usize, h: f64) -> f64 {
    let k = if 2 * i < n { i as f64 } else { i as f64 - n as f64 };
    TAU * k / (n as f64 * h)
}

impl Flow {
    pub fn new(nodes: Nodes) -> Flow {
        let mut planner = FftPlanner::new();
        let (row_forward, row_inverse) =
            (planner.plan_fft_forward(nodes.nx), planner.plan_fft_inverse(nodes.nx));
        let (column_forward, column_inverse) =
            (planner.plan_fft_forward(nodes.ny), planner.plan_fft_inverse(nodes.ny));
        let scratch = [&row_forward, &row_inverse, &column_forward, &column_inverse]
            .iter()
            .map(|f| f.get_inplace_scratch_len())
            .max()
            .unwrap_or(0);
        let (mut k2, mut d2) = (Vec::with_capacity(nodes.len()), Vec::with_capacity(nodes.len()));
        for i in 0..nodes.nx {
            let kx = frequency(i, nodes.nx, nodes.h);
            for j in 0..nodes.ny {
                let ky = frequency(j, nodes.ny, nodes.h);
                k2.push(kx * kx + ky * ky);
                d2.push(((kx * nodes.h).sin().powi(2) + (ky * nodes.h).sin().powi(2)) / (nodes.h * nodes.h));
            }
        }
        let zero = Complex::new(0.0, 0.0);
        Flow {
            nodes,
            row_forward,
            row_inverse,
            column_forward,
            column_inverse,
            k2,
            d2,
            buffer: vec![zero; nodes.len()],
            turned: vec![zero; nodes.len()],
            scratch: vec![zero; scratch],
        }
    }

    /// The central difference of `f` along x (`axis` 0) or y, the grid wrapping round.
    fn difference(&self, f: &[f64], axis: usize, out: &mut [f64]) {
        let (nx, ny, h) = (self.nodes.nx, self.nodes.ny, self.nodes.h);
        for (n, o) in out.iter_mut().enumerate() {
            let (i, j) = (n % nx, n / nx);
            let (before, after) = if axis == 0 {
                (j * nx + (i + nx - 1) % nx, j * nx + (i + 1) % nx)
            } else {
                (((j + ny - 1) % ny) * nx + i, ((j + 1) % ny) * nx + i)
            };
            *o = (f.get(after).copied().unwrap_or(0.0) - f.get(before).copied().unwrap_or(0.0)) / (2.0 * h);
        }
    }

    /// `u` replaced by the inverse transform of its transform divided by `denominator`: rows,
    /// then columns, which the grid turned lays out in a row each, and back.
    fn solve(&mut self, u: &mut [f64], denominator: &[f64]) {
        let (nx, ny) = (self.nodes.nx, self.nodes.ny);
        for (b, &v) in self.buffer.iter_mut().zip(u.iter()) {
            *b = Complex::new(v, 0.0);
        }
        for row in self.buffer.chunks_exact_mut(nx) {
            self.row_forward.process_with_scratch(row, &mut self.scratch);
        }
        for (n, t) in self.turned.iter_mut().enumerate() {
            *t = self.buffer.get((n % ny) * nx + n / ny).copied().unwrap_or_default();
        }
        for column in self.turned.chunks_exact_mut(ny) {
            self.column_forward.process_with_scratch(column, &mut self.scratch);
        }
        for (t, &d) in self.turned.iter_mut().zip(denominator) {
            *t /= d;
        }
        for column in self.turned.chunks_exact_mut(ny) {
            self.column_inverse.process_with_scratch(column, &mut self.scratch);
        }
        for (n, b) in self.buffer.iter_mut().enumerate() {
            *b = self.turned.get((n % nx) * ny + n / nx).copied().unwrap_or_default();
        }
        for row in self.buffer.chunks_exact_mut(nx) {
            self.row_inverse.process_with_scratch(row, &mut self.scratch);
        }
        let scale = 1.0 / (nx * ny) as f64;
        for (v, b) in u.iter_mut().zip(&self.buffer) {
            *v = b.re * scale;
        }
    }

    /// Steps `u` `steps` times down the energy of rolls `q0` held by `walls`, and pulled toward
    /// the field below, `(u↓, χ)`, where it lay.
    pub fn relax(
        &mut self,
        u: &mut [f64],
        walls: &Walls,
        q0: f64,
        steps: usize,
        below: Option<(&[f64], &[bool])>,
    ) {
        let q2 = q0 * q0;
        // In the rolls' own lengths `D` is `D/q0`: the anchoring and its implicit share carry `1/q0²`.
        let anchor = KAPPA / q2;
        let denominator: Vec<f64> = self
            .k2
            .iter()
            .zip(&self.d2)
            .map(|(&k2, &d2)| 1.0 - DT * (EPSILON - (1.0 - k2 / q2).powi(2)) + DT * anchor * d2)
            .collect();
        let n = self.nodes.len();
        let [mut ux, mut uy, mut fx, mut fy, mut dfx, mut dfy, mut dux, mut duy] =
            std::array::from_fn(|_| vec![0.0; n]);
        for _ in 0..steps {
            self.difference(u, 0, &mut ux);
            self.difference(u, 1, &mut uy);
            for (k, (a, b)) in fx.iter_mut().zip(fy.iter_mut()).enumerate() {
                let (w, m) = (
                    walls.weight.get(k).copied().unwrap_or(0.0),
                    walls.along.get(k).copied().unwrap_or([0.0; 2]),
                );
                let f =
                    w * (m[0] * ux.get(k).copied().unwrap_or(0.0) + m[1] * uy.get(k).copied().unwrap_or(0.0));
                (*a, *b) = (f * m[0], f * m[1]);
            }
            // The anchoring, `κ·D·(w·m·(m·Du))`, less the share `κ·D·Du` taken implicitly.
            self.difference(&fx, 0, &mut dfx);
            self.difference(&fy, 1, &mut dfy);
            self.difference(&ux, 0, &mut dux);
            self.difference(&uy, 1, &mut duy);
            for (k, v) in u.iter_mut().enumerate() {
                let get = |a: &[f64]| a.get(k).copied().unwrap_or(0.0);
                let explicit = get(&dfx) + get(&dfy) - get(&dux) - get(&duy);
                *v += DT * (anchor * explicit - *v * *v * *v);
            }
            self.solve(u, &denominator);
            for (k, v) in u.iter_mut().enumerate() {
                if let Some((below, lay)) = below
                    && lay.get(k).copied().unwrap_or(false)
                {
                    *v = (*v + DT * MU * below.get(k).copied().unwrap_or(0.0)) / (1.0 + DT * MU);
                }
                *v /= 1.0 + DT * walls.damping.get(k).copied().unwrap_or(0.0);
            }
        }
    }
}
