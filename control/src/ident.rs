//! Regressors that turn gyroscope readings and motor commands into the `(φ, y)` pairs of
//! recursive least squares.
//!
//! Both regressors identify the rate loop, not the angle: the angle is the known integral
//! of the rate, and the rate loop has fewer parameters and is better conditioned. The pairs
//! go to a [`minikalman`] RLS estimator and, for the diagnostics, to `miniident`.
//!
//! Both work in physical units: the output `y` is an angular acceleration (rad/s², the
//! rate difference divided by the sample period), and the regressors are thrusts (N) or
//! accelerations. Regressors of about unit size keep the RLS covariance and the
//! `miniident` thresholds (which assume unit-sized regressors) meaningful.
//!
//! Call `push` once per sample, after the command for that sample is known, with the
//! gyroscope reading of that sample and the command. Both regressors assume one sample of
//! delay between the command and the motors, as the model in [`crate::model`] does.

use crate::model::AxisParameters;
use crate::{Error, positive, to_f32};

/// Regressor for the rate model with three parameters `θ = [a, β₀, β₁]`:
///
/// ```text
/// α[k+1] = a·α[k] + β₀·u[k] + β₁·u[k−1],   α[k] = (ω[k+1] − ω[k]) / T_s
/// ```
///
/// `a` is the motor pole; [`AxisParameters::from_rate_arx`] converts an estimate back into
/// the axis parameters. The rate differences in `φ` contain the gyroscope noise twice, which
/// biases least squares. See [`GainRegressor`] for an unbiased alternative when the motor
/// time constant is known.
#[derive(Debug, Clone, PartialEq)]
pub struct RateRegressor {
    /// `[ω[n−1], ω[n−2]]`, newest first.
    omega: [f32; 2],
    /// `[u[n−1], u[n−2], u[n−3]]`, newest first.
    u: [f32; 3],
    /// Number of samples pushed so far, saturated at 3.
    filled: u8,
    /// `1 / T_s` in 1/s.
    rate: f32,
}

impl RateRegressor {
    /// Creates an empty regressor for the sample period of `axis`.
    #[must_use]
    pub fn new(axis: &AxisParameters) -> Self {
        Self {
            omega: [0.0; 2],
            u: [0.0; 3],
            filled: 0,
            rate: to_f32(1.0 / axis.sample_period()),
        }
    }

    /// Takes the rate `omega` (rad/s) and the command `u` (N) of sample `n`.
    ///
    /// Returns `φ = [(ω[n−1] − ω[n−2]) / T_s, u[n−2], u[n−3]]` and
    /// `y = (ω[n] − ω[n−1]) / T_s`, or `None` during the first three samples.
    pub fn push(&mut self, omega: f32, u: f32) -> Option<([f32; 3], f32)> {
        let pair = (self.filled >= 3).then(|| {
            let phi = [
                (self.omega[0] - self.omega[1]) * self.rate,
                self.u[1],
                self.u[2],
            ];
            (phi, (omega - self.omega[0]) * self.rate)
        });

        self.omega = [omega, self.omega[0]];
        self.u = [u, self.u[0], self.u[1]];
        self.filled = self.filled.saturating_add(1).min(3);

        pair
    }
}

/// Regressor for the input gain `b` alone, with a known motor time constant:
///
/// ```text
/// α[k] = b·(tau_m·g·s[k] + (T_s − tau_m·g)·u[k−1]) / T_s,   s[k+1] = a·s[k] + g·u[k−1]
/// ```
///
/// with `a = e^(−T_s / tau_m)`, `g = 1 − a` and the angular acceleration
/// `α[k] = (ω[k+1] − ω[k]) / T_s`. `s` is the thrust difference that the commands produce
/// through the motor model, and `φ` is its average over the sample, in N. `φ` contains only commands, which are
/// known exactly, so the gyroscope noise does not bias the estimate.
///
/// `tau_m` comes from a thrust stand (a step of the motor command, with the rotor speed
/// sensor of the chapter on thrust), or from a [`RateRegressor`] estimate.
#[derive(Debug, Clone, PartialEq)]
pub struct GainRegressor {
    /// Motor pole `a`.
    a: f32,
    /// `g = 1 − a`.
    g: f32,
    /// Coefficient `tau_m·g / T_s` of `s` in `φ`.
    c_state: f32,
    /// Coefficient `1 − tau_m·g / T_s` of the delayed command in `φ`.
    c_input: f32,
    /// Modeled thrust difference per unit of `b`, `s[n−1]`.
    s: f32,
    /// The previous rate `ω[n−1]`; `None` before the first sample.
    omega: Option<f32>,
    /// `1 / T_s` in 1/s.
    rate: f32,
    /// `[u[n−1], u[n−2]]`, newest first.
    u: [f32; 2],
}

impl GainRegressor {
    /// Creates the regressor for the motor time constant and sample period of `axis`; its
    /// input gain is not used.
    #[must_use]
    pub fn new(axis: &AxisParameters) -> Self {
        Self::with_lag(axis.tau_m(), axis.sample_period())
    }

    /// Creates the regressor for `tau_m` and the sample period, both in s and already
    /// checked to be finite and positive.
    fn with_lag(tau_m: f64, sample_period: f64) -> Self {
        let a = libm::exp(-sample_period / tau_m);
        let g = 1.0 - a;
        let tau_g = tau_m * g;

        Self {
            a: to_f32(a),
            g: to_f32(g),
            c_state: to_f32(tau_g / sample_period),
            c_input: to_f32(1.0 - tau_g / sample_period),
            s: 0.0,
            omega: None,
            rate: to_f32(1.0 / sample_period),
            u: [0.0; 2],
        }
    }

    /// Takes the rate `omega` (rad/s) and the command `u` (N) of sample `n`.
    ///
    /// Returns `φ = [(tau_m·g·s[n−1] + (T_s − tau_m·g)·u[n−2]) / T_s]` (N) and
    /// `y = (ω[n] − ω[n−1]) / T_s` (rad/s²), or `None` on the first sample.
    pub fn push(&mut self, omega: f32, u: f32) -> Option<([f32; 1], f32)> {
        let pair = self.omega.map(|prev| {
            let phi = self.c_state * self.s + self.c_input * self.u[1];
            ([phi], (omega - prev) * self.rate)
        });

        self.s = self.a * self.s + self.g * self.u[1];
        self.omega = Some(omega);
        self.u = [u, self.u[0]];

        pair
    }
}

/// A bank of [`GainRegressor`]s over candidate motor time constants, to find `tau_m` and
/// `b` together.
///
/// The three-parameter [`RateRegressor`] needs `tau_m` from the rate differences, and the
/// gyroscope noise in those differences biases it badly. Each regressor of the bank instead
/// assumes one `tau_m` and fits only `b`, which is unbiased. The candidate whose fit leaves
/// the smallest residual is the best `tau_m`; [`MotorLagSearch::estimate`] refines it by a
/// parabola through the neighbors.
///
/// The fits are batch least squares over all samples, with `f64` sums: three products per
/// candidate and sample.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorLagSearch<const K: usize> {
    /// Candidate time constants in s, strictly ascending and positive.
    candidates: [f64; K],
    /// One regressor per candidate.
    regressors: [GainRegressor; K],
    /// Sums `[Σφ², Σφ·y, Σy²]` per candidate.
    sums: [[f64; 3]; K],
    /// Number of `(φ, y)` pairs in the sums.
    pairs: u32,
}

/// The result of a [`MotorLagSearch`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LagEstimate {
    /// Estimated motor time constant in s.
    pub tau_m: f64,
    /// Estimated input gain in rad/s²/N, from the best candidate.
    pub b: f64,
    /// RMS residual of the best candidate in rad/s.
    pub rms: f64,
    /// True if the best candidate is the first or the last one. The true `tau_m` can then
    /// be outside the range, and the candidates must be widened.
    pub at_edge: bool,
}

impl<const K: usize> MotorLagSearch<K> {
    /// Creates the bank for the candidate time constants (s, strictly ascending and
    /// positive, at least three) and the sample period (s).
    ///
    /// # Errors
    ///
    /// [`Error::InvalidParameter`] if there are fewer than three candidates, if they are
    /// not strictly ascending and positive, or if the sample period is not positive.
    pub fn new(candidates: [f64; K], sample_period: f64) -> Result<Self, Error> {
        let ascending = candidates.windows(2).all(|w| w[0] < w[1]);
        let positive_first = candidates.first().is_some_and(|c| *c > 0.0);
        if K < 3 || !ascending || !positive_first || !candidates.iter().all(|c| c.is_finite()) {
            return Err(Error::InvalidParameter("candidates"));
        }

        let sample_period = positive(sample_period, "sample_period")?;

        Ok(Self {
            candidates,
            regressors: candidates.map(|tau| GainRegressor::with_lag(tau, sample_period)),
            sums: [[0.0; 3]; K],
            pairs: 0,
        })
    }

    /// Takes the rate `omega` (rad/s) and the command `u` (N) of one sample.
    pub fn push(&mut self, omega: f32, u: f32) {
        let mut added = false;
        for (regressor, sums) in self.regressors.iter_mut().zip(&mut self.sums) {
            if let Some(([phi], y)) = regressor.push(omega, u) {
                let (phi, y) = (f64::from(phi), f64::from(y));
                sums[0] += phi * phi;
                sums[1] += phi * y;
                sums[2] += y * y;
                added = true;
            }
        }

        if added {
            self.pairs = self.pairs.saturating_add(1);
        }
    }

    /// The candidate time constants in s and, per candidate, the fitted input gain
    /// (rad/s²/N) and the RMS residual (rad/s²), for diagnostics. A candidate without input
    /// so far has a NaN gain and an infinite residual.
    #[must_use]
    pub fn candidates(&self) -> [(f64, f64, f64); K] {
        let pairs = f64::from(self.pairs.max(1));
        core::array::from_fn(|i| {
            let [sxx, sxy, _] = self.sums[i];
            let gain = if sxx > 0.0 { sxy / sxx } else { f64::NAN };
            (
                self.candidates[i],
                gain,
                libm::sqrt(residual_sum(self.sums[i]) / pairs),
            )
        })
    }

    /// The current estimate, or `None` while no candidate has seen an input.
    #[must_use]
    pub fn estimate(&self) -> Option<LagEstimate> {
        let rss = self.sums.map(residual_sum);
        let best = (0..K)
            .filter(|&i| rss[i].is_finite())
            .min_by(|&i, &j| rss[i].total_cmp(&rss[j]))?;
        let at_edge = best == 0 || best == K - 1;
        let tau_m = if at_edge {
            self.candidates[best]
        } else {
            parabola_minimum(
                [
                    self.candidates[best - 1],
                    self.candidates[best],
                    self.candidates[best + 1],
                ],
                [rss[best - 1], rss[best], rss[best + 1]],
            )
        };

        let [sxx, sxy, _] = self.sums[best];

        Some(LagEstimate {
            tau_m,
            b: sxy / sxx,
            rms: libm::sqrt(rss[best] / f64::from(self.pairs.max(1))),
            at_edge,
        })
    }
}

/// The residual sum of squares `Σy² − (Σφ·y)²/Σφ²` of a one-parameter fit, or infinity if
/// `Σφ² = 0`.
fn residual_sum([sxx, sxy, syy]: [f64; 3]) -> f64 {
    if sxx > 0.0 {
        syy - sxy * sxy / sxx
    } else {
        f64::INFINITY
    }
}

/// The abscissa of the vertex of the parabola through three points, clamped to the outer
/// points. Falls back to the middle point if the points are collinear.
fn parabola_minimum(x: [f64; 3], y: [f64; 3]) -> f64 {
    let d1 = (y[1] - y[0]) / (x[1] - x[0]);
    let d2 = (y[2] - y[1]) / (x[2] - x[1]);
    let curvature = (d2 - d1) / (x[2] - x[0]);
    if curvature <= 0.0 || !curvature.is_finite() {
        return x[1];
    }

    let vertex = f64::midpoint(x[0], x[1]) - d1 / (2.0 * curvature);
    vertex.clamp(x[0], x[2])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reference airframe.
    fn axis() -> AxisParameters {
        AxisParameters::new(34.0, 0.03, 0.002).expect("reference values are valid")
    }

    /// Runs the noise-free discrete model with an alternating input and feeds `push`.
    fn run(mut push: impl FnMut(f32, f32)) {
        let d = axis().discretize();
        let mut x = [0.0; 4];
        for k in 0..200_u16 {
            let u = if (k / 7) % 2 == 0 { 0.3 } else { -0.2 };
            push(to_f32(x[1]), u);
            x = d.step(&x, f64::from(u));
        }
    }

    /// On noise-free data, every pair of the rate regressor satisfies the model exactly.
    #[test]
    fn rate_pairs_fit_the_model() {
        let params = axis();
        let discrete = params.discretize();
        let pole = params.motor_pole();
        let (c1, c2) = (discrete.phi[1][2], discrete.phi[1][3]);
        let period = params.sample_period();
        let theta = [pole, c2 / period, (c1 * (1.0 - pole) - pole * c2) / period].map(to_f32);

        let mut regressor = RateRegressor::new(&params);
        run(|omega, u| {
            if let Some((phi, y)) = regressor.push(omega, u) {
                let model: f32 = theta.iter().zip(&phi).map(|(t, f)| t * f).sum();
                assert!((model - y).abs() < 2e-3, "{model} != {y}");
            }
        });
    }

    /// On noise-free data, the lag search finds the true time constant between the grid
    /// points.
    #[test]
    fn lag_search_finds_tau() {
        let mut search =
            MotorLagSearch::new([0.010, 0.020, 0.028, 0.040, 0.050], 0.002).expect("valid grid");
        run(|omega, u| search.push(omega, u));
        let est = search
            .estimate()
            .expect("the input excites every candidate");
        assert!(!est.at_edge);
        assert!((est.tau_m - 0.03).abs() < 0.003, "{est:?}");
    }

    /// On noise-free data, every pair of the gain regressor gives `y = b·φ`.
    #[test]
    fn gain_pairs_fit_the_model() {
        let mut r = GainRegressor::new(&axis());
        run(|omega, u| {
            if let Some(([phi], y)) = r.push(omega, u) {
                assert!((34.0 * phi - y).abs() < 2e-3, "{} != {y}", 34.0 * phi);
            }
        });
    }
}
