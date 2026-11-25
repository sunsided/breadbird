//! The model of one attitude axis and its discretization.
//!
//! The commanded thrust difference `u` (in N) goes through one sample of delay and a
//! first-order motor lag with the time constant `tau_m` to the actual thrust difference
//! `ΔT`, which accelerates the frame by `b·ΔT` (with `b = l / J`), and two integrators give
//! the rate `ω` and the angle `θ`:
//!
//! ```text
//! θ' = ω,  ω' = b·ΔT,  ΔT' = (u_prev − ΔT) / tau_m
//! ```
//!
//! The discrete state is `x = [θ, ω, ΔT, u_prev]`, with `x[k+1] = Φ·x[k] + Γ·u[k]`.

use crate::linalg::Matrix;
use crate::{Error, positive};

/// The number of states of the discrete axis model: angle, rate, thrust difference and the
/// delayed input.
pub const STATES: usize = 4;

/// The physical parameters of one attitude axis and the sample period of the controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisParameters {
    /// Angular acceleration per newton of thrust difference, `b = l / J`, in rad/s²/N.
    /// Always finite and greater than zero.
    b: f64,
    /// Motor time constant `tau_m` in s. Always finite and greater than zero.
    tau_m: f64,
    /// Sample period `T_s` of the controller in s. Always finite and greater than zero.
    sample_period: f64,
}

impl AxisParameters {
    /// Creates the parameters from the input gain `b` (rad/s²/N), the motor time constant
    /// `tau_m` (s) and the sample period (s).
    ///
    /// # Errors
    ///
    /// [`Error::InvalidParameter`] if one of the values is not finite or not greater than
    /// zero.
    pub fn new(b: f64, tau_m: f64, sample_period: f64) -> Result<Self, Error> {
        Ok(Self {
            b: positive(b, "b")?,
            tau_m: positive(tau_m, "tau_m")?,
            sample_period: positive(sample_period, "sample_period")?,
        })
    }

    /// Recovers the parameters from the estimate `θ = [a, β₀, β₁]` of the rate model of
    /// [`RateRegressor`](crate::ident::RateRegressor).
    ///
    /// The motor pole `a = e^(−T_s / tau_m)` gives `tau_m = −T_s / ln a`. The sum of the
    /// input coefficients is `β₀ + β₁ = (1 − a)·b`, which gives `b`.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidParameter`] if `a` is not inside `(0, 1)` (no stable, non-oscillating
    /// motor lag), or if the resulting `b` is not greater than zero.
    pub fn from_rate_arx(theta: [f64; 3], sample_period: f64) -> Result<Self, Error> {
        let [a, beta0, beta1] = theta;
        let sample_period = positive(sample_period, "sample_period")?;
        if !(a > 0.0 && a < 1.0) {
            return Err(Error::InvalidParameter("a"));
        }

        let tau_m = -sample_period / libm::log(a);
        let b = (beta0 + beta1) / (1.0 - a);

        Self::new(b, tau_m, sample_period)
    }

    /// Angular acceleration per newton of thrust difference in rad/s²/N.
    #[must_use]
    pub fn b(&self) -> f64 {
        self.b
    }

    /// Motor time constant in s.
    #[must_use]
    pub fn tau_m(&self) -> f64 {
        self.tau_m
    }

    /// Sample period in s.
    #[must_use]
    pub fn sample_period(&self) -> f64 {
        self.sample_period
    }

    /// The motor pole `a = e^(−T_s / tau_m)` of the discrete model.
    #[must_use]
    pub fn motor_pole(&self) -> f64 {
        libm::exp(-self.sample_period / self.tau_m)
    }

    /// Discretizes the axis with a zero-order hold and one sample of input delay.
    ///
    /// The matrix exponential has a closed form for this chain, so no general `expm` is
    /// needed. `matlab/test_design.m` checks the same formulas against `c2d`.
    #[must_use]
    pub fn discretize(&self) -> DiscreteAxis {
        let (b, tau, t) = (self.b, self.tau_m, self.sample_period);
        let a = self.motor_pole();
        let g = 1.0 - a;

        let phi = [
            [
                1.0,
                t,
                b * tau * (t - tau * g),
                b * (t * t / 2.0 - tau * (t - tau * g)),
            ],
            [0.0, 1.0, b * tau * g, b * (t - tau * g)],
            [0.0, 0.0, a, g],
            [0.0, 0.0, 0.0, 0.0],
        ];

        DiscreteAxis {
            phi,
            gamma: [0.0, 0.0, 0.0, 1.0],
            sample_period: t,
        }
    }
}

/// The discrete model `x[k+1] = Φ·x[k] + Γ·u[k]` of one axis, with
/// `x = [θ, ω, ΔT, u_prev]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiscreteAxis {
    /// State transition matrix `Φ`.
    pub phi: Matrix<STATES, STATES>,
    /// Input vector `Γ`. The input only enters the delay state.
    pub gamma: [f64; STATES],
    /// Sample period in s.
    pub sample_period: f64,
}

impl DiscreteAxis {
    /// Advances the state by one sample with the input `u`.
    #[must_use]
    pub fn step(&self, x: &[f64; STATES], u: f64) -> [f64; STATES] {
        let mut next = [0.0; STATES];
        for (n, (row, g)) in next.iter_mut().zip(self.phi.iter().zip(&self.gamma)) {
            *n = row.iter().zip(x).map(|(p, xi)| p * xi).sum::<f64>() + g * u;
        }

        next
    }

    /// `Γ` as a column matrix.
    #[must_use]
    pub fn gamma_column(&self) -> Matrix<STATES, 1> {
        self.gamma.map(|g| [g])
    }

    /// The model augmented by the integral of the angle error, for integral action.
    ///
    /// The fifth state is `x_i[k+1] = x_i[k] + T_s·(r − θ[k])`. For the design, the
    /// reference is zero, so the row is `[−T_s, 0, 0, 0, 1]`.
    #[must_use]
    pub fn with_integral(&self) -> (Matrix<5, 5>, Matrix<5, 1>) {
        let mut phi = [[0.0; 5]; 5];
        for (row, src) in phi.iter_mut().zip(&self.phi) {
            row[..STATES].copy_from_slice(src);
        }

        phi[STATES] = [-self.sample_period, 0.0, 0.0, 0.0, 1.0];

        let mut gamma = [[0.0]; 5];
        gamma[STATES - 1] = [1.0];

        (phi, gamma)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reference airframe of the chapter.
    fn reference() -> AxisParameters {
        AxisParameters::new(34.0, 0.03, 0.002).expect("reference values are valid")
    }

    /// A step of the discrete model matches a fine Euler integration of the continuous one.
    #[test]
    fn discretization_matches_fine_integration() {
        let p = reference();
        let d = p.discretize();
        let x0 = [0.1, -0.5, 0.2, 0.3];
        let exact = d.step(&x0, 0.7);

        let (mut theta, mut omega, mut thrust) = (x0[0], x0[1], x0[2]);
        let n = 200_000_u32;
        let h = p.sample_period() / f64::from(n);
        for _ in 0..n {
            let d_thrust = (x0[3] - thrust) / p.tau_m();
            theta += h * omega;
            omega += h * p.b() * thrust;
            thrust += h * d_thrust;
        }

        let euler = [theta, omega, thrust, 0.7];
        for (e, x) in euler.iter().zip(&exact) {
            assert!((e - x).abs() < 1e-6, "{euler:?} != {exact:?}");
        }
    }

    /// The parameters survive the round trip through the rate model coefficients.
    #[test]
    fn rate_arx_round_trip() {
        let p = reference();
        let d = p.discretize();
        let a = p.motor_pole();
        let (c1, c2) = (d.phi[1][2], d.phi[1][3]);
        let theta = [a, c2 / 0.002, (c1 * (1.0 - a) - a * c2) / 0.002];

        let back = AxisParameters::from_rate_arx(theta, 0.002).expect("theta is consistent");
        assert!((back.b() - 34.0).abs() < 1e-9);
        assert!((back.tau_m() - 0.03).abs() < 1e-12);
    }

    /// Invalid values are rejected with the name of the parameter.
    #[test]
    fn invalid_parameters_are_rejected() {
        assert_eq!(
            AxisParameters::new(f64::NAN, 0.03, 0.002),
            Err(Error::InvalidParameter("b"))
        );
        assert_eq!(
            AxisParameters::new(34.0, 0.0, 0.002),
            Err(Error::InvalidParameter("tau_m"))
        );
        assert_eq!(
            AxisParameters::from_rate_arx([1.2, 0.0, 0.0], 0.002),
            Err(Error::InvalidParameter("a"))
        );
    }
}
