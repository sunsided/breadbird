//! Gain design: pole placement, the discrete Riccati equation, LQR and the observer gain.
//!
//! Everything here runs in `f64`. At high sample rates, `Φ` is close to the identity, and
//! the controllability matrix of the reference airframe has a condition number of about
//! `2.7·10⁵` at 500 Hz. That costs about five of the 16 significant digits of `f64` and would
//! leave almost nothing of the 7 digits of `f32`. Design once after identification, then
//! run the loop in `f32` with the gains of [`AxisGains`].
//!
//! The functions are generic over the number of states, and checked against the Octave
//! control package by `matlab/test_design.m` (Octave side) and `tests/it/octave.rs`.

use crate::linalg::{Matrix, add, identity, inverse, mul, norm1, symmetrize, transpose};
use crate::model::{AxisParameters, STATES};
use crate::{Error, positive, to_f32};

/// The largest number of doubling steps [`dare`] tries. Doubling converges quadratically;
/// the reference airframe needs 11 steps.
pub const MAX_DOUBLING_STEPS: u32 = 100;

/// A pole in the `z`-plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pole {
    /// Real part.
    pub re: f64,
    /// Imaginary part.
    pub im: f64,
}

impl Pole {
    /// A real pole at `z`.
    #[must_use]
    pub fn real(z: f64) -> Self {
        Self { re: z, im: 0.0 }
    }

    /// The discrete pole `z = e^(s·T_s)` of the continuous pole `s = re + j·im`.
    #[must_use]
    pub fn from_continuous(re: f64, im: f64, sample_period: f64) -> Self {
        let r = libm::exp(re * sample_period);
        Self {
            re: r * libm::cos(im * sample_period),
            im: r * libm::sin(im * sample_period),
        }
    }

    /// The conjugate pair with natural frequency `wn` (rad/s) and damping ratio `zeta`
    /// (`0 < zeta < 1`), as discrete poles.
    #[must_use]
    pub fn pair(wn: f64, zeta: f64, sample_period: f64) -> [Self; 2] {
        let wd = wn * libm::sqrt(1.0 - zeta * zeta);
        [
            Self::from_continuous(-zeta * wn, wd, sample_period),
            Self::from_continuous(-zeta * wn, -wd, sample_period),
        ]
    }
}

/// The state feedback gain `K` that places the poles of `Φ − Γ·K` at `poles`, by
/// Ackermann's formula `K = [0 … 0 1]·C⁻¹·φ(Φ)`.
///
/// # Errors
///
/// * [`Error::NotConjugate`] if the complex poles do not come in conjugate pairs.
/// * [`Error::NotControllable`] if the controllability matrix `C` cannot be inverted.
pub fn place<const N: usize>(
    phi: &Matrix<N, N>,
    gamma: &[f64; N],
    poles: &[Pole; N],
) -> Result<[f64; N], Error> {
    let coefficients = characteristic_polynomial(poles)?;
    let c_inv = inverse(&controllability(phi, gamma)).ok_or(Error::NotControllable)?;
    let phi_of_phi = polynomial_of_matrix(&coefficients, phi);

    Ok(mul(&[c_inv[N - 1]], &phi_of_phi)[0])
}

/// The controllability matrix `[Γ, Φ·Γ, …, Φ^(N−1)·Γ]`.
#[must_use]
pub fn controllability<const N: usize>(phi: &Matrix<N, N>, gamma: &[f64; N]) -> Matrix<N, N> {
    let mut c = [[0.0; N]; N];
    let mut column = gamma.map(|g| [g]);
    for i in 0..N {
        for (row, value) in c.iter_mut().zip(&column) {
            row[i] = value[0];
        }

        column = mul(phi, &column);
    }

    c
}

/// The coefficients `[1, c₁, …, c_N]` of `(z − z₁)·…·(z − z_N)`, highest power first.
///
/// # Errors
///
/// [`Error::NotConjugate`] if an imaginary part of the product is larger than `1e-9` times
/// the largest coefficient, i.e. if the poles do not come in conjugate pairs.
fn characteristic_polynomial<const N: usize>(poles: &[Pole; N]) -> Result<[f64; N], Error> {
    const { assert!(N <= 8, "pole placement supports at most 8 states") };

    // Complex coefficients, highest power first; starts as the constant polynomial 1.
    let mut re = [0.0; 9];
    let mut im = [0.0; 9];
    re[0] = 1.0;

    for (degree, pole) in poles.iter().enumerate() {
        for k in (1..=degree + 1).rev() {
            re[k] -= pole.re * re[k - 1] - pole.im * im[k - 1];
            im[k] -= pole.re * im[k - 1] + pole.im * re[k - 1];
        }
    }

    let largest = re.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
    if im.iter().any(|c| c.abs() > 1e-9 * largest) {
        return Err(Error::NotConjugate);
    }

    let mut out = [0.0; N];
    out.copy_from_slice(&re[1..=N]);
    Ok(out)
}

/// `φ(Φ) = Φ^N + c₁·Φ^(N−1) + … + c_N·I` by Horner's scheme, with `coefficients` = `[c₁, …,
/// c_N]`.
fn polynomial_of_matrix<const N: usize>(
    coefficients: &[f64; N],
    phi: &Matrix<N, N>,
) -> Matrix<N, N> {
    let mut acc = identity::<N>();
    for c in coefficients {
        acc = mul(&acc, phi);
        for (i, row) in acc.iter_mut().enumerate() {
            row[i] += c;
        }
    }

    acc
}

/// Solves the discrete algebraic Riccati equation
/// `P = Q + Aᵀ·P·A − Aᵀ·P·B·(R + Bᵀ·P·B)⁻¹·Bᵀ·P·A` by the structure-preserving doubling
/// algorithm.
///
/// Each doubling step doubles the horizon of the Riccati recursion, so the iteration
/// converges in a few dozen steps where the plain recursion needs thousands at high sample
/// rates. For the steady-state Kalman filter, call it with `(Aᵀ, Hᵀ, Q_n, R_n)`; `P` is then
/// the prediction covariance.
///
/// # Errors
///
/// * [`Error::Singular`] if `R` or an intermediate matrix cannot be inverted.
/// * [`Error::NotConverged`] if the relative change of `P` stays above `1e-13` after
///   [`MAX_DOUBLING_STEPS`] steps.
pub fn dare<const N: usize, const M: usize>(
    a: &Matrix<N, N>,
    b: &Matrix<N, M>,
    q: &Matrix<N, N>,
    r: &Matrix<M, M>,
) -> Result<Matrix<N, N>, Error> {
    let r_inv = inverse(r).ok_or(Error::Singular)?;
    let mut ak = *a;
    let mut gk = mul(&mul(b, &r_inv), &transpose(b));
    let mut hk = *q;

    for _ in 0..MAX_DOUBLING_STEPS {
        let w_inv = inverse(&add(&identity::<N>(), &mul(&gk, &hk))).ok_or(Error::Singular)?;
        let ak_next = mul(&ak, &mul(&w_inv, &ak));
        let gk_next = add(&gk, &mul(&mul(&ak, &mul(&w_inv, &gk)), &transpose(&ak)));
        let hk_next = add(&hk, &mul(&transpose(&ak), &mul(&hk, &mul(&w_inv, &ak))));

        let change = norm1(&add(&hk_next, &hk.map(|row| row.map(|v| -v))));
        let converged = change <= 1e-13 * norm1(&hk_next);
        ak = ak_next;
        gk = symmetrize(&gk_next);
        hk = symmetrize(&hk_next);
        if converged {
            return Ok(hk);
        }
    }

    Err(Error::NotConverged)
}

/// The LQR gain `K = (R + Bᵀ·P·B)⁻¹·Bᵀ·P·A` for the Riccati solution `P` of [`dare`].
///
/// # Errors
///
/// See [`dare`]; also [`Error::Singular`] if `R + Bᵀ·P·B` cannot be inverted.
pub fn lqr<const N: usize, const M: usize>(
    a: &Matrix<N, N>,
    b: &Matrix<N, M>,
    q: &Matrix<N, N>,
    r: &Matrix<M, M>,
) -> Result<Matrix<M, N>, Error> {
    let riccati = dare(a, b, q, r)?;
    let bt_p = mul(&transpose(b), &riccati);
    let s_inv = inverse(&add(r, &mul(&bt_p, b))).ok_or(Error::Singular)?;

    Ok(mul(&s_inv, &mul(&bt_p, a)))
}

/// The steady-state Kalman gain `K_f = P·Hᵀ·(H·P·Hᵀ + R_n)⁻¹` in the current form
/// (`x̂ = x̄ + K_f·(y − H·x̄)`), from the Riccati equation of the dual system.
///
/// This is the gain a [`minikalman`] `LuenbergerObserver` in the `Current` form expects. The
/// predictor-form gain is `L = Φ·K_f`.
///
/// # Errors
///
/// See [`dare`]; also [`Error::Singular`] if `H·P·Hᵀ + R_n` cannot be inverted.
pub fn kalman_gain<const N: usize, const M: usize>(
    a: &Matrix<N, N>,
    h: &Matrix<M, N>,
    q: &Matrix<N, N>,
    r: &Matrix<M, M>,
) -> Result<Matrix<N, M>, Error> {
    let covariance = dare(&transpose(a), &transpose(h), q, r)?;
    let p_ht = mul(&covariance, &transpose(h));
    let s_inv = inverse(&add(&mul(h, &p_ht), r)).ok_or(Error::Singular)?;

    Ok(mul(&p_ht, &s_inv))
}

/// The LQR weights by Bryson's rule: each quantity is divided by the largest value that is
/// still acceptable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Weights {
    /// Largest acceptable angle error in rad.
    pub theta_max: f64,
    /// Largest acceptable rate in rad/s.
    pub omega_max: f64,
    /// Largest acceptable integral of the angle error in rad·s.
    pub integral_max: f64,
    /// Largest acceptable thrust difference command in N.
    pub u_max: f64,
}

impl Default for Weights {
    /// The weights of the reference airframe in the chapter "State-space control".
    fn default() -> Self {
        Self {
            theta_max: 0.05,
            omega_max: 1.0,
            integral_max: 0.01,
            u_max: 0.5,
        }
    }
}

/// The noise model for the observer gain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Noise {
    /// Process noise variance per state `[θ, ω, ΔT, u_prev]`, in the squared units of each
    /// state per sample.
    pub process: [f64; STATES],
    /// Standard deviation of the angle measurement in rad.
    pub theta_sigma: f64,
    /// Standard deviation of the gyroscope in rad/s.
    pub omega_sigma: f64,
}

impl Default for Noise {
    /// The noise model of the reference airframe: most uncertainty in the motors, none in
    /// the delayed input, which is known.
    fn default() -> Self {
        Self {
            process: [0.0, 1e-4, 1e-2, 0.0],
            theta_sigma: 0.002,
            omega_sigma: 0.01,
        }
    }
}

/// The gains of one axis, ready for the `f32` control loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisGains {
    /// State feedback gain on `[θ − r, ω, ΔT, u_prev]`, in N/rad, N·s/rad, 1 and 1.
    pub k: [f32; STATES],
    /// Integral gain on `∫(r − θ) dt`, in N/(rad·s).
    pub ki: f32,
    /// Current-form observer gain for the measurement `[θ, ω]`.
    pub observer: [[f32; 2]; STATES],
    /// The state transition `Φ` for the observer.
    pub phi: [[f32; STATES]; STATES],
    /// The input vector `Γ` for the observer.
    pub gamma: [f32; STATES],
}

/// The measurement matrix `H`: the angle and the rate are measured.
pub const MEASUREMENT: Matrix<2, STATES> = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]];

/// Designs LQR with integral action and the observer for one axis.
///
/// # Errors
///
/// * [`Error::InvalidParameter`] if a weight or a noise value is not finite and positive
///   (process noise may be zero).
/// * See [`lqr`] and [`kalman_gain`] for the design itself.
///
/// # Examples
///
/// ```
/// use breadbird_control::design::{Noise, Weights, design_axis};
/// use breadbird_control::model::AxisParameters;
///
/// let axis = AxisParameters::new(34.0, 0.03, 0.002)?;
/// let gains = design_axis(&axis, &Weights::default(), &Noise::default())?;
/// assert!(gains.k[0] > 0.0 && gains.ki > 0.0);
/// # Ok::<(), breadbird_control::Error>(())
/// ```
pub fn design_axis(
    axis: &AxisParameters,
    weights: &Weights,
    noise: &Noise,
) -> Result<AxisGains, Error> {
    let q = state_weights(weights)?;
    let r = [[1.0 / sq(positive(weights.u_max, "u_max")?)]];
    let discrete = axis.discretize();
    let (phi5, gamma5) = discrete.with_integral();
    let k5 = lqr(&phi5, &gamma5, &q, &r)?[0];

    let (qn, rn) = noise_covariances(noise)?;
    let kf = kalman_gain(&discrete.phi, &MEASUREMENT, &qn, &rn)?;

    Ok(AxisGains {
        k: [k5[0], k5[1], k5[2], k5[3]].map(to_f32),
        ki: to_f32(-k5[4]),
        observer: kf.map(|row| row.map(to_f32)),
        phi: discrete.phi.map(|row| row.map(to_f32)),
        gamma: discrete.gamma.map(to_f32),
    })
}

/// The state weights `Q` of the augmented system by Bryson's rule; the thrust states are not
/// weighted.
fn state_weights(w: &Weights) -> Result<Matrix<5, 5>, Error> {
    let mut q = [[0.0; 5]; 5];
    q[0][0] = 1.0 / sq(positive(w.theta_max, "theta_max")?);
    q[1][1] = 1.0 / sq(positive(w.omega_max, "omega_max")?);
    q[4][4] = 1.0 / sq(positive(w.integral_max, "integral_max")?);

    Ok(q)
}

/// The process and measurement covariances `Q_n` and `R_n` of `noise`.
fn noise_covariances(noise: &Noise) -> Result<(Matrix<STATES, STATES>, Matrix<2, 2>), Error> {
    let mut qn = [[0.0; STATES]; STATES];
    for (i, v) in noise.process.iter().enumerate() {
        if !(v.is_finite() && *v >= 0.0) {
            return Err(Error::InvalidParameter("process"));
        }

        qn[i][i] = *v;
    }

    let rn = [
        [sq(positive(noise.theta_sigma, "theta_sigma")?), 0.0],
        [0.0, sq(positive(noise.omega_sigma, "omega_sigma")?)],
    ];

    Ok((qn, rn))
}

/// `x²`.
fn sq(x: f64) -> f64 {
    x * x
}
