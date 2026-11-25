//! Differential tests against the gains that the Octave design desk computes for the
//! reference airframe (`matlab/attitude_control.m`, printed at the end of its run). The
//! Octave helpers are in turn checked against the Octave control package by
//! `matlab/test_design.m`, so these tests tie the Rust design to `place`, `dlqr` and
//! `dlqe`.

use breadbird_control::design::{MEASUREMENT, Pole, dare, kalman_gain, lqr, place};
use breadbird_control::linalg::{Matrix, add, mul, norm1, transpose};
use breadbird_control::model::{AxisParameters, DiscreteAxis};

/// `Φ` of the reference airframe from Octave (`Ad`).
#[expect(
    clippy::excessive_precision,
    reason = "Octave prints 17 digits; the extra digits round away"
)]
const PHI: Matrix<4, 4> = [
    [
        1.0,
        0.002,
        6.651_374_196_750_409_7e-5,
        1.486_258_032_495_902_1e-6,
    ],
    [
        0.0,
        1.0,
        0.065_782_875_267_749_868,
        0.002_217_124_732_250_136_4,
    ],
    [
        0.0,
        0.0,
        0.935_506_985_031_617_78,
        0.064_493_014_968_382_223,
    ],
    [0.0, 0.0, 0.0, 0.0],
];

/// Pole placement gain for `ω_n = 15 rad/s`, `ζ = 0.7`, motor pole at `−40 rad/s` and the
/// delay pole at zero (`K_pp`).
#[expect(
    clippy::excessive_precision,
    reason = "Octave prints 17 digits; the extra digits round away"
)]
const K_PP: [f64; 4] = [
    7.725_099_804_632_711_7,
    0.937_383_348_434_156_94,
    0.820_478_189_065_421_58,
    0.054_402_153_819_160_741,
];

/// LQR gain for `Q = diag(400, 1, 0, 0)`, `R = 4` (`K_lqr`).
#[expect(
    clippy::excessive_precision,
    reason = "Octave prints 17 digits; the extra digits round away"
)]
const K_LQR: [f64; 4] = [
    9.727_074_549_810_629,
    1.149_587_443_122_257_5,
    0.840_948_815_747_493_36,
    0.055_324_591_527_519_396,
];

/// LQR gain with integral action, `Q = diag(400, 1, 0, 0, 10⁴)`, `R = 4` (`K_lqr_i`).
#[expect(
    clippy::excessive_precision,
    reason = "Octave prints 17 digits; the extra digits round away"
)]
const K_LQR_I: [f64; 5] = [
    15.183_456_355_206_371,
    1.396_792_917_861_791_7,
    0.975_950_028_200_325_61,
    0.064_066_695_473_651_125,
    -48.423_239_749_242_533,
];

/// Current-form observer gain for `Q_n = diag(0, 10⁻⁴, 10⁻², 0)` and the reference noise
/// (`Kf`).
#[expect(
    clippy::excessive_precision,
    reason = "Octave prints 17 digits; the extra digits round away"
)]
const KF: Matrix<4, 2> = [
    [0.011_100_446_996_202_428, 0.001_176_280_667_547_125_7],
    [0.029_407_016_688_678_112, 0.765_822_523_971_713_69],
    [0.221_571_748_244_553_2, 4.110_861_026_802_417_9],
    [0.0, 0.0],
];

/// The discrete reference airframe.
fn reference() -> DiscreteAxis {
    AxisParameters::new(34.0, 0.03, 0.002)
        .expect("reference values are valid")
        .discretize()
}

/// Asserts that `actual` matches `expected` to the relative tolerance `tol`.
fn assert_close(actual: &[f64], expected: &[f64], tol: f64) {
    let scale = expected.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    for (a, e) in actual.iter().zip(expected) {
        assert!((a - e).abs() <= tol * scale, "{actual:?} != {expected:?}");
    }
}

/// The closed-form discretization matches Octave's `expm`.
#[test]
fn discretization_matches_octave() {
    let d = reference();
    for (row, expected) in d.phi.iter().zip(&PHI) {
        assert_close(row, expected, 1e-12);
    }
}

/// Ackermann's formula at 500 Hz matches Octave.
#[test]
fn pole_placement_matches_octave() {
    let d = reference();
    let [p1, p2] = Pole::pair(15.0, 0.7, 0.002);
    let poles = [
        p1,
        p2,
        Pole::from_continuous(-40.0, 0.0, 0.002),
        Pole::real(0.0),
    ];
    let k = place(&d.phi, &d.gamma, &poles).expect("the axis is controllable");
    assert_close(&k, &K_PP, 1e-6);
}

/// LQR at 500 Hz matches Octave, and the Riccati solution has a small residual.
#[test]
fn lqr_matches_octave() {
    let axis = reference();
    let mut weights = [[0.0; 4]; 4];
    weights[0][0] = 400.0;
    weights[1][1] = 1.0;
    let input_weight = [[4.0]];

    let gain =
        lqr(&axis.phi, &axis.gamma_column(), &weights, &input_weight).expect("design converges")[0];
    assert_close(&gain, &K_LQR, 1e-8);

    let riccati =
        dare(&axis.phi, &axis.gamma_column(), &weights, &input_weight).expect("design converges");
    assert!(
        riccati_residual(&axis, &riccati, &weights, input_weight[0][0]) < 1e-9 * norm1(&riccati)
    );
}

/// LQR with integral action matches Octave.
#[test]
fn lqr_with_integral_matches_octave() {
    let (phi, gamma) = reference().with_integral();
    let mut q = [[0.0; 5]; 5];
    q[0][0] = 400.0;
    q[1][1] = 1.0;
    q[4][4] = 1e4;

    let k = lqr(&phi, &gamma, &q, &[[4.0]]).expect("design converges")[0];
    assert_close(&k, &K_LQR_I, 1e-8);
}

/// The steady-state Kalman gain matches Octave.
#[test]
fn observer_gain_matches_octave() {
    let d = reference();
    let mut qn = [[0.0; 4]; 4];
    qn[1][1] = 1e-4;
    qn[2][2] = 1e-2;
    let rn = [[0.002 * 0.002, 0.0], [0.0, 0.01 * 0.01]];

    let kf = kalman_gain(&d.phi, &MEASUREMENT, &qn, &rn).expect("design converges");
    for (row, expected) in kf.iter().zip(&KF) {
        assert_close(row, expected, 1e-7);
    }
}

/// `‖Q + ΦᵀPΦ − ΦᵀPΓ(R + ΓᵀPΓ)⁻¹ΓᵀPΦ − P‖₁` for a single input.
fn riccati_residual(
    axis: &DiscreteAxis,
    p: &Matrix<4, 4>,
    weights: &Matrix<4, 4>,
    input_weight: f64,
) -> f64 {
    let gamma = axis.gamma_column();
    let phi_t = transpose(&axis.phi);
    let quadratic = mul(&phi_t, &mul(p, &axis.phi));
    let cross = mul(&phi_t, &mul(p, &gamma));
    let scale = input_weight + mul(&transpose(&gamma), &mul(p, &gamma))[0][0];
    let correction = mul(&cross, &transpose(&cross)).map(|row| row.map(|v| -v / scale));
    let rhs = add(&add(weights, &quadratic), &correction);

    norm1(&add(&rhs, &p.map(|row| row.map(|v| -v))))
}
