//! Bring-up of one attitude axis against a simulated airframe: identify, design, fly.
//!
//! The simulated airframe has parameters that differ from the placeholder values of the
//! chapter "State-space control" (`b = 30` instead of 34, `tau_m = 25 ms` instead of 30 ms),
//! so that the identification has something to find. A deliberately soft PD controller keeps
//! the axis level throughout, and a PRBS of ±0.15 N is added to its output.
//!
//! 1. **Motor lag (0 to 20 s).** A [`MotorLagSearch`] finds `tau_m`. For comparison, a
//!    `minikalman` RLS fits the three-parameter [`RateRegressor`] model on the same data,
//!    with `miniident` judging its identifiability.
//! 2. **Input gain (20 to 40 s).** A one-parameter RLS on the [`GainRegressor`] with the
//!    found `tau_m` tracks `b`, with `miniident` excitation and identifiability checks.
//! 3. **Design and flight.** [`design_axis`] computes the gains, and a `minikalman`
//!    Luenberger observer with the [`StateFeedback`] law flies a step of 0.1 rad, once with
//!    the gains of the placeholder airframe and once with the identified ones.
//!
//! Writes `typst/data/control/bringup_ident.csv`, `bringup_lag.csv` and `bringup_step.csv`. Run with
//! `cargo run --release --example bringup`.

use std::error::Error;
use std::fmt::Write as _;
use std::path::PathBuf;

use breadbird_control::design::{AxisGains, Noise, Weights, design_axis};
use breadbird_control::ident::{GainRegressor, LagEstimate, MotorLagSearch, RateRegressor};
use breadbird_control::law::StateFeedback;
use breadbird_control::model::{AxisParameters, DiscreteAxis};
use breadbird_control::pid::{IncrementalPid, PidConfig};
use miniident::prelude::*;
use minikalman::prelude::*;
use minikalman::regular::{
    BorrowedControl, BorrowedFixedGainObservation, BorrowedLuenbergerObserver, ControlStorage,
    FixedGainObservationStorage, ObserverStorage,
};
use minikalman::rls::{BorrowedEstimator, EstimatorStorage};

/// Sample period of the flight controller in s.
const T_S: f64 = 0.002;
/// Standard deviation of the gyroscope in rad/s.
const GYRO_SIGMA: f64 = 0.01;
/// Standard deviation of the angle estimate in rad.
const ANGLE_SIGMA: f64 = 0.002;
/// Standard deviation of the turbulence acting on the rate per sample, in rad/s.
const TURBULENCE_SIGMA: f64 = 0.002;
/// Variance of the angular acceleration from differenced gyroscope readings, in (rad/s²)².
const ACCEL_VARIANCE: f64 = 2.0 * GYRO_SIGMA * GYRO_SIGMA / (T_S * T_S);
/// Amplitude of the PRBS added to the stabilizer output, in N.
const PRBS_AMPLITUDE: f32 = 0.15;
/// Samples of the motor lag phase (20 s).
const LAG_SAMPLES: u32 = 10_000;
/// Samples of the input gain phase (20 s).
const GAIN_SAMPLES: u32 = 10_000;
/// A row of the identification log is written every this many samples (0.1 s).
const LOG_EVERY: u32 = 50;
/// Duration of the flight test in samples (1.5 s).
const STEP_SAMPLES: u32 = 750;
/// Output limit of the controllers in N.
const U_MAX: f64 = 1.0;
/// Candidate motor time constants in s. The true 25 ms is deliberately not on the grid.
const LAG_CANDIDATES: [f64; 8] = [0.010, 0.016, 0.022, 0.028, 0.034, 0.040, 0.050, 0.060];

/// Runs the bring-up and writes the CSV files.
fn main() -> Result<(), Box<dyn Error>> {
    let truth = AxisParameters::new(30.0, 0.025, T_S)?;
    let placeholder = AxisParameters::new(34.0, 0.03, T_S)?;
    let mut rig = Rig::new(&truth)?;
    let mut log = Vec::new();

    let lag = identify_lag(&mut rig, &mut log)?;
    println!(
        "motor lag: tau_m = {:.1} ms (b = {:.2} at the best candidate, edge: {})",
        lag.tau_m * 1e3,
        lag.b,
        lag.at_edge
    );

    let b = identify_gain(&mut rig, lag.tau_m, &mut log)?;
    println!("input gain: b = {b:.2} rad/s^2/N");
    write_csv(
        "bringup_ident",
        "t,a_arx,arx_identified,tau_search,b_gain,gain_identified,gain_excited",
        &log,
    )?;

    let identified = AxisParameters::new(b, lag.tau_m, T_S)?;
    let gains = design_axis(&identified, &Weights::default(), &Noise::default())?;
    let prior_gains = design_axis(&placeholder, &Weights::default(), &Noise::default())?;
    println!("gains: k = {:?}, ki = {:.2}", gains.k, gains.ki);

    let prior = fly_step(&truth, &prior_gains, &mut rig.noise)?;
    let tuned = fly_step(&truth, &gains, &mut rig.noise)?;
    let rows: Vec<[f64; 7]> = prior
        .iter()
        .zip(&tuned)
        .map(|(p, t)| [t[0], t[1], p[2], t[2], t[3], t[4], t[5]])
        .collect();
    write_csv("bringup_step", "t,r,theta_prior,theta,u,td,td_hat", &rows)?;

    Ok(())
}

/// The simulated airframe on its soft stabilizer, with the PRBS on the command.
struct Rig {
    /// The true discrete model.
    plant: DiscreteAxis,
    /// The state `[θ, ω, ΔT, u_prev]`.
    x: [f64; 4],
    /// Soft PD controller that keeps the axis level.
    stabilizer: IncrementalPid,
    /// Excitation signal.
    prbs: Prbs,
    /// Sensor and turbulence noise.
    noise: Gaussian,
}

impl Rig {
    /// Creates the rig at rest.
    fn new(truth: &AxisParameters) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            plant: truth.discretize(),
            x: [0.0; 4],
            stabilizer: IncrementalPid::new(&PidConfig {
                kp: 2.0,
                ki: 0.0,
                kd: 0.3,
                n: 10.0,
                u_max: U_MAX,
                sample_period: T_S,
            })?,
            prbs: Prbs::new(9, 4).ok_or("PRBS order 9 with hold 4 is valid")?,
            noise: Gaussian::new(42),
        })
    }

    /// Runs one sample: measures, computes the command, advances the plant. Returns the
    /// measured rate and the command, the pair the regressors need.
    fn sample(&mut self) -> (f32, f32) {
        let (theta, omega) = measure(&self.x, &mut self.noise);
        let u = self.stabilizer.step_with_derivative(0.0, theta, omega)
            + self.prbs.next_level(PRBS_AMPLITUDE);

        self.x = self.plant.step(&self.x, f64::from(u));
        self.x[1] += TURBULENCE_SIGMA * self.noise.sample();

        (omega, u)
    }
}

/// Phase 1: finds the motor time constant, and fits the three-parameter rate model on the
/// same data for comparison.
fn identify_lag(rig: &mut Rig, log: &mut Vec<[f64; 7]>) -> Result<LagEstimate, Box<dyn Error>> {
    let mut search = MotorLagSearch::new(LAG_CANDIDATES, T_S)?;

    let mut storage = EstimatorStorage::<3, f32>::new(0.0);
    let mut arx = storage.build(10.0);
    arx.set_forgetting_factor(0.9995);
    arx.set_measurement_variance(narrow(ACCEL_VARIANCE));
    let mut rate = RateRegressor::new(&AxisParameters::new(1.0, 0.03, T_S)?);
    let identifiability = Identifiability::new([0.01, 0.2, 0.2], 0.99);

    for k in 0..LAG_SAMPLES {
        let (omega, u) = rig.sample();
        search.push(omega, u);
        if let Some((phi, y)) = rate.push(omega, u) {
            // A rejected update (outlier gate, not set here) only skips the sample.
            let _applied = arx.update(&phi, y).is_applied();
        }

        if k % LOG_EVERY == LOG_EVERY - 1 {
            let status = identifiability.assess_rls(&arx).status;
            let tau = search.estimate().map_or(f64::NAN, |e| e.tau_m);
            log.push([
                time(k),
                arx_pole(&arx),
                flag(status == IdentifiabilityStatus::Identified),
                tau,
                f64::NAN,
                f64::NAN,
                f64::NAN,
            ]);
        }
    }

    let candidates: Vec<[f64; 3]> = search
        .candidates()
        .iter()
        .map(|&(tau, b, rms)| [tau, b, rms])
        .collect();
    write_csv("bringup_lag", "tau,b,rms", &candidates)?;

    println!(
        "rate model: a = {:.3} (true {:.3})",
        arx_pole(&arx),
        (-T_S / 0.025_f64).exp()
    );
    Ok(search
        .estimate()
        .ok_or("the PRBS excites the motor lag search")?)
}

/// The motor pole `a` of the three-parameter rate model.
fn arx_pole(rls: &BorrowedEstimator<'_, 3, f32>) -> f64 {
    f64::from(rls.parameters().get_row(0))
}

/// Phase 2: tracks the input gain with the found motor time constant. Returns the last
/// estimate after `miniident` has called it identified.
fn identify_gain(
    rig: &mut Rig,
    tau_m: f64,
    log: &mut Vec<[f64; 7]>,
) -> Result<f64, Box<dyn Error>> {
    let mut regressor = GainRegressor::new(&AxisParameters::new(1.0, tau_m, T_S)?);
    let mut storage = EstimatorStorage::<1, f32>::new(0.0);
    let mut rls = storage.build(1e4);
    rls.set_forgetting_factor(0.9999);
    rls.set_measurement_variance(narrow(ACCEL_VARIANCE));
    let mut excitation = ExcitationMonitor::for_rls(&rls, ExcitationThresholds::default());
    // ±1.5 rad/s²/N is about 5 % of b, well below what the controller is sensitive to.
    // shortcut: the RLS covariance assumes white noise in y; differenced gyro noise is not
    // white, so σ comes out about ten times too large here (see MINIIDENT_FINDINGS.md).
    let identifiability = Identifiability::new([1.5], 0.99);

    let mut identified = false;
    for k in LAG_SAMPLES..LAG_SAMPLES + GAIN_SAMPLES {
        let (omega, u) = rig.sample();
        if let Some((phi, y)) = regressor.push(omega, u) {
            // A rejected update (outlier gate, not set here) only skips the sample.
            let _applied = rls.update(&phi, y).is_applied();
            excitation.update(&phi);
        }

        if k % LOG_EVERY == LOG_EVERY - 1 {
            identified =
                identifiability.assess_rls(&rls).status == IdentifiabilityStatus::Identified;
            let excited = excitation.assess().status == ExcitationStatus::Sufficient;
            let b = f64::from(rls.parameters().get_row(0));
            log.push([
                time(k),
                f64::NAN,
                f64::NAN,
                f64::NAN,
                b,
                flag(identified),
                flag(excited),
            ]);
        }
    }

    if !identified {
        return Err("the input gain is not identified; excite longer or stronger".into());
    }

    Ok(f64::from(rls.parameters().get_row(0)))
}

/// Flies a step of 0.1 rad with `gains`, a `minikalman` observer and the state feedback
/// law. Returns rows `[t, r, θ, u, ΔT, ΔT̂]`.
fn fly_step(
    truth: &AxisParameters,
    gains: &AxisGains,
    noise: &mut Gaussian,
) -> Result<Vec<[f64; 6]>, Box<dyn Error>> {
    let plant = truth.discretize();
    let mut law = StateFeedback::new(gains, U_MAX, T_S)?;

    let mut observer_storage = ObserverStorage::<4, f32>::new(0.0);
    let mut measurement_storage = FixedGainObservationStorage::<4, 2, f32, Current>::new(0.0);
    let mut control_storage = ControlStorage::<4, 1, f32>::new(0.0);
    let mut observer = observer_storage.build();
    let mut measurement = measurement_storage.build();
    let mut control = control_storage.build();
    configure_observer(gains, &mut observer, &mut measurement, &mut control);

    let mut x = [0.0; 4];
    let mut rows = Vec::new();
    for k in 0..STEP_SAMPLES {
        let reference = if k >= 50 { 0.1 } else { 0.0 };
        let (theta, omega) = measure(&x, noise);

        observer.predict().control(&mut control);
        measurement.measurement_vector_mut().set_at(0, 0, theta);
        measurement.measurement_vector_mut().set_at(1, 0, omega);
        if !observer.correct(&mut measurement).is_applied() {
            return Err("the observer rejected a measurement".into());
        }

        let state = observer.state_vector();
        let x_hat = [0, 1, 2, 3].map(|i| state.get_row(i));
        let u = law.step(reference, &x_hat, theta);
        control.control_vector_mut().set_at(0, 0, u);

        rows.push([
            f64::from(k) * T_S,
            f64::from(reference),
            x[0],
            f64::from(u),
            x[2],
            f64::from(x_hat[2]),
        ]);
        x = plant.step(&x, f64::from(u));
        x[1] += TURBULENCE_SIGMA * noise.sample();
    }

    Ok(rows)
}

/// Copies `Φ`, `Γ`, `H` and the current-form gain into the `minikalman` buffers.
fn configure_observer(
    gains: &AxisGains,
    observer: &mut BorrowedLuenbergerObserver<'_, 4, f32>,
    measurement: &mut BorrowedFixedGainObservation<'_, 4, 2, f32>,
    control: &mut BorrowedControl<'_, 4, 1, f32>,
) {
    for (i, row) in gains.phi.iter().enumerate() {
        for (j, v) in row.iter().enumerate() {
            observer.state_transition_mut().set_at(i, j, *v);
        }
    }

    for (i, g) in gains.gamma.iter().enumerate() {
        control.control_matrix_mut().set_at(i, 0, *g);
    }

    measurement.observation_matrix_mut().set_at(0, 0, 1.0);
    measurement.observation_matrix_mut().set_at(1, 1, 1.0);
    let flat: [f32; 8] = std::array::from_fn(|i| gains.observer[i / 2][i % 2]);
    measurement.set_gain(&MatrixData::new_array::<4, 2, 8, f32>(flat));
}

/// The noisy angle and rate measurement of the state `x`.
fn measure(x: &[f64; 4], noise: &mut Gaussian) -> (f32, f32) {
    (
        narrow(x[0] + ANGLE_SIGMA * noise.sample()),
        narrow(x[1] + GYRO_SIGMA * noise.sample()),
    )
}

/// The time in s at the end of sample `k`.
fn time(k: u32) -> f64 {
    f64::from(k + 1) * T_S
}

/// 1.0 for true, 0.0 for false.
fn flag(value: bool) -> f64 {
    f64::from(u8::from(value))
}

/// Rounds a simulation value to the `f32` of the flight controller.
#[expect(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    reason = "simulated sensor values are far inside the f32 range; rounding is the intent"
)]
fn narrow(value: f64) -> f32 {
    value as f32
}

/// Writes `rows` with `header` to `typst/data/control/<name>.csv`.
fn write_csv<const C: usize>(
    name: &str,
    header: &str,
    rows: &[[f64; C]],
) -> Result<(), Box<dyn Error>> {
    let mut text = String::from(header);
    text.push('\n');
    for row in rows {
        let line: Vec<String> = row.iter().map(|v| format!("{v:.9}")).collect();
        writeln!(text, "{}", line.join(","))?;
    }

    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "..",
        "typst",
        "data",
        "control",
        &format!("{name}.csv"),
    ]
    .iter()
    .collect();
    std::fs::write(&path, text)?;
    println!("wrote {}", path.display());

    Ok(())
}

/// Seeded standard normal samples (xorshift64* and Box-Muller), to keep the example free of
/// further dependencies.
struct Gaussian(u64);

impl Gaussian {
    /// Creates the generator; a zero seed becomes 1, because xorshift stays at zero.
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    /// A uniform sample in `(0, 1)`, never 0, so that its logarithm is finite.
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        let x = self.0.wrapping_mul(0x2545_F491_4F6C_DD1D);
        (top_53_bits(x) + 0.5) / 9_007_199_254_740_992.0
    }

    /// A standard normal sample.
    fn sample(&mut self) -> f64 {
        let (u1, u2) = (self.uniform(), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

/// The 53 high bits of `x` as an `f64`.
#[expect(
    clippy::as_conversions,
    clippy::cast_precision_loss,
    reason = "x >> 11 is below 2^53, so f64 represents it exactly"
)]
fn top_53_bits(x: u64) -> f64 {
    (x >> 11) as f64
}
