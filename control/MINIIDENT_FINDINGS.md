# miniident findings

Notes from using `miniident` 0.1.0-alpha.2 (with `minikalman` 0.9.1) for the attitude-axis
bring-up in `examples/bringup.rs`. Each item has what happened, why, and a suggestion.
Reproduce with `cargo run --release --example bringup`.

## 1. Identifiability cannot see bias

**What happened.** The three-parameter rate model (`ident::RateRegressor`, regressor
`[α[k], u[k], u[k−1]]` with `α` the differenced gyro rate) converges to a motor pole of
`a ≈ −0.41` where the truth is `0.92`. The gyroscope noise sits in the regressor
(errors-in-variables), and differencing makes it MA(1) with lag-1 correlation −0.5, which
pulls `a` toward −0.5. `Identifiability` judges the covariance only, so a tighter or looser
tolerance decides whether a confidently wrong estimate is reported as `Identified`. In this
run it stays `Unidentified` (tolerance 0.01 on `a`), but only because of the variance.

**Suggestion.** A model-validation check next to the covariance check: the normalized
autocorrelation of the innovations at a few lags (a Ljung–Box style statistic), and the
cross-correlation between innovations and past inputs. Here, the lag-1 autocorrelation of
the residuals is strongly negative and would flag the model. Cheap online: a handful of
running sums. Could be a `ResidualMonitor` beside `ExcitationMonitor`.

## 2. RLS covariance under colored measurement noise

**What happened.** For the one-parameter gain model (`ident::GainRegressor`), the RLS
standard deviation of `b` is about 2 rad/s²/N after 10 s and still about 1.5 after 15 s,
while the estimate itself varies with a standard deviation of only 0.12 (29.7 to 30.2)
between 25 and 40 s, i.e. the reported σ is about ten times too large. The output `y` is a differenced gyro reading, so its noise is MA(1);
against a smooth regressor, consecutive noise samples largely cancel, and the white-noise
covariance overstates the variance several times. The bring-up therefore runs 20 s and
accepts ±1.5 (5 % of `b`).

**Suggestion.** Document that `Identifiability` inherits the white-noise assumption of the
estimator. Optionally accept an effective noise variance or a known noise model (e.g. the
MA(1) of a differenced sensor), or estimate the variance of `θ̂` empirically from a
sliding window as a cross-check.

## 3. Scale of the excitation thresholds

**What happened.** With the regressor in per-sample units (rate differences of ~1e-4 rad/s,
thrusts times `T_s`), `ExcitationMonitor` reports `Insufficient` with a minimum eigenvalue
of 1.6e-8, against the default `min_eigenvalue = 1e-3`, although the data identify `b`
well. After rescaling the regressor to physical units (angular acceleration, thrust in N),
the same data are `Sufficient` at 4e-3.

**Status.** The doc comment of `ExcitationThresholds::min_eigenvalue` says the default
assumes regressors of unit amplitude, so this is documented. It is still easy to miss.

**Suggestion.** Mention the scaling in the crate-level docs and the README quick start,
or offer thresholds relative to an expected regressor power (`ExcitationThresholds::for_scale`)
or a debug-time warning when the regressor power is orders of magnitude below 1.

## 4. Input delay and model structure

**What happened.** The regressors here hard-code one sample of input delay. Neither crate
helps to find the delay or to choose between model structures (e.g. ARX order, delay 1 vs
2); a wrong delay shows up only as a poor fit.

**Suggestion.** A small helper that runs a bank of candidate structures (as
`ident::MotorLagSearch` does for the motor time constant) and ranks them by residual
variance or an information criterion (AIC/BIC from the RLS residual sums). The bank
pattern would also cover the motor-lag grid search, which currently lives in this crate.

## 5. Small API notes

- `RecursiveLeastSquares::update` returns a `CorrectionStatus`; in a loop without a gate,
  the only use is to discard it. Fine, but an example of what to do with a rejection would
  help.
- Reading `θ̂` needs `parameters().get_row(i)` per entry. A `parameters_array() -> [T; N]`
  convenience would shorten the glue code.
- `Prbs::next_level(amplitude)` is exactly what closed-loop identification needs (an offset
  is added by the stabilizer); worked well.
