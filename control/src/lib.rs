//! Controller design and plant identification for one attitude axis of a quadrotor.
//!
//! The crate is the practical side of the chapters "State-space control" and "Identifying
//! the plant" in `typst/`. It fills the gap between the estimators of
//! [`minikalman`] and the identification diagnostics of `miniident` on one side and a
//! running attitude controller on the other:
//!
//! * [`model`] describes the axis (input gain, motor lag, sample period) and discretizes it.
//! * [`ident`] builds the regressors for recursive least squares, and [`model`] turns the
//!   estimated parameters back into a model.
//! * [`design`] computes the gains: pole placement, LQR with integral action, and the
//!   observer gain. It runs in `f64`, once, after identification.
//! * [`law`] and [`pid`] are the controllers that run every sample, in `f32`.
//! * [`twiddle`] searches controller parameters against a simulated plant.
//!
//! The crate is `no_std` and allocates nothing.
//!
//! ```mermaid
//! flowchart LR
//!     gyro[gyro, attitude] --> ident
//!     ident --> rls[minikalman RLS]
//!     rls --> check[miniident checks]
//!     check --> model
//!     model --> design
//!     design --> observer[minikalman observer]
//!     design --> law
//!     observer --> law
//!     law --> motors
//! ```
#![no_std]
#![forbid(unsafe_code)]

pub mod design;
pub mod ident;
pub mod law;
pub mod linalg;
pub mod model;
pub mod pid;
pub mod twiddle;

/// The ways a model, a design or a controller setup can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// A parameter is NaN, infinite or outside its valid range. The value names the
    /// parameter.
    InvalidParameter(&'static str),
    /// The input cannot move all states, so the poles cannot be placed.
    NotControllable,
    /// A matrix that the design must invert is singular or too badly conditioned.
    Singular,
    /// The Riccati iteration did not converge within its iteration limit.
    NotConverged,
    /// The requested poles do not come in complex-conjugate pairs, so no real gain places
    /// them.
    NotConjugate,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidParameter(name) => write!(f, "parameter `{name}` is out of range"),
            Self::NotControllable => f.write_str("the system is not controllable"),
            Self::Singular => f.write_str("a matrix in the design is singular"),
            Self::NotConverged => f.write_str("the Riccati iteration did not converge"),
            Self::NotConjugate => f.write_str("the poles are not complex-conjugate pairs"),
        }
    }
}

impl core::error::Error for Error {}

/// Returns `value` if it is finite and greater than zero, else
/// [`Error::InvalidParameter`] with `name`.
pub(crate) fn positive(value: f64, name: &'static str) -> Result<f64, Error> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(Error::InvalidParameter(name))
    }
}

/// Converts a design result from `f64` to the `f32` of the control loop.
///
/// The gains and model entries of one attitude axis are far inside the `f32` range, so the
/// conversion only rounds.
#[expect(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    reason = "rounding to f32 is the intent; design values are far inside the f32 range"
)]
#[must_use]
pub(crate) fn to_f32(value: f64) -> f32 {
    value as f32
}
