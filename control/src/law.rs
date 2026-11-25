//! The state feedback law with integral action, as it runs every sample.
//!
//! The law takes the state estimate from an observer (e.g. a [`minikalman`]
//! `LuenbergerObserver` with the gain of [`AxisGains::observer`]) and the measured angle,
//! and returns the saturated thrust difference command:
//!
//! ```text
//! u = sat(−K·(x̂ − [r, 0, 0, 0]) + k_i·x_i),   x_i ← x_i + T_s·(r − θ)
//! ```
//!
//! The integral stops while the output is saturated in the direction the integral pushes
//! (conditional integration), so that it cannot wind up.

use crate::design::AxisGains;
use crate::model::STATES;
use crate::{Error, positive, to_f32};

/// State feedback with integral action and anti-windup for one axis.
#[derive(Debug, Clone, PartialEq)]
pub struct StateFeedback {
    /// State feedback gain on `[θ − r, ω, ΔT, u_prev]`.
    k: [f32; STATES],
    /// Integral gain in N/(rad·s).
    ki: f32,
    /// Largest output magnitude in N. Always finite and positive.
    u_max: f32,
    /// Sample period in s. Always finite and positive.
    sample_period: f32,
    /// Integral of the angle error in rad·s.
    integral: f32,
}

impl StateFeedback {
    /// Creates the law from designed gains, the output limit `u_max` (N) and the sample
    /// period (s).
    ///
    /// # Errors
    ///
    /// [`Error::InvalidParameter`] if `u_max` or `sample_period` is not finite and positive.
    pub fn new(gains: &AxisGains, u_max: f64, sample_period: f64) -> Result<Self, Error> {
        Ok(Self {
            k: gains.k,
            ki: gains.ki,
            u_max: to_f32(positive(u_max, "u_max")?),
            sample_period: to_f32(positive(sample_period, "sample_period")?),
            integral: 0.0,
        })
    }

    /// Computes the command for the reference `reference` (rad), the state estimate `x_hat`
    /// and the measured angle `theta` (rad).
    ///
    /// A NaN in the inputs gives a NaN command; the caller must treat it as a sensor fault.
    pub fn step(&mut self, reference: f32, x_hat: &[f32; STATES], theta: f32) -> f32 {
        let error = reference - theta;
        let integral = self.integral + self.sample_period * error;
        let feedback = self.feedback(reference, x_hat);

        let unsaturated = feedback + self.ki * integral;
        let u = unsaturated.clamp(-self.u_max, self.u_max);
        if !self.winds_up(u, unsaturated, error) {
            self.integral = integral;
        }

        u
    }

    /// The integral of the angle error in rad·s.
    #[must_use]
    pub fn integral(&self) -> f32 {
        self.integral
    }

    /// Clears the integral, e.g. before take-off.
    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    /// The proportional part `−K·(x̂ − [r, 0, 0, 0])`.
    fn feedback(&self, reference: f32, x_hat: &[f32; STATES]) -> f32 {
        let mut offset = *x_hat;
        offset[0] -= reference;

        -self.k.iter().zip(&offset).map(|(k, x)| k * x).sum::<f32>()
    }

    /// True if the output saturates and the integral pushes further into saturation.
    fn winds_up(&self, u: f32, unsaturated: f32, error: f32) -> bool {
        let saturated = (u - unsaturated).abs() > 0.0;
        saturated && unsaturated.signum() == (self.ki * error).signum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gains with only a proportional and an integral part, for hand-checked numbers.
    fn gains() -> AxisGains {
        AxisGains {
            k: [10.0, 1.0, 0.0, 0.0],
            ki: 50.0,
            observer: [[0.0; 2]; STATES],
            phi: [[0.0; STATES]; STATES],
            gamma: [0.0; STATES],
        }
    }

    /// Without saturation, the law is `−K·(x̂ − x_ref) + k_i·x_i`.
    #[test]
    fn unsaturated_output() {
        let mut law = StateFeedback::new(&gains(), 10.0, 0.002).expect("valid limits");
        let u = law.step(0.1, &[0.0, 0.5, 0.0, 0.0], 0.0);
        let expected = 10.0 * 0.1 - 0.5 + 50.0 * 0.002 * 0.1;
        assert!((u - expected).abs() < 1e-6, "{u} != {expected}");
    }

    /// While saturated in the direction of the error, the integral holds.
    #[test]
    fn integral_holds_in_saturation() {
        let mut law = StateFeedback::new(&gains(), 0.5, 0.002).expect("valid limits");
        for _ in 0..100 {
            let u = law.step(1.0, &[0.0; STATES], 0.0);
            assert!((u - 0.5).abs() < f32::EPSILON);
        }

        assert!(law.integral().abs() < f32::EPSILON);
    }
}
