//! A PID controller in incremental (velocity) form, for the microcontroller.
//!
//! The controller computes the change of the output and adds it to the previous output:
//!
//! ```text
//! Δu = K_p·(e[k] − e[k−1]) + K_i·T_s·e[k] + (D[k] − D[k−1])
//! u  = sat(u[k−1] + Δu)
//! ```
//!
//! The output is the integrator, and it is saturated every sample, so the controller cannot
//! wind up. The derivative acts on the measurement, not on the error, so that a reference
//! step does not kick the output. It is low-pass filtered with `T_f = K_d / (K_p·N)`. All
//! coefficients are computed once in [`IncrementalPid::new`].

use crate::{Error, positive, to_f32};

/// The gains and limits of a [`IncrementalPid`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidConfig {
    /// Proportional gain. Finite and positive.
    pub kp: f64,
    /// Integral gain (per second). Finite and not negative.
    pub ki: f64,
    /// Derivative gain (seconds). Finite and not negative.
    pub kd: f64,
    /// Derivative filter ratio `N`, typically 5 to 20. Finite and positive.
    pub n: f64,
    /// Largest output magnitude. Finite and positive.
    pub u_max: f64,
    /// Sample period in s. Finite and positive.
    pub sample_period: f64,
}

/// A PID controller in incremental form with the derivative on the measurement.
#[derive(Debug, Clone, PartialEq)]
pub struct IncrementalPid {
    /// Proportional gain `K_p`.
    kp: f32,
    /// Integral coefficient `K_i·T_s`.
    ki_t: f32,
    /// Derivative filter pole `α = T_f / (T_f + T_s)`.
    alpha: f32,
    /// Derivative coefficient `β = K_d / (T_f + T_s)`.
    beta: f32,
    /// Derivative gain `K_d`, for a measured derivative.
    kd: f32,
    /// Largest output magnitude.
    u_max: f32,
    /// The error of the previous sample.
    e_prev: f32,
    /// The measurement of the previous sample; `None` before the first sample.
    y_prev: Option<f32>,
    /// The derivative term of the previous sample.
    d_prev: f32,
    /// The output of the previous sample.
    u: f32,
}

impl IncrementalPid {
    /// Creates the controller and precomputes its coefficients.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidParameter`] if a value of `config` is out of its range.
    pub fn new(config: &PidConfig) -> Result<Self, Error> {
        let kp = positive(config.kp, "kp")?;
        let ki = non_negative(config.ki, "ki")?;
        let kd = non_negative(config.kd, "kd")?;
        let t = positive(config.sample_period, "sample_period")?;
        let tf = kd / kp / positive(config.n, "n")?;

        Ok(Self {
            kp: to_f32(kp),
            ki_t: to_f32(ki * t),
            alpha: to_f32(tf / (tf + t)),
            beta: to_f32(kd / (tf + t)),
            kd: to_f32(kd),
            u_max: to_f32(positive(config.u_max, "u_max")?),
            e_prev: 0.0,
            y_prev: None,
            d_prev: 0.0,
            u: 0.0,
        })
    }

    /// Computes the output for the reference `reference` and the measurement `y`, with the
    /// derivative of `y` estimated by the filtered difference.
    pub fn step(&mut self, reference: f32, y: f32) -> f32 {
        let y_prev = self.y_prev.unwrap_or(y);
        let d = self.alpha * self.d_prev - self.beta * (y - y_prev);
        self.y_prev = Some(y);

        self.update(reference - y, d)
    }

    /// Computes the output for the reference `reference`, the measurement `y` and its
    /// measured derivative `dy`, e.g. the angle and the gyroscope rate.
    ///
    /// The derivative term is `−K_d·dy`, unfiltered: the gyroscope already measures the
    /// derivative, and its noise is lower than that of a difference of angles.
    pub fn step_with_derivative(&mut self, reference: f32, y: f32, dy: f32) -> f32 {
        self.y_prev = Some(y);
        let d = -self.kd * dy;

        self.update(reference - y, d)
    }

    /// The output of the last sample.
    #[must_use]
    pub fn output(&self) -> f32 {
        self.u
    }

    /// Applies one increment with the error `e` and the derivative term `d`.
    fn update(&mut self, e: f32, d: f32) -> f32 {
        let du = self.kp * (e - self.e_prev) + self.ki_t * e + (d - self.d_prev);
        self.u = (self.u + du).clamp(-self.u_max, self.u_max);
        self.e_prev = e;
        self.d_prev = d;

        self.u
    }
}

/// Returns `value` if it is finite and not negative, else [`Error::InvalidParameter`].
fn non_negative(value: f64, name: &'static str) -> Result<f64, Error> {
    if value.is_finite() && value >= 0.0 {
        Ok(value)
    } else {
        Err(Error::InvalidParameter(name))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A rate PID with the SIMC gains of the reference airframe.
    fn config() -> PidConfig {
        PidConfig {
            kp: 1.696,
            ki: 13.9,
            kd: 0.038,
            n: 10.0,
            u_max: 1.0e6,
            sample_period: 0.002,
        }
    }

    /// Positional form with the same coefficients, as the reference for the test.
    struct Positional {
        /// The incremental controller whose coefficients are reused.
        c: IncrementalPid,
        /// Integral term.
        i: f32,
        /// Derivative term.
        d: f32,
        /// Previous measurement.
        y_prev: f32,
    }

    impl Positional {
        /// One positional step.
        fn step(&mut self, r: f32, y: f32) -> f32 {
            let e = r - y;
            self.i += self.c.ki_t * e;
            self.d = self.c.alpha * self.d - self.c.beta * (y - self.y_prev);
            self.y_prev = y;

            self.c.kp * e + self.i + self.d
        }
    }

    /// Without saturation, the incremental and the positional form give the same output.
    #[test]
    fn incremental_matches_positional() {
        let mut inc = IncrementalPid::new(&config()).expect("valid config");
        let mut pos = Positional {
            c: inc.clone(),
            i: 0.0,
            d: 0.0,
            y_prev: 0.0,
        };

        let mut y = 0.0_f32;
        for k in 0..500_u16 {
            let r = if k > 10 { 1.0 } else { 0.0 };
            let a = inc.step(r, y);
            let b = pos.step(r, y);
            assert!((a - b).abs() < 1e-3, "sample {k}: {a} != {b}");
            y += 0.002 * (a - y);
        }
    }

    /// The output never leaves the limit, and recovers at once when the error changes sign.
    #[test]
    fn output_is_saturated_without_windup() {
        let mut pid = IncrementalPid::new(&PidConfig {
            u_max: 0.1,
            ..config()
        })
        .expect("valid config");
        for _ in 0..1000 {
            assert!(pid.step(10.0, 0.0) <= 0.1);
        }

        assert!(pid.step(-1.0, 0.0) < 0.1);
    }

    /// Out-of-range gains are rejected.
    #[test]
    fn invalid_gains_are_rejected() {
        let bad = PidConfig {
            ki: -1.0,
            ..config()
        };
        assert_eq!(
            IncrementalPid::new(&bad),
            Err(Error::InvalidParameter("ki"))
        );
    }
}
