//! Twiddle: coordinate descent with adaptive step sizes, as taught by Sebastian Thrun.
//!
//! Each parameter is tried one step up and, if that does not lower the cost, one step down.
//! A successful step grows by 10 %, a failed one shrinks by 10 %. The search needs no
//! gradient and finds a local minimum. Run it against a simulated plant only: every cost
//! evaluation is a full experiment, and some of the parameters it tries are unstable.

/// The limits of a [`twiddle`] search.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// The search stops when the sum of the step sizes falls below this value.
    pub tolerance: f64,
    /// The search stops after this many sweeps over all parameters. Each sweep evaluates
    /// the cost at most `2·N` times.
    pub max_sweeps: u32,
}

/// The result of a [`twiddle`] search.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outcome<const N: usize> {
    /// The best parameters found.
    pub params: [f64; N],
    /// The cost of [`Outcome::params`].
    pub cost: f64,
    /// The number of sweeps done.
    pub sweeps: u32,
    /// True if the step sizes fell below the tolerance, false if the sweep limit stopped
    /// the search.
    pub converged: bool,
}

/// Searches the parameters that minimize `cost`, starting at `params` with the initial step
/// sizes `steps`.
///
/// A cost of NaN counts as worse than every other cost, so that unstable trials are never
/// kept.
pub fn twiddle<const N: usize>(
    mut params: [f64; N],
    mut steps: [f64; N],
    mut cost: impl FnMut(&[f64; N]) -> f64,
    limits: Limits,
) -> Outcome<N> {
    let mut best = sanitize(cost(&params));
    let mut sweeps = 0;
    while steps.iter().sum::<f64>() > limits.tolerance && sweeps < limits.max_sweeps {
        sweeps += 1;
        for (i, step) in steps.iter_mut().enumerate() {
            best = try_coordinate(&mut params, step, i, best, &mut cost);
        }
    }

    Outcome {
        params,
        cost: best,
        sweeps,
        converged: steps.iter().sum::<f64>() <= limits.tolerance,
    }
}

/// Tries parameter `i` one step up, then one step down, keeps the better value and adapts
/// the step. Returns the new best cost.
fn try_coordinate<const N: usize>(
    params: &mut [f64; N],
    step: &mut f64,
    i: usize,
    best: f64,
    cost: &mut impl FnMut(&[f64; N]) -> f64,
) -> f64 {
    let start = params[i];
    for candidate in [start + *step, start - *step] {
        params[i] = candidate;
        let c = sanitize(cost(params));
        if c < best {
            *step *= 1.1;
            return c;
        }
    }

    params[i] = start;
    *step *= 0.9;

    best
}

/// Maps NaN to infinity, so that it never compares as an improvement.
fn sanitize(cost: f64) -> f64 {
    if cost.is_nan() { f64::INFINITY } else { cost }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Twiddle finds the minimum of a bowl.
    #[test]
    fn finds_the_bottom_of_a_bowl() {
        let target = [1.0, -2.0, 3.0];
        let out = twiddle(
            [0.0; 3],
            [1.0; 3],
            |p| p.iter().zip(&target).map(|(a, b)| (a - b) * (a - b)).sum(),
            Limits {
                tolerance: 1e-6,
                max_sweeps: 1000,
            },
        );

        assert!(out.converged);
        for (p, t) in out.params.iter().zip(&target) {
            assert!((p - t).abs() < 1e-3, "{:?}", out.params);
        }
    }
}
