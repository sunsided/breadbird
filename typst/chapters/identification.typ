// Chapter 4: identifying the plant (new). Numbers and figures come from the
// bring-up example of the control crate (control/examples/bringup.rs) against
// a simulated airframe.
#import "../commands.typ": *
#import "../figures/control/identification.typ": ident-setup, ident-trajectories, lag-residual, bringup-step

= Identifying the plant <cha:identification>
#context set-lhead[Chapter #counter(heading).get().first(). _Identifying the plant_]

Every design of @cha:control rests on two numbers: the input gain $b$, which says how strongly a thrust difference accelerates the frame, and the motor time constant $tau_m$, which says how quickly the thrust follows a command.
Both can be estimated from geometry and datasheets, but not well; the moment of inertia of a frame with battery, cables and electronics in particular is hard to compute.
_System identification_#index("system identification") measures them instead: it applies a known input to the vehicle, records how it responds, and fits the model to the data.

This chapter does that for one axis, in a way that can run on the flight controller itself.
Without sensors and actuators at hand, it is demonstrated on a simulated airframe whose parameters differ from the reference airframe of @tab:reference_airframe ($b = #qty("30", "rad/s^2/N")$ instead of 34, $tau_m = #qty("25", "ms")$ instead of 30), with the same sensor noise; the simulation knows the true values, so the estimates can be checked.

== What there is to identify <sec:ident_what>

The model of @sec:state_space has three numbers: $b$, $tau_m$ and the delay of one sample.
The delay is a property of the software (when the sensors are read and when the motor command goes out) and can be checked with a logic analyzer, so only $b$ and $tau_m$ remain.

On a real vehicle, the input is not a thrust difference in newtons, but a command to the motor controllers.
Around the hover point, this makes no difference: with @eq:thrust_coefficient, a small change $delta n$ of the rotor speed around the hover speed $n_0$ changes the thrust by $delta T approx 2 C_T rho n_0 D^4 delta n$, which is linear in $delta n$, and the motor controllers map their command to a speed in roughly the same linear way.
The identified $b$ then has the unit of an angular acceleration per command unit, it absorbs all these factors, and the limits $u_max$ of the controller and of Bryson's rule have to be given in the same unit.
The motor time constant also depends on the operating point, since a faster rotor has more aerodynamic load; both values should therefore be identified at hover, where the controller spends its time.

Pitch works exactly like roll, with its own moment of inertia.
Yaw is driven by the reaction torque of @eq:torque_yaw instead of a thrust difference; its model has the same structure, but its $b$ is much smaller and has to be identified separately.

== Least squares, recursively <sec:rls>

Most identification methods start from a model that is linear in its unknown parameters,

$ y[k] = bold(phi)[k]^top bold(theta) + v[k] $ <eq:linear_regression>

where $y$ is a measured output, the _regressor_#index("regressor") $bold(phi)$ a vector of measured or known quantities, $bold(theta)$ the vector of unknown parameters and $v$ the noise.
The model of the attitude axis is linear in neither $b$ nor $tau_m$ as it stands, but the following sections bring it into this form.
_Least squares_#index("least squares") chooses the $hat(bold(theta))$ that minimizes the sum of the squared errors $sum (y[k] - bold(phi)[k]^top hat(bold(theta)))^2$ over all samples.

On a flight controller, the samples cannot be stored and solved at the end.
_Recursive least squares_#index("recursive least squares") (RLS) updates the estimate with each new sample instead:

$ epsilon &= y[k] - bold(phi)[k]^top hat(bold(theta)) \
  bold(k) &= (bold(P) bold(phi)[k]) / (mu + bold(phi)[k]^top bold(P) bold(phi)[k]) \
  hat(bold(theta)) &<- hat(bold(theta)) + bold(k) epsilon \
  bold(P) &<- (bold(P) - bold(k) bold(phi)[k]^top bold(P)) \/ mu $ <eq:rls>

The first line is the _prediction error_, what the current estimate gets wrong about the new sample; the third corrects the estimate by a fraction $bold(k)$ of it.
$bold(P)$ measures how uncertain the estimate still is: with white noise of variance $r$, $r bold(P)$ is the covariance of $hat(bold(theta))$, the square roots of its diagonal are the standard deviations of the parameters, and the gain $bold(k)$ is large while $bold(P)$ is large and small once the estimate has settled.
This is a Kalman filter for a state that does not move, and the update has the same structure as @eq:observer.
The _forgetting factor_#index("forgetting factor") $mu$ slightly below one makes old samples count less, by $mu$ per sample, so that the estimate can follow slow changes: with $mu = 0.9999$ at #qty("500", "Hz"), the memory spans about $1 \/ (1 - mu) = 10^4$ samples, or #qty("20", "s").

The estimator in this chapter is the one of the `minikalman` library @minikalman, and the checks of the following sections come from `miniident` @miniident, both written for microcontrollers without dynamic memory.

== Exciting the plant <sec:excitation>

Least squares can only find what the data contain.
If the input never changes, the regressor always points in the same direction, and any parameter combination that explains this one direction fits equally well.
The data must therefore vary in every direction of the parameter space, which is called _persistent excitation_#index("persistent excitation").
Whether they do shows in the _information matrix_ $sum mu^(n - k) bold(phi)[k] bold(phi)[k]^top$, which grows with every sample in the directions the regressor points to: its smallest eigenvalue is the information about the worst-determined parameter combination, and its eigenvector says which combination that is.

A good test input is a _pseudo-random binary sequence_#index("PRBS") (PRBS), a signal that jumps between $+A$ and $-A$ in a pattern that looks random but repeats exactly.
It is generated by a shift register of a given order with feedback, and each value is held for a few samples.
With order 9 and a hold of 4 samples at #qty("500", "Hz"), the sequence repeats after $(2^9 - 1) dot 4 = 2044$ samples, or #qty("4.1", "s"), and its power is spread evenly up to about #qty("40", "Hz"), falling to zero at $500 \/ 4 = #qty("125", "Hz")$.
That covers the motor corner frequency $1 \/ (2 pi tau_m) approx #qty("6.4", "Hz")$ with room to spare.
The amplitude is a compromise between the signal, which has to stand out from the noise, and the motion it causes; here it is $A = #qty("0.15", "N")$, 15 % of the largest thrust difference.

== Where to excite <sec:where_to_excite>

An attitude axis cannot simply be excited and watched: the plant is unstable (@fig:open_loop), and a free vehicle would tumble within a second.
There are two ways around this.

The first is a test rig that lets the vehicle rotate about one axis only, e.g. a hinge or two bearings.
The pivot must pass through the center of gravity; otherwise, gravity adds a restoring or tipping torque $m g d sin theta$ for an offset $d$, and the model gains a term in $theta$ that the real vehicle does not have.
A vehicle hanging from strings is a pendulum for the same reason.
The rig also adds its own moment of inertia and bearing friction to the measurement, and even on a good hinge the axis is still a double integrator that drifts against the end stops.

The second, used here, is to identify in closed loop (@fig:ident_setup).
A deliberately soft controller keeps the vehicle roughly level, here a PD controller with $K_p = #qty("2", "N/rad")$ and $K_d = #qty("0.3", "N s/rad")$, an eighth and a fifth of the LQR gains of @cha:control.
It is safe by a wide margin: for this plant, a PD loop is stable for any $b > 0$ as long as $tau_m < K_d \/ K_p = #qty("0.15", "s")$ (ignoring the delay of one sample, which changes little at #qty("500", "Hz")), six times the expected motor time constant.
The PRBS is added to its output, and the identification sees the sum $u$ that actually goes to the motors.
The catch is that feedback correlates the input with the noise: the controller reacts to the measured noise, so the input carries a copy of it, which can bias least squares.
@sec:gain_regressor shows under which condition it does not.

#figure(
  placement: auto,
  {
    ident-setup()
    caption-rule
  },
  caption: flex-caption(
    [Identification in closed loop],
    [Identification in closed loop: a soft stabilizer keeps the axis level, a PRBS on its output excites it, and the regressor turns the command $u$ and the gyroscope rate $omega$ into the data for the RLS estimator, whose estimate $hat(bold(theta))$ and uncertainty $bold(P)$ are checked for excitation and identifiability.],
  ),
) <fig:ident_setup>

== A first model, and why it fails <sec:arx>

The angle is the integral of the rate, so it carries no information of its own, and identification can work on the gyroscope alone.
From the discrete model @eq:axis_zoh, the _angular acceleration_ $alpha[k] = (omega[k + 1] - omega[k]) \/ T_s$, i.e. the difference of two gyroscope readings, obeys

$ alpha[k + 1] = a alpha[k] + beta_0 u[k] + beta_1 u[k - 1] $ <eq:rate_arx>

with the motor pole $a = e^(-T_s \/ tau_m)$ and two coefficients $beta_0$ and $beta_1$ that depend on $b$ and $tau_m$, with $beta_0 + beta_1 = (1 - a) b$.
This is of the form @eq:linear_regression with $bold(theta) = (a, beta_0, beta_1)$ and $bold(phi)[k] = (alpha[k], u[k], u[k - 1])$, and $tau_m = -T_s \/ ln a$ and $b = (beta_0 + beta_1) \/ (1 - a)$ follow from the estimate.

It does not work.
The top of @fig:ident_trajectories shows the estimate of the motor pole: it settles within a second, but at $hat(a) approx -0.41$ instead of 0.92, which is not a motor lag at all.
The reason is the gyroscope noise.
Each $alpha$ is the difference of two readings, so the noise of the reading $omega[k + 1]$ appears in $alpha[k]$ with a plus sign and in $alpha[k + 1]$ with a minus sign.
The regressor $alpha[k]$ and the output $alpha[k + 1]$ thus share a noise term with opposite signs, and since the noise of a gyroscope at #qty("500", "Hz") is larger than the change of the rate from one sample to the next, this correlation dominates the fit and pulls $hat(a)$ towards $-0.5$.
Least squares assumes that the regressor is exact and only the output is noisy; a noisy regressor biases it (_errors in variables_#index("errors in variables")), and more data do not help, they only make the wrong answer more certain.

#figure(
  placement: auto,
  {
    ident-trajectories()
    caption-rule
  },
  caption: flex-caption(
    [Estimates during the identification run],
    [Estimates during the identification run, true values dashed. Top: the motor pole of the three-parameter model @eq:rate_arx converges, but to the wrong value. Middle: the motor time constant from the grid search of @sec:lag_search. Bottom: the input gain from the one-parameter model, which the identifiability check accepts from the dotted line on.],
  ),
) <fig:ident_trajectories>

== A model without noisy regressors <sec:gain_regressor>

The way out is a regressor that contains no measurement at all.
If $tau_m$ is known, the thrust difference that the commands produce can be computed from the commands alone, by running the motor lag of @eq:axis_zoh on them:

$ s[k + 1] = a s[k] + (1 - a) u[k - 1] $ <eq:thrust_model>

The angular acceleration over one sample is then $b$ times the average thrust difference during that sample,

$ alpha[k] = b dot (tau_m (1 - a) / T_s s[k] + (1 - tau_m (1 - a) / T_s) u[k - 1]) $ <eq:gain_regression>

which is @eq:linear_regression with the single parameter $theta = b$ and a regressor $phi[k]$, the bracket, in newtons.

The regressor is made of commands only, and commands are known exactly.
The output $alpha[k]$ contains the gyroscope noise of the readings $k$ and $k + 1$, while the newest command in $phi[k]$ is $u[k - 1]$, computed from readings up to $k - 1$.
With white sensor noise, the two are therefore uncorrelated even though a controller closes the loop, and least squares is unbiased.
This argument rests on the delay: if the command took effect within the same sample, $u[k]$ would appear in $phi[k]$ and carry the noise of reading $k$.
A one-sample delay that is real and accounted for is what makes closed-loop identification work here, which is one more reason to check it with a logic analyzer.

The argument also assumes that the sensor noise is independent of the commands.
On a real quadrotor, much of what the gyroscope picks up is vibration from the propellers, which follows the rotor speeds and with them the commands.
The gyroscope signal is therefore usually low-pass or notch filtered before anything uses it; if the identification works on the filtered rate, the same filter has to be applied to the regressor, which keeps @eq:gain_regression linear in $b$ and the two sides consistent.

== Finding the motor time constant <sec:lag_search>

@eq:gain_regression needs $tau_m$, which is the other unknown.
It can be measured on the thrust stand of @subsec:sensing, by stepping the motor command at hover speed and recording the rotor speed.
It can also be found from the flight data: for each candidate $tau_m$ of a grid, fit $b$ by least squares and compute how much of $alpha$ the fit leaves unexplained.
The candidate with the smallest residual is the best one, and a parabola through it and its two neighbors refines the estimate between the grid points.
Each candidate costs three running sums, so a grid of eight costs a few dozen multiplications per sample.

@fig:lag_residual shows the result after #qty("20", "s"), for a grid from 10 to #qty("60", "ms") that deliberately does not contain the true #qty("25", "ms").
The residual is smallest between 22 and #qty("28", "ms"), and the parabola puts the estimate at #qty("24.8", "ms"); the middle of @fig:ident_trajectories shows it within 1.3 ms of the truth from #qty("5", "s") on.
Over five runs with different noise, the final estimate lies between 24.8 and #qty("25.3", "ms"), and the running estimate stays within 2.1 ms of the truth after the first #qty("5", "s").
The minimum is shallow: almost all of the residual of about #qty("7.18", "rad/s^2") is gyroscope noise, whose share alone is $sqrt(2) sigma_omega \/ T_s approx #qty("7.07", "rad/s^2")$.
The bottom of @fig:lag_residual shows why $tau_m$ still matters: the fitted $b$ depends on it strongly, from 28.7 at #qty("22", "ms") to 31.9 at #qty("28", "ms"), since a slower motor has to be compensated by a stronger one to explain the same data.
If the search ends at the first or last grid point, the true value may lie outside the grid, which has to be widened.

#figure(
  placement: auto,
  {
    lag-residual()
    caption-rule
  },
  caption: flex-caption(
    [Grid search for the motor time constant],
    [Grid search for the motor time constant after #qty("20", "s") of data. Top: RMS residual of the one-parameter fit for each assumed $tau_m$. Bottom: the fitted input gain $b$. True values dashed.],
  ),
) <fig:lag_residual>

== Tracking the input gain <sec:gain_rls>

With $tau_m$ fixed, an RLS estimator with the single parameter $b$ runs on @eq:gain_regression for another #qty("20", "s"), with $mu = 0.9999$.
The bottom of @fig:ident_trajectories shows its estimate, which is within 1 % of the true value after a few seconds and ends at $hat(b) = #qty("30.08", "rad/s^2/N")$; over the five runs, between 29.6 and 30.3.

Before the estimate is used, two checks decide whether it can be trusted @miniident.
The _excitation_ check looks at the information matrix of @sec:excitation and reports whether its smallest eigenvalue is large enough; it only works if the regressor is of about unit size, which is the reason why @eq:gain_regression is written in newtons and radians per second squared rather than in increments per sample.
The _identifiability_ check compares the standard deviations from $r bold(P)$ with a tolerance per parameter, here #qty("1.5", "rad/s^2/N") or 5 % of $b$, and also flags parameters whose estimates are strongly correlated.
In this run, the identifiability check accepts the estimate after #qty("15", "s"), at the dotted line, and after 15 to #qty("16", "s") in the other four.

That is conservative.
The estimate itself varies by a standard deviation of only 0.12 between 25 and #qty("40", "s"), while $bold(P)$ claims about ten times that: the RLS covariance assumes white noise in the output, but the noise of a difference of two gyroscope readings is not white, and against a slowly varying regressor, consecutive noise samples largely cancel.
The check errs on the safe side, at the cost of a longer run.
For the biased model of @sec:arx, the identifiability check never accepted the estimate in this run, but only because its variance stayed large; with a looser tolerance, the wrong motor pole would have passed.
Both checks judge how much the data say, not whether the model is right.
A model that is wrong in structure can be very certain about the wrong values, and only a look at what it leaves unexplained (its residuals should be white) or a comparison with an independent measurement reveals it.

== From parameters to a flying controller <sec:ident_to_gains>

With $hat(b)$ and $hat(tau)_m$, the design of @cha:control runs once, in double precision: discretization @eq:axis_zoh, LQR with integral action and Bryson's weights, and the observer gain from the Riccati equation.
For the identified airframe, it yields

$ bold(K) = mat(15.2, 1.41, 0.771, 0.0614), quad k_i = 48.5 $

in the units of @sec:integral.
The control loop itself runs in single precision and consists of the observer of @eq:observer, which predicts with the identified model and corrects with angle and rate, and the state feedback with integral action and anti-windup.
In Rust, with the observer and its buffers set up once at start, one sample reads:

```rust
// observer, measurement, command: the observer of Section 3.8 with the
// identified Phi, the measurement matrix H and gain K_f, and the input Gamma.
// law: the state feedback with integral action and anti-windup (Section 3.7).

// Observer: predict with the previous command, correct with the measurement.
observer.predict().control(&mut command);
measurement.measurement_vector_mut().set_at(0, 0, theta);
measurement.measurement_vector_mut().set_at(1, 0, omega);
observer.correct(&mut measurement);

// State feedback with integral action, saturated, without windup.
let x = observer.state_vector();
let x_hat = [0, 1, 2, 3].map(|i| x.get_row(i));
let u = law.step(reference, &x_hat, theta);
command.control_vector_mut().set_at(0, 0, u);
```

and the design, after identification, is a single call:

```rust
// design_axis: discretization, LQR with integral action and observer gain,
// with Bryson's weights and the noise model of Chapter 3 as defaults.
let axis = AxisParameters::new(b_hat, tau_m_hat, sample_period)?;
let gains = design_axis(&axis, &Weights::default(), &Noise::default())?;
```

@fig:bringup_step flies a step of #qty("0.1", "rad") on the simulated airframe, once with the gains designed for the placeholder values of @cha:control and once with the identified ones.
The difference is small: 29 % against 27 % overshoot, settling after 0.93 against #qty("0.89", "s").
LQR tolerates the 13 % error in $b$ of the placeholder values well, and the overshoot of both comes from the integral action on a reference step (@sec:integral), not from the model.
The identification pays off where the placeholder values are further off than here, which on a new airframe they easily are, and in the observer, whose estimate of the thrust difference (bottom of @fig:bringup_step) follows the true value with an RMS error of #qty("0.04", "N").

#figure(
  placement: auto,
  {
    bringup-step()
    caption-rule
  },
  caption: flex-caption(
    [Step response after identification],
    [Step of #qty("0.1", "rad") on the simulated airframe with measurement noise, with the gains for the placeholder airframe of @cha:control and for the identified one. Bottom: the thrust difference and the observer's estimate of it, with the identified gains.],
  ),
) <fig:bringup_step>

== Bring-up procedure <sec:bringup>

Put together, bringing up one axis on a real vehicle takes the following steps:

+ *Thrust stand.* Measure $C_T$, the hover speed and the motor time constant of each motor and propeller by stepping the command at hover speed (@subsec:sensing). This gives a first value for $tau_m$ and a check for the grid search.
+ *Timing and sensors.* Check the sample period and the delay between reading the sensors and the motor command taking effect, and measure the noise of the gyroscope and of the attitude estimate at rest. These go into $T_s$, the regressor and the noise model of the observer.
+ *Soft stabilizer.* Fly or mount the vehicle with a PD controller of about a tenth of the gains designed for the placeholder values, and check that it holds the axis.
+ *Excitation.* Add a PRBS of 10 to 15 % of the command range to the stabilizer's output for at least #qty("20", "s"), at hover thrust, and record or process rate and command.
+ *Motor time constant.* Run the grid search, check that the minimum is not at the edge of the grid, and compare with the thrust stand.
+ *Input gain.* Run the one-parameter RLS with the found $tau_m$ until the excitation and identifiability checks agree.
+ *Design and check.* Compute the gains, look at the closed-loop poles and simulate a step with the identified model before anything flies.
+ *First flight.* Start with a larger input weight $rho$ (gentler gains), make small steps, and tighten the weights while watching the motor commands for saturation and noise.
+ *Other axes.* Repeat for pitch; identify yaw separately, with its own, much smaller input gain.

The input gain changes in flight, e.g. with the battery voltage, which lowers the motor speed for the same command as the battery drains.
The one-parameter RLS is cheap enough to keep running during flight, with its forgetting factor and the excitation check deciding whether a new estimate is used or the last trusted one is kept; that the excitation check knows when the data say nothing new is exactly what makes this safe.
