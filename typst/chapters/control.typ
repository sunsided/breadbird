// Chapter 3: state-space control of one attitude axis (new). Numbers and
// figures come from matlab/attitude_control.m; matlab/test_design.m checks
// its helpers against the Octave control package.
#import "../commands.typ": *
#import "../figures/control/diagrams.typ": axis-chain, feedback-loop
#import "../figures/control/z_plane.typ": z-plane
#import "../figures/control/poles.typ": pole-gallery, closed-loop-poles
#import "../figures/control/responses.typ": open-loop, placement-wn, placement-zeta, placement-motor, lqr-rho, integral-action, windup, rate-tuning, twiddle-cost, cascade

= State-space control <cha:control>
#context set-lhead[Chapter #counter(heading).get().first(). _State-space control_]

@cha:attitude ended with torques that turn the frame and the observation that how fast they do so depends on the motors and on the moment of inertia.
This chapter turns that into a controller for one axis: a rule that computes, many hundred times per second, the thrust difference that brings the vehicle to the commanded angle and keeps it there.
The design works on a model with a handful of numbers in it; which numbers these are for a real airframe, and how to measure them, is the subject of @cha:identification.
Until then, a reference airframe with placeholder values stands in for it.

All numbers and figures in this chapter come from simulations of the model of @sec:state_space with this reference airframe, sampled at #qty("500", "Hz") unless stated otherwise.

== Notation <sec:notation>

The chapter uses a handful of conventions that are standard in control engineering, but not elsewhere.

_Signals over time._
A quantity that changes continuously, such as the angle, is written with its time as argument, $theta(t)$; a dot marks the derivative with respect to time, so $dot(theta)$ is the rate of change of the angle and $dot.double(theta)$ the rate of change of that.
The flight controller, however, does not see continuous signals.
It reads its sensors and computes a new output every _sample period_ $T_s$, and numbers its samples with an integer $k = 0, 1, 2, dots$
A value at sample $k$ is written with square brackets: $theta[k] = theta(k T_s)$ is the angle at time $k T_s$, and $u[k]$ is the motor command computed at sample $k$, which the motors keep until the next command arrives.
$theta[k + 1]$ is then simply the next sample, and an equation that relates $[k + 1]$ to $[k]$ describes how the system gets from one sample to the next.

_Vectors and matrices._
Bold lowercase letters are vectors, i.e. several numbers stacked into a column, such as $bold(x) = (theta, omega)$ for an angle and a rate.
Bold uppercase letters are matrices, rectangular tables of numbers.
The product $bold(A) bold(x)$ of a matrix and a vector is a vector whose $i$-th entry is the $i$-th row of $bold(A)$ multiplied entry by entry with $bold(x)$ and summed up, i.e. a weighted sum of the entries of $bold(x)$, with one set of weights per row.
This is what makes matrices compact: one $bold(A) bold(x)$ stands for as many weighted sums as $bold(A)$ has rows.
The transpose $bold(x)^top$ turns a column into a row (and a matrix's rows into its columns), so that $bold(x)^top bold(x) = x_1^2 + x_2^2 + dots$ is the sum of the squared entries.
$bold(I)$ is the identity matrix, ones on the diagonal and zeros elsewhere, for which $bold(I) bold(x) = bold(x)$, and $bold(A)^(-1)$ is the inverse of $bold(A)$, the matrix that undoes it: $bold(A)^(-1) bold(A) = bold(I)$.

== Why feedback <sec:why_feedback>

Without feedback, the commanded thrust difference is the only thing that moves the vehicle, and the vehicle has no preferred attitude.
@fig:open_loop shows what happens when the rotors on one side produce #qty("0.05", "N") (about #qty("5", "g")) more thrust than on the other for #qty("50", "ms"): the vehicle starts to roll and keeps rolling at a constant rate long after the pulse is over, since nothing brings the rate back to zero.
Any disturbance, be it a gust, a slightly heavier side or a motor that runs a little faster than its neighbor, does the same.
A flight controller therefore has to measure the attitude and correct it continuously; this is _feedback_#index("feedback").

#figure(
  placement: auto,
  {
    open-loop()
    caption-rule
  },
  caption: flex-caption(
    [Open-loop response to a thrust pulse],
    [Open-loop response of the reference airframe to a thrust difference of #qty("0.05", "N") for #qty("50", "ms"): the roll rate $omega$ (dashed) stays where the pulse left it, and the angle $theta$ grows without bound.],
  ),
) <fig:open_loop>

== The plant in state-space form <sec:state_space>

The part of the system that is controlled is called the _plant_#index("plant").
For one axis, it consists of three effects in a row (@fig:axis_chain):

+ The motors do not change their thrust instantly. To a first approximation, the thrust difference $Delta T$ follows the commanded value $u$ like a low-pass filter with the _motor time constant_#index("motor time constant") $tau_m$, which @subsec:area_vs_velocity related to the rotational inertia of the rotors.
+ The thrust difference acts on the lever arm $l$ and accelerates the frame by @eq:torque_ab and @eq:angular_acceleration. Writing the moment of inertia as $J$ (to keep $I$ free for the integral term later on), the angular acceleration is $dot(omega) = b Delta T$ with $b = l \/ J$.
+ The angular rate $omega$ integrates to the angle $theta$.

#figure(
  placement: auto,
  {
    axis-chain()
    caption-rule
  },
  caption: flex-caption(
    [One attitude axis as a chain of blocks],
    [One attitude axis from the commanded thrust difference $u$ to the angle $theta$: one sample of delay, the motor lag, the gain $b = l \/ J$ and two integrators.],
  ),
) <fig:axis_chain>

The classical way of writing this down is a _transfer function_#index("transfer function").
It rests on the Laplace transform, which replaces a signal over time by a function of a complex frequency $s$; its use here is that differentiating a signal becomes a multiplication by $s$ and integrating it a division by $s$.
A chain of blocks then becomes a product of simple fractions, and the ratio of output to input is the transfer function.
For the chain of @fig:axis_chain without the delay, the motor lag contributes $1 \/ (tau_m s + 1)$, the gain $b$ and each integrator $1 \/ s$:

$ G(s) = theta(s) / u(s) = b / (s^2 (tau_m s + 1)) $ <eq:axis_tf>

This is compact, but hides everything that happens inside the chain.
The _state-space form_#index("state space") keeps it visible.
It collects the quantities that describe the plant at one instant, its _state_#index("state"), into a vector $bold(x)$ and writes the dynamics as one first-order differential equation for all of them:

$ dot(bold(x)) = bold(A) bold(x) + bold(B) u, quad
  bold(x) = vec(theta, omega, Delta T), quad
  bold(A) = mat(0, 1, 0; 0, 0, b; 0, 0, -1 \/ tau_m), quad
  bold(B) = vec(0, 0, 1 \/ tau_m) $ <eq:axis_ss>

Each row is one of the effects above, read as "the rate of change of this state is the following weighted sum of the states and the input": the angle changes with the rate, the rate with $b$ times the thrust difference, and the thrust difference with the distance between command and current value, divided by $tau_m$.
The state is the minimal memory of the plant: knowing $bold(x)$ now and $u$ from now on is enough to predict everything that follows, which is exactly what a controller needs.

=== Discretization <subsec:discretization>

The flight controller does not run continuously, but samples its sensors and updates its output at a fixed _sample period_#index("sample period") $T_s$, holding the output constant in between, so that the motor command is a staircase (a _zero-order hold_#index("zero-order hold")).
Since the input is constant during each step, the differential equation @eq:axis_ss can be solved exactly from one sample to the next, and the plant turns into a _difference equation_ that computes the next state from the current one,

$ bold(x)[k + 1] = bold(Phi) bold(x)[k] + bold(Gamma) u[k], quad
  bold(Phi) = e^(bold(A) T_s), quad
  bold(Gamma) = integral_0^(T_s) e^(bold(A) sigma) dif sigma thin bold(B) $ <eq:zoh>

with the _matrix exponential_ $e^(bold(A) T_s) = bold(I) + bold(A) T_s + (bold(A) T_s)^2 \/ 2! + dots$, defined by the same power series as the ordinary exponential function.
$bold(Phi)$ says where the state goes on its own within one sample, $bold(Gamma)$ how much one sample of constant input adds to it.
For the chain of @eq:axis_ss, both can be written out: with the motor pole $a = e^(-T_s \/ tau_m)$ and $g = 1 - a$,

$ bold(Phi) = mat(1, T_s, b tau_m (T_s - tau_m g); 0, 1, b tau_m g; 0, 0, a), quad
  bold(Gamma) = vec(b (T_s^2 \/ 2 - tau_m (T_s - tau_m g)), b (T_s - tau_m g), g) $ <eq:axis_zoh>

which avoids evaluating a matrix exponential on the flight controller.

A real controller also needs some time to compute its output: the sensors are read at the start of a sample, the new motor command takes effect at the start of the next one.
This one-sample delay becomes a fourth state, the previous input $u_"prev"$:

$ bold(x)[k] = vec(theta, omega, Delta T, u_"prev"), quad
  bold(x)[k + 1] = mat(bold(Phi), bold(Gamma); bold(0), 0) bold(x)[k] + vec(bold(0), 1) u[k] $ <eq:axis_delay>

From here on, $bold(Phi)$ and $bold(Gamma)$ denote these four-state matrices.

=== The reference airframe <subsec:reference_airframe>

@tab:reference_airframe lists the values used throughout this chapter.
They describe a small quadrotor of about #qty("0.5", "kg") and are placeholders, plausible but not measured; @cha:identification replaces them with the values of the real vehicle.

#figure(
  placement: auto,
  table(
    columns: 4,
    align: (left, center, right, left),
    stroke: none,
    table.hline(),
    table.header([Quantity], [Symbol], [Value], [Unit]),
    table.hline(stroke: 0.5pt),
    [arm length], $l$, [0.17], unit("m"),
    [moment of inertia (roll)], $J$, [0.005], unit("kg m^2"),
    [input gain], $b = l \/ J$, [34], unit("rad/s^2/N"),
    [motor time constant], $tau_m$, [0.03], unit("s"),
    [sample period], $T_s$, [0.002], unit("s"),
    [largest thrust difference], $u_max$, [1.0], unit("N"),
    [noise of the angle estimate], $sigma_theta$, [0.002], unit("rad"),
    [noise of the gyroscope], $sigma_omega$, [0.01], unit("rad/s"),
    table.hline(),
  ),
  caption: flex-caption(
    [Reference airframe],
    [Reference airframe used for the designs in this chapter. Placeholder values, to be replaced by identified ones.],
  ),
) <tab:reference_airframe>

== Poles and what they mean <sec:poles>

Without input, @eq:axis_ss is $dot(bold(x)) = bold(A) bold(x)$.
For most starting states, $bold(A) bold(x)$ points somewhere else than $bold(x)$, and the state changes its direction as well as its size.
For a few special directions $bold(v)$, the _eigenvectors_, it does not: $bold(A) bold(v) = s bold(v)$ with a number $s$, the _eigenvalue_.
A state that starts along such a direction stays on it and only changes its size, by the factor $e^(s t)$, since that is the function whose derivative is $s$ times itself.
Every other motion is a mix of these few _modes_, and the eigenvalues of $bold(A)$, the _poles_#index("pole") of the system, therefore describe everything it can do on its own.

The eigenvalues can be complex numbers $s = sigma + j omega$, and $e^(s t) = e^(sigma t) (cos omega t + j sin omega t)$ then shows what each part does: the real part $sigma$ makes the mode decay ($sigma < 0$) or grow ($sigma > 0$), the imaginary part $omega$ makes it oscillate with the angular frequency $omega$.
Complex poles always come in pairs $sigma plus.minus j omega$ whose imaginary parts cancel, so the motion itself stays real.
@fig:pole_gallery shows what the position of a pole means for its mode: the further left, the faster the decay; the further up, the faster the oscillation; and anything right of the imaginary axis grows without bound.
Read the other way round, the figure is a menu for pole placement: picking a position picks the kind of motion the closed loop will have.

#figure(
  placement: auto,
  {
    pole-gallery()
    caption-rule
  },
  caption: flex-caption(
    [Pole positions and their modes],
    [Poles in the upper half of the $s$-plane and the mode $e^(sigma t) cos omega t$ each of them contributes. The gray line in each sketch is zero. Left of the imaginary axis (shaded), modes decay; on it, they persist, the pole at the origin as a constant; right of it, they grow. Poles on the real axis do not oscillate.],
  ),
) <fig:pole_gallery>

The open-loop plant has the poles $0$, $0$ (the two integrators) and $-1 \/ tau_m$ (the motor): the double pole at zero is the reason for @fig:open_loop.

The whole point of a controller is to move these poles: feedback changes the system, and with it its eigenvalues, so that a controller can be specified by where it puts them.
Choosing the gains that move the poles to prescribed positions is called _pole placement_ (@sec:pole_placement); the rest of this section is about which positions to prescribe.
A pair of complex poles

$ s_(1,2) = -zeta omega_n plus.minus j omega_n sqrt(1 - zeta^2) $ <eq:pole_pair>

behaves like a mass on a spring with the _natural frequency_#index("natural frequency") $omega_n$ and the _damping ratio_#index("damping ratio") $zeta$, and their effect on a step response is well known @astrom2021feedback:
the step overshoots its target by

$ M_p = e^(-pi zeta \/ sqrt(1 - zeta^2)) $ <eq:overshoot>

(about 37 % for $zeta = 0.3$, 4.6 % for $zeta = 0.7$ and none from $zeta = 1$ on), and it settles to within 2 % of the target after about $t_s approx 4 \/ (zeta omega_n)$.
The natural frequency thus sets the speed and the damping the shape, which is what makes poles a useful language for specifying a controller.

For the discrete system, the same holds for the eigenvalues $z$ of $bold(Phi)$, which relate to the continuous poles by

$ z = e^(s T_s) $ <eq:s_to_z>

The left half of the $s$-plane maps into the unit circle: a discrete pole inside the unit circle decays, one outside grows, and $z = 0$ is the fastest possible pole, which is gone after one sample.
The delay state of @eq:axis_delay has its pole there.

@fig:z_plane shows a set of pole pairs ($omega_n$ from 8 to #qty("25", "rad/s"), $zeta$ from 0.3 to 1) at sample rates of #qty("50", "Hz") and #qty("500", "Hz").
At #qty("500", "Hz"), all of them crowd into a small region just left of $z = 1$: the pair with $omega_n = #qty("15", "rad/s")$ and $zeta = 0.7$ has $|z| = e^(-zeta omega_n T_s) approx 0.979$.
Fast sampling is good for control, since the delay it adds is small, but it makes $bold(Phi)$ very close to the identity matrix, and the numbers that describe the dynamics end up in the fourth or fifth significant digit.
This comes back to bite in @sec:pole_placement.

#figure(
  placement: auto,
  {
    z-plane()
    caption-rule
  },
  caption: flex-caption(
    [Discrete poles at 50 Hz and 500 Hz],
    [Pole pairs with $omega_n in {8, 15, 25}$ rad/s and $zeta in {0.3, 0.7, 1}$, mapped into the $z$-plane by @eq:s_to_z for sample rates of #qty("50", "Hz") (circles) and #qty("500", "Hz") (crosses), with the unit circle in gray. Right: the region near $z = 1$.],
  ),
) <fig:z_plane>

== Pole placement <sec:pole_placement>

A _state feedback_#index("state feedback") controller computes the input as a weighted sum of the states,

$ u[k] = -bold(K) bold(x)[k] $ <eq:state_feedback>

with a row vector $bold(K)$ of gains, one per state.
Inserted into @eq:axis_delay, the closed loop is

$ bold(x)[k + 1] = (bold(Phi) - bold(Gamma) bold(K)) bold(x)[k] $ <eq:closed_loop>

whose poles are the eigenvalues of $bold(Phi) - bold(Gamma) bold(K)$.
With four states and four gains, there is one gain per pole to choose, and for a single input, the gains that put the poles at given positions are unique.
_Pole placement_#index("pole placement") computes them.

=== Controllability <subsec:controllability>

The gains can only move a pole if the input reaches the corresponding part of the state.
Starting from $bold(x) = bold(0)$, the input reaches the directions $bold(Gamma)$ after one step, $bold(Phi) bold(Gamma)$ after two and so on; if the _controllability matrix_#index("controllability")

$ bold(C) = mat(bold(Gamma), bold(Phi) bold(Gamma), bold(Phi)^2 bold(Gamma), bold(Phi)^3 bold(Gamma)) $ <eq:controllability>

has full rank, every state can be reached and every pole can be placed.
The _rank_ of a matrix is the number of independent directions among its columns; full rank means that the columns point into as many different directions as there are states, so that combinations of them reach every state.
For the attitude axis, this holds: the input reaches the delay state directly and everything else through the chain of @fig:axis_chain.

A system that is _not_ controllable is more familiar than the term suggests.
Take a shower with a single lever that opens the hot and the cold valve together, by the same amount.
With the hot flow $h$ and the cold flow $c$ as states, both valves following the lever $u$ with the same lag, one sample of the discrete model reads

$ vec(h, c)[k + 1] = mat(a, 0; 0, a) vec(h, c)[k] + vec(g, g) u[k] $ <eq:shower>

where, as for the motor lag of @eq:axis_zoh, $a = e^(-T_s \/ tau_v)$ is the fraction of its flow a valve with the time constant $tau_v$ keeps from one sample to the next, and $g = 1 - a$ the fraction of the way to the lever setting it covers in one sample.
The controllability matrix is

$ bold(C) = mat(bold(Gamma), bold(Phi) bold(Gamma)) = mat(g, a g; g, a g) $ <eq:shower_controllability>

whose two columns point in the same direction, $(1, 1)$: its rank is one, not two.
The lever moves both flows up and down together, i.e. it sets the total flow, but nothing it does changes the difference $h - c$, which only decays on its own.
And the difference is what sets the temperature, which therefore cannot be controlled at all, however clever the controller.
On the quadrotor, the same happens if all four rotors are always given the same command: the vehicle can climb and sink, but never roll, since the collective thrust does not reach the attitude states.
A tricopter with three fixed rotors is the airframe version of the shower: it has one input too few for its four quantities and cannot hover without spinning (@sec:why_four_rotors).

Had the two valves different time constants, and with them different $a_h != a_c$ and $g_h != g_c$, the columns of @eq:shower_controllability would be $(g_h, g_c)$ and $(a_h g_h, a_c g_c)$, which point in different directions, and the shower would formally be controllable: by wiggling the lever quickly, the faster valve could be made to run ahead of the slower one.
Formally controllable but practically hopeless is exactly the situation the condition number of $bold(C)$ measures.

Full rank is a yes-or-no question, but the numbers behind it are not.
Since $bold(Phi) approx bold(I)$ at high sample rates, the columns of $bold(C)$ are almost the same vector, and its condition number grows accordingly: about $2.7 dot 10^5$ for the reference airframe at #qty("500", "Hz"), compared to about 285 at #qty("50", "Hz").
Anything that inverts $bold(C)$ loses about five significant digits to this, which is fine in double precision (about 16 digits) and hopeless in single precision (about 7).

=== Ackermann's formula

For a single input, the gains follow in closed form from _Ackermann's formula_#index("Ackermann's formula") @ackermann1972

$ bold(K) = mat(0, 0, 0, 1) bold(C)^(-1) phi(bold(Phi)) $ <eq:ackermann>

where $phi(z) = (z - z_1)(z - z_2)(z - z_3)(z - z_4)$ is the _characteristic polynomial_#index("characteristic polynomial") with the desired poles $z_1$ to $z_4$, the polynomial whose roots are the eigenvalues, multiplied out and evaluated with the matrix $bold(Phi)$ in place of the number $z$.
The formula is not very intuitive, but it does the obvious thing: it finds the one gain vector for which the characteristic polynomial of $bold(Phi) - bold(Gamma) bold(K)$ equals $phi(z)$.
The formula is easy to implement and good enough for four states in double precision; for larger systems, the numerical issue above makes iterative methods the better choice.

=== Choosing the poles

The harder question is where to put the poles.
The four poles of the axis play different roles, and the figures of this section show each choice the same way: a map of where the poles are placed on the left, and what the closed loop then does on a step of #qty("0.1", "rad") on the right, in matching colors.
The delay pole is the easy one: it goes to $z = 0$, where it is gone after one sample.

_The motor pole._
The motor pole should stay roughly where it is, since moving it far would mean driving the motors much harder than their own dynamics allow.
A motor lag only speeds up if the command overshoots: to make the thrust settle in a quarter of $tau_m$, the controller has to command about four times the thrust change it wants for the first moments, and with $u_max = #qty("1", "N")$, a change of #qty("0.25", "N") already takes the full range.
@fig:placement_motor shows what happens when the motor pole is moved from $-#qty("40", "1/s")$ to $-130$ and $-#qty("400", "1/s")$, with the dominant pair unchanged and with the saturation and sensor noise of the real vehicle.
The angle hardly notices: its response is set by the dominant pair, which did not move.
The command does: the gains grow from $bold(K) = mat(7.7, 0.94, 0.82, 0.05)$ to $mat(55, 5.5, 7.8, 0.53)$, the step would ask for a thrust difference of #qty("5.5", "N") instead of #qty("0.77", "N") if the motors could deliver it, and the sensor noise in the command grows tenfold, from #qty("0.04", "N") to #qty("0.44", "N") RMS.
On the vehicle, this is a motor that runs hot, draws current spikes from the battery and audibly buzzes, for no gain in the response.
It also makes the loop fragile: one sample more delay than designed for leaves the design at $-#qty("400", "1/s")$ with a mode at #qty("250", "Hz") whose damping is only 0.05, against 0.4 for the design at $-#qty("40", "1/s")$.
Moving a pole that does not dominate the response buys nothing and costs a lot, and the motor pole is therefore placed at $-#qty("40", "1/s")$, only slightly faster than $1 \/ tau_m approx #qty("33", "1/s")$.

#figure(
  placement: auto,
  {
    placement-motor()
    caption-rule
  },
  caption: flex-caption(
    [Moving the motor pole],
    [Moving the motor pole: the same dominant pair (gray) with the motor pole at $-40$, $-130$ and $-400$ #unit("1/s"), with saturation and sensor noise. The angle (top) is the same in all three cases; the command (bottom) gets noisier the further the pole is moved.],
  ),
) <fig:placement_motor>

_The natural frequency._
The remaining pair describes how the angle responds, and its distance $omega_n$ from the origin sets the speed (@fig:placement_wn).
With $zeta = 0.7$, the response settles after 0.77, 0.43, 0.27 and #qty("0.15", "s") for $omega_n = 8$, 15, 25 and #qty("40", "rad/s"), with about 4 % overshoot, close to the 4.6 % of @eq:overshoot (1.3 % at #qty("40", "rad/s"), where the saturated command holds the response back).
The price of speed is the command: its peak grows from #qty("0.22", "N") to #qty("0.77", "N"), and from $omega_n = #qty("25", "rad/s")$ on it saturates at $u_max$ in the first samples.

So far, the plant is exactly the model the gains were designed for, which a real vehicle never is.
The dotted lines in @fig:placement_wn fly the same gains on an airframe whose motors are twice as slow as modeled and which has #qty("8", "ms") more delay, e.g. from the motor controllers.
The slow designs barely notice (6 % overshoot instead of 4.5 % at $omega_n = 8$); the fast ones do, with 14 % at $omega_n = 25$ and 52 % at $omega_n = #qty("40", "rad/s")$, which now swings back and forth for #qty("0.7", "s") before it settles.
The faster the poles are placed, the more the design relies on the model being right, and the less margin it keeps for what the model got wrong; this is the strongest argument for both a modest $omega_n$ and for identifying the model (@cha:identification).

#figure(
  placement: auto,
  {
    placement-wn()
    caption-rule
  },
  caption: flex-caption(
    [Pole placement for different natural frequencies],
    [Pole placement for $omega_n in {8, 15, 25, 40}$ rad/s at $zeta = 0.7$, with the motor pole at $-#qty("40", "1/s")$ (gray). Top: angle; dotted, the same gains on a hypothetical airframe with motors twice as slow as modeled and #qty("8", "ms") more delay. Bottom: command.],
  ),
) <fig:placement_wn>

_The damping._
@fig:placement_zeta shows the familiar picture: 34.5 % overshoot for $zeta = 0.3$, none for $zeta = 1$, where the pair has become a double real pole.

#figure(
  placement: auto,
  {
    placement-zeta()
    caption-rule
  },
  caption: flex-caption(
    [Pole placement for different damping ratios],
    [Pole placement for $zeta in {0.3, 0.7, 1}$ at $omega_n = #qty("15", "rad/s")$, with the motor pole at $-#qty("40", "1/s")$ (gray); for $zeta = 1$, the pair is a double pole on the real axis.],
  ),
) <fig:placement_zeta>

This looks like a trade between speed and overshoot, but for a pair of poles it is not quite that: the damping alone sets the overshoot (@eq:overshoot), and the natural frequency alone sets the speed, at any damping.
The trade within the damping is between rising quickly and arriving cleanly.
A low $zeta$ rises fast and swings past the target; $zeta = 1$ never overshoots, but creeps into the target and needs $5.8 \/ omega_n$ to settle within 2 %.
In between lies the sweet spot: $zeta approx 0.8$ settles after only $3.8 \/ omega_n$ with 1.5 % overshoot, and the classic $zeta = 0.7$ rises a little faster at 4.6 % overshoot.

The choice is therefore made in two steps.
First, the damping follows from how much overshoot the task can accept: $zeta$ between 0.7 and 0.8 as the default, closer to 1 if overshooting the target is a problem in itself (an angle reference usually is not; a landing gear touching the ground would be).
Second, the natural frequency is raised as far as the plant allows: the command must not saturate for the steps that matter, the noise in the command must stay acceptable, and the loop must keep enough margin for model errors, which in practice means staying well below the motor's $1 \/ tau_m$.
In practice, this means simulations like @fig:placement_wn with the real step sizes, noise levels and some pessimism about the model, and the largest $omega_n$ that still looks clean.
@sec:lqr reaches the same compromise from the other side, by pricing error and effort instead of prescribing poles.

For $omega_n = #qty("15", "rad/s")$ and $zeta = 0.7$, the gains are

$ bold(K)_"pp" = mat(7.73, 0.937, 0.820, 0.0544) $

in #unit("N/rad"), #unit("N s/rad") and two dimensionless entries for the thrust states.
The first two entries are a proportional and a derivative gain on the angle; the others compensate for the motor lag and the delay, which a plain PD controller ignores (@subsec:pid_state_feedback).

== The linear-quadratic regulator <sec:lqr>

Pole placement answers the question "where should the poles be" with a guess.
The _linear-quadratic regulator_#index("LQR") (LQR) asks a different one, "how much does an error cost compared to an effort", and lets the poles follow.
It chooses the gains that minimize

$ cal(J) = sum_(k = 0)^infinity (bold(x)[k]^top bold(Q) bold(x)[k] + R u[k]^2) $ <eq:lqr_cost>

with a weight matrix $bold(Q)$ on the state error and a weight $R$ on the input.
For a diagonal $bold(Q)$, the term $bold(x)^top bold(Q) bold(x) = Q_(11) theta^2 + Q_(22) omega^2 + dots$ is a weighted sum of the squared states, so @eq:lqr_cost adds up, over all future samples, how far the vehicle is from where it should be and how hard the motors work to get it there.
Squares make large errors expensive and small ones cheap, and they make the minimum easy to compute.
A large $bold(Q)$ asks for fast correction, a large $R$ for gentle use of the motors.

=== The Riccati equation

The minimum of @eq:lqr_cost can be found backwards in time.
Let $bold(P)_N$ describe the _cost-to-go_, the smallest cost that can still be achieved from the state $bold(x)$ with $N$ steps left, as $bold(x)^top bold(P)_N bold(x)$; with no step left, only the state error counts, so $bold(P)_0 = bold(Q)$.
One step further back, the controller has to choose the $u$ that minimizes the cost of this step plus the cost of all following steps from wherever $u$ leads, and since everything is quadratic, the minimum follows in closed form:

$ bold(K)_N &= (R + bold(Gamma)^top bold(P)_N bold(Gamma))^(-1) bold(Gamma)^top bold(P)_N bold(Phi) \
  bold(P)_(N + 1) &= bold(Q) + bold(Phi)^top bold(P)_N bold(Phi) - bold(Phi)^top bold(P)_N bold(Gamma) bold(K)_N $ <eq:riccati_iteration>

As $N$ grows, $bold(P)_N$ converges to the solution $bold(P)$ of the _discrete algebraic Riccati equation_#index("Riccati equation") (DARE)

$ bold(P) = bold(Q) + bold(Phi)^top bold(P) bold(Phi) - bold(Phi)^top bold(P) bold(Gamma) (R + bold(Gamma)^top bold(P) bold(Gamma))^(-1) bold(Gamma)^top bold(P) bold(Phi) $ <eq:dare>

and $bold(K)_N$ to the gain $bold(K)$ of the infinite horizon, which is the LQR.

Iterating @eq:riccati_iteration works, but slowly: each iteration extends the horizon by one sample, and at #qty("500", "Hz"), the slowest closed-loop pole decays by about 3 % per sample, so the iteration needs thousands of steps.
The _doubling algorithm_#index("doubling algorithm") @chu2004sda doubles the horizon in each step instead.
With $bold(A)_0 = bold(Phi)$, $bold(G)_0 = bold(Gamma) R^(-1) bold(Gamma)^top$ and $bold(H)_0 = bold(Q)$, it iterates

$ bold(W)_i &= bold(I) + bold(G)_i bold(H)_i \
  bold(A)_(i + 1) &= bold(A)_i bold(W)_i^(-1) bold(A)_i \
  bold(G)_(i + 1) &= bold(G)_i + bold(A)_i bold(W)_i^(-1) bold(G)_i bold(A)_i^top \
  bold(H)_(i + 1) &= bold(H)_i + bold(A)_i^top bold(H)_i bold(W)_i^(-1) bold(A)_i $ <eq:sda>

and $bold(H)_i$ converges quadratically to $bold(P)$: for the reference airframe, 11 iterations bring the residual of @eq:dare down to about $10^(-12)$.

=== Choosing the weights

The weights still have to be chosen, but they are easier to reason about than poles.
_Bryson's rule_#index("Bryson's rule") divides each quantity by the largest value that is still acceptable, so that all terms of @eq:lqr_cost become comparable:

$ Q_(i i) = 1 / x_(i, max)^2, quad R = 1 / u_max^2 $ <eq:bryson>

For the reference airframe, an angle error of #qty("0.05", "rad") (about 3°), a rate of #qty("1", "rad/s") and a thrust difference of #qty("0.5", "N") are acceptable, and the thrust states are not weighted at all:

$ bold(Q) = "diag"(400, 1, 0, 0), quad R = 4 $

which yields

$ bold(K)_"lqr" = mat(9.73, 1.15, 0.841, 0.0553) $

with closed-loop poles at $|z| = 0.941$ and a pair at $|z| = 0.970$, plus the delay pole at zero.
@fig:lqr shows the effect of scaling $R$ by a factor $rho$: with $rho = 0.1$, the controller saturates and settles after #qty("0.23", "s"); with $rho = 10$, it stays below #qty("0.31", "N") and takes #qty("0.59", "s").
Neither run overshoots by more than 3 %, without a single pole having been placed by hand.

#figure(
  placement: auto,
  {
    lqr-rho()
    caption-rule
  },
  caption: flex-caption(
    [LQR for different input weights],
    [Step of #qty("0.1", "rad") with LQR, for the input weight $R$ of @eq:bryson scaled by $rho in {0.1, 1, 10}$. Top: angle; bottom: command.],
  ),
) <fig:lqr>

This is why LQR is the practical default: the weights describe what matters in the units of the problem, the result is stable by construction (for any $bold(Q)$ and $R > 0$ with a controllable plant), and a design that is too aggressive is fixed by turning one number up.

@fig:closed_loop_poles puts the designs of this and the previous section side by side, with the discrete poles converted back into the $s$-plane by $s = ln(z) \/ T_s$, where they are easier to read.
The pole placement designs sit exactly where they were put: on the two rays of constant damping $zeta = 0.7$, at the distance $omega_n$ from the origin, with the motor pole at $-#qty("40", "1/s")$.
The gray dots trace the LQR poles as the input weight is scaled from $rho = 10^(-3)$ to $10^3$: expensive input lets the dominant pair creep towards the origin and leaves the real pole close to the motor's own $-1 \/ tau_m approx -#qty("33", "1/s")$, while cheap input pulls the pair far to the left (out of the plot, to about $-95 plus.minus j 94$ #unit("1/s") at $rho = 10^(-3)$) and leaves a real pole behind at about $-#qty("20", "1/s")$, which then limits the speed.
With $rho = 1$, LQR ends up with a damping of about 0.8 and a natural frequency of about #qty("19", "rad/s"), between the second and third pole placement design, without having been told about either.

#figure(
  placement: auto,
  {
    closed-loop-poles()
    caption-rule
  },
  caption: flex-caption(
    [Closed-loop poles of the designs],
    [Closed-loop poles in the $s$-plane: pole placement for three natural frequencies (circles), LQR with Bryson's weights (squares) and the path of the LQR poles as the input weight is scaled by $rho$ (gray), with the rays of constant damping $zeta = 0.7$ (dashed). The cross marks the slow observer pole of @sec:observer; its other two poles lie far outside the plot. The delay poles at $z = 0$ have no counterpart in the $s$-plane.],
  ),
) <fig:closed_loop_poles>

== Integral action <sec:integral>

A state feedback controller holds the state at zero, or at a reference $bold(x)_"ref"$ if written as $u = -bold(K) (bold(x) - bold(x)_"ref")$.
If something pushes constantly, such as a center of gravity slightly off the axis or a motor that produces a little more thrust than its neighbors, the controller can only push back by having an error: some angle remains so that $bold(K)$ produces the counter-thrust.

The remedy is to add the integral of the error as a fifth state,

$ x_i [k + 1] = x_i [k] + T_s (r[k] - theta[k]) $ <eq:integral_state>

and to design the controller for the augmented system.
Since the integral grows as long as an error remains, the only steady state is one without error.
With a weight of $1 \/ 0.01^2$ on $x_i$, LQR yields

$ bold(K)_"lqr,i" = mat(15.2, 1.40, 0.976, 0.0641, -48.4) $

where the last entry belongs to the integral; the controller is $u = -bold(K)_(1..4) (bold(x) - bold(x)_"ref") + k_i x_i$ with $k_i = 48.4$.

@fig:integral shows both controllers following a step of #qty("0.1", "rad") and then a thrust offset of #qty("0.05", "N") appearing after #qty("1.5", "s").
Without integral action, the angle settles about #qty("0.006", "rad") (6 %) off; with it, the offset is corrected within about #qty("0.4", "s").
The integral comes at a price on reference steps, though: it starts accumulating the moment the reference jumps, and the response overshoots by about 27 %.
Ramping the reference instead of stepping it, or limiting how fast the integral may grow, avoids this.

#figure(
  placement: auto,
  {
    integral-action()
    caption-rule
  },
  caption: flex-caption(
    [Integral action and observer],
    [Step of #qty("0.1", "rad") and a constant thrust offset of #qty("0.05", "N") from #qty("1.5", "s") on, with measurement noise. Top: angle with LQR and with LQR plus integral action. Bottom: the thrust difference $Delta T$ and its estimate by the observer (@sec:observer).],
  ),
) <fig:integral>

== Observing what is not measured <sec:observer>

The controllers above need the full state, but only the angle (from the attitude estimate) and the rate (from the gyroscope) are measured.
The thrust difference could be measured with the sensors of @subsec:sensing, but on the reference airframe it is not, and the delayed input is known anyway.
An _observer_#index("observer") estimates the missing states by running a copy of the model alongside the plant and correcting it with the measurements:

$ macron(bold(x))[k] &= bold(Phi) hat(bold(x))[k - 1] + bold(Gamma) u[k - 1] \
  hat(bold(x))[k] &= macron(bold(x))[k] + bold(K)_f (bold(y)[k] - bold(H) macron(bold(x))[k]) $ <eq:observer>

where $bold(y) = (theta, omega)$ is the measurement, $bold(H)$ selects the measured states and $bold(K)_f$ is the observer gain.
The first line predicts, the second corrects the prediction by a fraction of what it got wrong.
This is the _current_ form, since the estimate uses the measurement of the same sample; the _predictor_ form instead feeds the correction into the next prediction, with the gain $bold(L) = bold(Phi) bold(K)_f$.

How well this works follows from the _estimation error_ $bold(e) = bold(x) - hat(bold(x))$.
Subtracting the predictor form $hat(bold(x))[k + 1] = bold(Phi) hat(bold(x))[k] + bold(Gamma) u[k] + bold(L) (bold(y)[k] - bold(H) hat(bold(x))[k])$ from the plant @eq:axis_delay, with $bold(y) = bold(H) bold(x)$, the input cancels and

$ bold(e)[k + 1] = (bold(Phi) - bold(L) bold(H)) bold(e)[k] $ <eq:observer_error>

remains: the error is a system of its own, without input, and decays if the eigenvalues of $bold(Phi) - bold(L) bold(H)$, the observer's poles, are inside the unit circle.
Choosing $bold(L)$ is thus a pole placement problem as well (@sec:pole_placement), with $bold(H)^top$ in the role of $bold(Gamma)$ and $bold(Phi)^top$ in that of $bold(Phi)$, since a matrix and its transpose have the same eigenvalues.

Such a model with a correction term is a _Luenberger observer_#index("Luenberger observer"), and @eq:observer is also exactly the structure of the Kalman filter.
The Kalman filter merely chooses $bold(K)_f$ from noise statistics.
These are given as _covariance_ matrices, which generalize the variance (the squared standard deviation) to several quantities at once: the diagonal holds the variance of each quantity, the other entries how strongly two of them vary together.
With the covariance $bold(Q)_n$ of the _process noise_, the random disturbances that act on the plant itself, and $bold(R)_n$ of the measurement noise, its steady-state gain follows from the same Riccati equation @eq:dare with $bold(Phi)^top$, $bold(H)^top$, $bold(Q)_n$ and $bold(R)_n$ in place of $bold(Phi)$, $bold(Gamma)$, $bold(Q)$ and $R$, as

$ bold(K)_f = bold(P) bold(H)^top (bold(H) bold(P) bold(H)^top + bold(R)_n)^(-1) $ <eq:kalman_gain>

where $bold(P)$ is now the covariance of the prediction.
This _duality_#index("duality") is why one Riccati solver serves both the controller and the observer.

For the reference airframe, $bold(R)_n$ holds the noise levels of @tab:reference_airframe, and $bold(Q)_n = "diag"(0, 10^(-4), 10^(-2), 0)$ assumes most of the uncertainty in the motors (thrust ripple, turbulence) and none in the delayed input, which is known exactly.
The resulting observer poles are at $|z| = 0.989$ ($s approx -#qty("5.6", "1/s")$, the cross in @fig:closed_loop_poles), a pair at $|z| = 0.468$ ($s approx -380 plus.minus j 141$ #unit("1/s")) and $0$; the slow one belongs to the thrust difference, which is only seen through its effect on the rate.
Observers are usually made several times faster than the controller, so that the estimate has settled before the controller acts on it; here, the thrust difference is the exception, which the integral action of @sec:integral has to make up for.
The bottom of @fig:integral shows its estimate, which follows the true value with an RMS error of about #qty("0.05", "N").

#figure(
  placement: auto,
  {
    feedback-loop()
    caption-rule
  },
  caption: flex-caption(
    [State feedback with observer and integral action],
    [State feedback with observer and integral action: the observer estimates the state $hat(bold(x))$ from the measurement $bold(y)$ and the input $u$, the controller combines $-bold(K) hat(bold(x))$ with the integral of the angle error.],
  ),
) <fig:feedback_loop>

Controller and observer can be designed independently: by the _separation principle_#index("separation principle"), the poles of the combined loop of @fig:feedback_loop are those of the controller and those of the observer, as long as the model is right.
Mixing up the two forms is an easy mistake, since both gains are common in the literature and in software; used in the wrong recursion, a gain moves the observer poles and can make it diverge.
A Kalman filter running on board computes the time-varying version of $bold(K)_f$ in every step, at the cost of propagating $bold(P)$.

== PID control <sec:pid>

Most flight controllers do not use state feedback, but _PID control_#index("PID controller"), which acts on the error $e = r - y$ between the reference $r$ (where the measured quantity should be) and the measurement $y$ with three terms: a proportional one for the error now, an integral one for the error accumulated so far, and a derivative one for the direction in which the error is heading:

$ u(t) = K_p e(t) + K_i integral_0^t e(tau) dif tau + K_d dot(e)(t) $ <eq:pid_parallel>

This is the _parallel_ form; the textbook _ideal_ form writes the same with $K_i = K_p \/ tau_I$ and $K_d = K_p tau_D$, using the integral time $tau_I$ and the derivative time $tau_D$ @astrom2006pid.
PID needs no model, which explains much of its popularity (@subsec:why_pid), and @subsec:pid_state_feedback shows that it is less different from state feedback than it looks.

=== From the textbook to the difference equation

@eq:pid_parallel is not what runs on a microcontroller.
Three changes turn it into something that does @astrom2006pid:

- _Derivative on the measurement._ With a constant reference, $dot(e) = -dot(y)$, but at a reference step, $dot(e)$ is infinite and the derivative term kicks the motors. Differentiating $-y$ instead avoids this.
- _A filtered derivative._ Differentiating a noisy measurement amplifies the noise, the more so the higher the sample rate. The derivative is therefore low-pass filtered with a time constant $T_f = tau_D \/ N$, typically with $N$ between 5 and 20.
- _Precomputed coefficients._ With the backward difference, each term becomes a recursion whose coefficients depend only on the gains and $T_s$ and are computed once.

The resulting _positional_ form is

$ P[k] &= K_p e[k] \
  I[k] &= I[k - 1] + K_i T_s e[k] \
  D[k] &= alpha D[k - 1] - beta (y[k] - y[k - 1]), quad alpha = T_f / (T_f + T_s), quad beta = K_d / (T_f + T_s) \
  u[k] &= "sat"(P[k] + I[k] + D[k]) $ <eq:pid_positional>

where $"sat"(v)$ limits $v$ to the range $plus.minus u_max$ the motors can deliver.
All of it costs a handful of multiplications and additions per sample.

=== The incremental form

The _incremental_ or _velocity form_#index("PID controller!incremental form") computes only the change of the output and adds it to the previous output:

$ Delta u[k] &= K_p (e[k] - e[k - 1]) + K_i T_s e[k] + (D[k] - D[k - 1]) \
  u[k] &= "sat"(u[k - 1] + Delta u[k]) $ <eq:pid_incremental>

As long as nothing saturates, both forms produce the same output.
The difference is where the integral lives: in the incremental form, the output itself is the integrator, and since it is saturated in every sample, it cannot run away.
Gains can also be changed in flight without a jump in the output, and switching between manual and automatic control is seamless.
In fixed point (e.g. Q16.16), the form needs nothing but a saturating addition at the end.

=== Windup <subsec:windup>

What happens without it is shown in @fig:windup, for a step of #qty("0.8", "rad") with the output limited to #qty("0.15", "N").
While the output is saturated, the vehicle accelerates as fast as it can and the error shrinks only slowly, but the integral keeps growing.
When the angle finally reaches the reference, the integral holds a large value that first has to be worked off, and the angle overshoots, here by 268 % (at which point the real vehicle would be upside down and the linear model long invalid).

_Conditional integration_#index("anti-windup") stops the integral whenever the output is saturated and the error would drive it further into saturation, which reduces the overshoot to 65 %; the incremental form reaches the reference without overshoot.

#figure(
  placement: auto,
  {
    windup()
    caption-rule
  },
  caption: flex-caption(
    [Integrator windup],
    [Step of #qty("0.8", "rad") with the angle PID of @subsec:pid_state_feedback and the output limited to #qty("0.15", "N"), without anti-windup, with conditional integration and in incremental form. Top: angle; bottom: command.],
  ),
) <fig:windup>

=== PID as state feedback <subsec:pid_state_feedback>

On a quadrotor, the derivative of the angle does not have to be computed at all, since the gyroscope measures it.
With $dot(e) = -omega$ for a constant reference, a PID controller on the angle is

$ u = K_p (r - theta) - K_d omega + K_i x_i $ <eq:pid_gyro>

which is the state feedback of @sec:integral with $bold(K) = mat(K_p, K_d, 0, 0)$ and $k_i = K_i$: a PID controller is a state feedback controller that ignores the motor lag and the delay.
This also gives a way of choosing PID gains: from the LQR of @sec:integral, $K_p = #qty("15.2", "N/rad")$, $K_d = #qty("1.40", "N s/rad")$ and $K_i = 48.4$ #unit("N/(rad s)").
Whether ignoring the other two states is acceptable depends on how fast the motors are compared to the closed loop; for the reference airframe, it is (@fig:cascade).

=== Cascaded loops <subsec:cascade>

Flight controllers commonly split the axis into two loops: an inner loop that controls the rate $omega$ from the gyroscope, and an outer loop that computes the rate reference from the angle error.
The inner loop sees only the motor lag, the delay and one integrator, and runs on the fastest and cleanest signal; the outer loop sees the inner loop as a fast, well-behaved integrator and gets away with a proportional gain.
This structure also makes tuning by rules possible, as the next section shows.

#figure(
  placement: auto,
  {
    cascade()
    caption-rule
  },
  caption: flex-caption(
    [Cascaded PID, angle PID and LQR with integral action],
    [Step of #qty("0.1", "rad") and a thrust offset of #qty("0.05", "N") from #qty("1.5", "s") on, with measurement noise: cascaded rate PI and angle P, the angle PID of @eq:pid_gyro with gains from the LQR, and LQR with integral action.],
  ),
) <fig:cascade>

=== Why flight controllers use PID <subsec:why_pid>

State feedback is the more general tool: @subsec:pid_state_feedback showed that a PID controller is state feedback with two of the four gains set to zero, so anything a PID controller can do, state feedback can do as well.
Still, the widely used open-source flight controllers (PX4, ArduPilot and Betaflight among them) run cascaded PID loops, and there are good reasons for it.

- _No model needed._ State feedback needs $bold(Phi)$ and $bold(Gamma)$ for the design and, since not every state is measured, for the observer that runs in flight. A PID controller needs neither; its gains can be found on the vehicle, by experiment, one loop at a time.
- _Gains that mean something._ "More P" makes the response stiffer, "more D" damps it, "more I" removes the drift. A pilot can tune by feel, which is not true for the entries of $bold(K)$ that act on an estimated thrust difference.
- _The missing states are small here._ The two states that a PID controller ignores are the thrust difference and the delayed command, and both only matter if the motors are slow compared to the closed loop. For a small quadrotor, they are not: in @fig:cascade, the cascade does as well as LQR with integral action, with less overshoot.
- _The axes are nearly independent._ Around hover, roll, pitch and yaw hardly influence each other, so three single-input loops cover the vehicle. Where state feedback really pulls ahead, with many inputs acting on coupled states, a quadrotor near hover gives it little to work with.
- _Robustness to a wrong model._ An observer built on a wrong model produces wrong estimates, and the controller acts on them. PID acts on measurements only, so a wrong guess of the motor time constant costs performance, not correctness.

Racing and freestyle quadrotors, which flip and roll on command, are no exception: they too fly cascaded PID, and in their most common mode only the inner rate loop, with the pilot closing the angle loop by hand.
That works because the rate loop does not care about the attitude.
The torques act on the body rates in the same way whether the vehicle is level or upside down, and gravity, which acts at the center of gravity, adds no torque; the chain of @fig:axis_chain from command to rate holds throughout a flip, apart from a coupling between the axes that grows with the square of the rotation rates and that the loops treat as a disturbance.
What does change far from hover is everything around it: the angles of @sec:frames stop being a good description of the attitude, the thrust points sideways, and at full throttle the motors have no headroom left for the thrust differences.
Their firmware handles this with additions to PID rather than with a model, e.g. by lowering the gains as the throttle rises, since the thrust per unit of command grows with the rotor speed (@sec:ident_what), which amounts to scheduling the gains by hand.
Their light rotors on fast motors also keep the states that PID ignores small.

The advantage of model-based control shows where these reasons stop holding: slow or heavy motors whose lag matters, payloads that swing, autonomous flight at the limits of thrust and attitude, where the controller rather than a pilot has to deal with saturation and the direction of the thrust, or whenever the gains should follow a changing model automatically, which is where @cha:identification leads.
Research on aggressive autonomous flight therefore tends to use the model, e.g. incremental nonlinear dynamic inversion, which uses measured angular accelerations and the known effect of the motors to cancel the dynamics @smeur2016indi.
Both designs are kept here for that reason: the cascade as the fallback and sanity check, state feedback as the design that can make use of a good model.

== Tuning by rules <sec:tuning_rules>

Without a model, PID gains are tuned by experiment, and a number of tables translate the result of a simple experiment into gains.
Most of them are not applicable to an attitude axis, and it is worth knowing why.

The classic _reaction curve_ methods (the step-response method of Ziegler and Nichols @ziegler1942, and the tables of Chien, Hrones and Reswick and of Cohen and Coon that followed it) apply a step to the plant without controller and read a gain, a delay and a time constant off the response.
They assume a _self-regulating_ plant, one that settles at a new value after a step; @fig:open_loop shows that the attitude axis does not.
The angle diverges, and even the rate, which has one integrator less, keeps growing as long as a thrust difference acts.

The _ultimate gain_#index("Ziegler–Nichols method") method of Ziegler and Nichols @ziegler1942 needs no open-loop step.
A proportional controller is closed around the plant and its gain increased until the loop oscillates steadily; this _ultimate gain_ $K_u$ and the period $T_u$ of the oscillation yield the PID gains $K_p = 0.6 K_u$, $tau_I = T_u \/ 2$ and $tau_D = T_u \/ 8$.
On the angle, even this fails: with two integrators and a lag, the phase of @eq:axis_tf is below $-180°$ at every frequency, so proportional control alone is unstable for every gain and never oscillates steadily.
On the rate, which has one integrator, the method works.
For the reference airframe, $K_u = #qty("9.98", "N s/rad")$ and $T_u = #qty("0.061", "s")$, which yields $K_p = 5.99$, $K_i = 198$ and $K_d = 0.045$.
The rules were designed for a quarter-amplitude decay, which is aggressive by today's standards, and the response in @fig:rate_tuning overshoots by 17 % and passes the gyroscope noise on to the motors with a standard deviation of #qty("0.28", "N"), a quarter of the available range.

Skogestad's SIMC rules @skogestad2003simc are a modern alternative that comes with rules for integrating plants.
For a plant $k e^(-theta_d s) \/ (s (tau_2 s + 1))$, an integrator with a lag $tau_2$ and a delay $theta_d$, and a desired closed-loop time constant $tau_c$, they give a series PID with

$ K_c = 1 / (k (tau_c + theta_d)), quad tau_I = 4 (tau_c + theta_d), quad tau_D = tau_2 $ <eq:simc>

The rate loop is exactly this plant with $k = b$, $tau_2 = tau_m$ and $theta_d = 1.5 T_s$ (one sample of delay, half a sample for the zero-order hold).
With $tau_c = #qty("20", "ms")$ and converted to the parallel form of @eq:pid_parallel, this yields $K_p = 1.70$, $K_i = 13.9$ and $K_d = 0.038$, a response with 15 % overshoot and half the noise of Ziegler–Nichols.
The only tuning knob is $tau_c$, which makes SIMC a good starting point; it does need the model parameters, though, which @cha:identification provides.

For the cascade, the outer angle loop follows the same idea: it sees the closed rate loop as an integrator with a time constant of about $tau_c$, and a proportional gain of $1 \/ (4 tau_c) = #qty("12.5", "1/s")$ keeps it a factor of four slower than the inner loop.
@fig:cascade shows the result with the inner loop from the next section.

#figure(
  placement: auto,
  {
    rate-tuning()
    caption-rule
  },
  caption: flex-caption(
    [Rate loop tuned by rules and by search],
    [Rate step of #qty("1", "rad/s") and a thrust offset of #qty("0.05", "N") after #qty("0.5", "s"), with gyroscope noise, for the rate PID tuned by Ziegler--Nichols, by SIMC and by twiddle. Top: rate; bottom: command.],
  ),
) <fig:rate_tuning>

== Tuning by search <sec:twiddle>

With a model at hand, the gains can also be searched for numerically: simulate a step, compute a cost, change the gains, repeat.
A particularly simple search is _twiddle_#index("twiddle"), which Sebastian Thrun teaches in his course on artificial intelligence for robotics @thrun_cs373.
It tries each parameter in turn with a step $Delta p_i$ up and, if that does not help, down:

+ For each parameter $p_i$: add $Delta p_i$ and evaluate the cost.
+ If the cost improved, keep the change and grow $Delta p_i$ by 10 %.
+ Otherwise, try $p_i - Delta p_i$; if that improved the cost, keep it and grow $Delta p_i$ by 10 %.
+ Otherwise, restore $p_i$ and shrink $Delta p_i$ by 10 %.
+ Repeat until the sum of all $Delta p_i$ is small.

It is a coordinate descent with adaptive step sizes, needs no gradients and finds a local minimum, which for three PID gains is usually good enough.
Everything depends on the cost: twiddle optimizes exactly what is asked for and nothing else.
For the rate loop, the cost is the integral of the time-weighted absolute error, $"ITAE" = integral t |e(t)| dif t$, of the step in @fig:rate_tuning (the factor $t$ makes errors that linger late in the response expensive), plus a penalty on the sum of squared changes of the output from one sample to the next, which is where the gyroscope noise shows up.
Starting from the SIMC gains, twiddle reduces this cost from 0.22 to 0.024 in 127 sweeps (@fig:twiddle) and ends at $K_p = 1.20$, $K_i = 7.84$ and $K_d approx 0$.

#figure(
  placement: auto,
  {
    twiddle-cost()
    caption-rule
  },
  caption: flex-caption(
    [Twiddle convergence],
    [Best cost per sweep of twiddle for the rate PID, starting from the SIMC gains.],
  ),
) <fig:twiddle>

Twiddle removed the derivative term altogether, since it passes more noise to the motors than it buys in speed, and accepted 35 % overshoot in exchange for a command with almost ten times less noise than with SIMC.
Whether that is the better controller depends on the motors, the propellers and the pilot, which is the point: the cost function is where the actual design decision is made, and twiddle only executes it.
Since every evaluation is a complete step response, and some of the gains tried along the way are unstable, twiddle belongs on the simulated plant, never on a vehicle in flight.

== Choosing a controller <sec:choosing>

@fig:cascade compares three of the designs on the same task.
The cascade of the twiddle-tuned rate PI and a proportional angle loop overshoots by 11 %, the angle PID with gains from the LQR by 15 %, and LQR with integral action by 27 %, the latter for the reason given in @sec:integral; all three reject the thrust offset.

For the flight controller, the plan follows from this chapter: LQR with integral action and an observer for the full state as the main design, since its weights describe the requirements directly and it accounts for the motor lag and delay; and a cascaded PID with SIMC gains as a fallback and sanity check, since it is easy to reason about and needs only two parameters of the model.
Both need the model parameters $b$ and $tau_m$, and the next chapter is about getting them.
