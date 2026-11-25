// Simulated responses of one attitude axis for the chapter on state-space
// control. Data written by matlab/attitude_control.m.
#import "common.typ": *

#let panels(..plots) = cetz.canvas({
  import cetz.draw: *
  for (i, p) in plots.pos().enumerate() {
    group({
      translate((0, -i * 5.6))
      p
    })
  }
})

/// Open loop: a short thrust pulse leaves the vehicle rotating.
#let open-loop() = {
  let d = load("open_loop", every: 2)
  cetz.canvas(time-plot(d, (("theta", $theta$), ("omega", $omega$, "dashed")),
    y-label: [$theta$ in rad, $omega$ in rad/s], size: (12, 4), legend: "inner-north-west",
    x-tick-step: 0.2, y-tick-step: 0.02))
}

#import "poles.typ": pair, placement-scenarios

/// Pole placement for three natural frequencies at the same damping.
#let placement-wn() = placement-scenarios(load("pp_wn", every: 3), (
  ($omega_n = 8$, pair(8, 0.7), "theta_8", "u_8", "theta_8_err"),
  ($omega_n = 15$, pair(15, 0.7), "theta_15", "u_15", "theta_15_err"),
  ($omega_n = 25$, pair(25, 0.7), "theta_25", "u_25", "theta_25_err"),
  ($omega_n = 40$, pair(40, 0.7), "theta_40", "u_40", "theta_40_err"),
), common: ((-40, 0),), map: (-50, 5, 35, 10, 10))

/// Pole placement for three damping ratios at the same natural frequency.
#let placement-zeta() = placement-scenarios(load("pp_zeta", every: 3), (
  ($zeta = 0.3$, pair(15, 0.3), "theta_03", none),
  ($zeta = 0.7$, pair(15, 0.7), "theta_07", none),
  ($zeta = 1$, pair(15, 1), "theta_10", none),
), common: ((-40, 0),))

/// The same dominant pair with the motor pole moved further and further left,
/// with saturation and measurement noise.
#let placement-motor() = placement-scenarios(load("pp_motor", every: 2), (
  ([motor $-40$], ((-40, 0),), "theta_40", "u_40"),
  ([motor $-130$], ((-130, 0),), "theta_130", "u_130"),
  ([motor $-400$], ((-400, 0),), "theta_400", "u_400"),
), common: pair(15, 0.7), map: (-420, 20, 20, 200, 10), u-step: 0.5)

/// LQR for three scalings of the input weight.
#let lqr-rho() = {
  let d = load("lqr_rho", every: 3)
  panels(
    time-plot(d, (("theta_01", $rho = 0.1$), ("theta_1", $rho = 1$), ("theta_10", $rho = 10$)),
      y-label: [$theta$ in rad], size: (12, 4), x-tick-step: 0.25, y-tick-step: 0.02),
    time-plot(d, (("u_01", $rho = 0.1$), ("u_1", $rho = 1$), ("u_10", $rho = 10$)),
      y-label: [$u$ in N], size: (12, 4), legend: "inner-north-east", x-tick-step: 0.25, y-tick-step: 0.25),
  )
}

/// Integral action against a thrust offset, and the observer's thrust estimate.
#let integral-action() = {
  let d = load("integral", every: 2)
  panels(
    time-plot(d, (("theta_p", [LQR]), ("theta_i", [LQR + integral])),
      y-label: [$theta$ in rad], ref: "r", size: (12, 4), y-min: -0.01, y-max: 0.13, x-tick-step: 0.5, y-tick-step: 0.02),
    time-plot(d, (("td", [$Delta T$]), ("td_hat", [$Delta hat(T)$], "dashed")),
      y-label: [$Delta T$ in N], size: (12, 4), legend: "inner-north-east", x-tick-step: 0.5, y-tick-step: 0.1),
  )
}

/// Windup: a large step against a tight output limit.
#let windup() = {
  let d = load("windup", every: 3)
  panels(
    time-plot(d, (("theta_none", [no anti-windup]), ("theta_clamp", [conditional integration]), ("theta_inc", [incremental form])),
      y-label: [$theta$ in rad], ref: "r", size: (12, 4), legend: "inner-south-east", x-tick-step: 0.5, y-tick-step: 0.5),
    time-plot(d, (("u_none", [no anti-windup]), ("u_clamp", [conditional integration])),
      y-label: [$u$ in N], size: (12, 3.5), legend: "inner-south-east", y-min: -0.2, y-max: 0.2, x-tick-step: 0.5, y-tick-step: 0.1),
  )
}

/// Rate loop tuned by Ziegler-Nichols, SIMC and twiddle.
#let rate-tuning() = {
  let d = load("rate_tuning")
  panels(
    time-plot(d, (("omega_zn", [Ziegler--Nichols]), ("omega_simc", [SIMC]), ("omega_tw", [twiddle])),
      y-label: [$omega$ in rad/s], ref: "r", size: (12, 4), x-tick-step: 0.1, y-tick-step: 0.25),
    time-plot(d, (("u_zn", [Ziegler--Nichols]), ("u_simc", [SIMC]), ("u_tw", [twiddle])),
      y-label: [$u$ in N], size: (12, 4), legend: "inner-north-east", x-tick-step: 0.1, y-tick-step: 0.5),
  )
}

/// Twiddle: best cost per sweep.
#let twiddle-cost() = {
  let d = load("twiddle")
  cetz.canvas(plot.plot(
    size: (12, 3.5),
    x-label: [sweep], y-label: [cost],
    x-min: 0, y-min: 0, x-tick-step: 20, y-tick-step: 0.05,
    {
      plot.add(d.sweep.zip(d.cost), style: (stroke: (paint: palette.at(0), thickness: 0.9pt)))
    },
  ))
}

/// Cascaded PID, PID from the LQR gains and LQR with integral action.
#let cascade() = {
  let d = load("cascade", every: 2)
  cetz.canvas(time-plot(d,
    (("theta_cascade", [cascade]), ("theta_pid", [angle PID]), ("theta_lqr_i", [LQR + integral])),
    y-label: [$theta$ in rad], ref: "r", size: (12, 4.5), y-min: -0.01, y-max: 0.14, x-tick-step: 0.5, y-tick-step: 0.02))
}
