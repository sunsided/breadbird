// Figures for the chapter on identifying the plant. Data written by the
// bring-up example of the control crate (control/examples/bringup.rs).
#import "common.typ": *

/// (x, y) pairs of two columns, without rows where either is NaN.
#let finite(d, x, y) = d.at(x).zip(d.at(y)).filter(((a, b)) => not float.is-nan(a) and not float.is-nan(b))

/// A thin horizontal reference line at `y` over `[x0, x1]`.
#let ref-line(x0, x1, y) = plot.add(((x0, y), (x1, y)), style: (stroke: (paint: gray, thickness: 0.6pt, dash: "dashed")))

/// Closed-loop identification: soft stabilizer, PRBS on its output, the
/// regressors and the estimators.
#let ident-setup() = cetz.canvas({
  import cetz.draw: *
  sum-node((2.4, 0), "s")
  block-node((0.4, 0), [stabilizer], "stab", w: 1.9)
  block-node((2.4, 1.4), [PRBS], "prbs", w: 1.2)
  block-node((4.6, 0), [airframe], "plant", w: 1.7)
  block-node((7.6, 0), [gyroscope], "gyro", w: 1.8)
  block-node((7.6, -2.6), [regressor], "reg", w: 1.8)
  block-node((4.6, -2.6), [RLS], "rls", w: 1.2)
  block-node((1.6, -2.6), [checks], "chk", w: 1.4)

  line("stab.east", "s.west", mark: to)
  line("prbs.south", "s.north", mark: to)
  line("s.east", "plant.west", mark: to)
  content((3.15, 0.3), $u$)
  line("plant.east", "gyro.west", mark: to)
  line("gyro.south", "reg.north", mark: to)
  content((7.85, -1.2), $omega$)
  line((3.2, 0), (3.2, -1.2), (6.9, -1.2), (6.9, -2.2), mark: to)
  line("reg.west", "rls.east", mark: to)
  content((5.95, -2.35), $bold(phi), y$)
  line("rls.west", "chk.east", mark: to)
  content((3.1, -2.35), $hat(theta), bold(P)$)
  line("gyro.north", (7.6, 2.3), (0.4, 2.3), "stab.north", mark: to)
  content((1.6, -3.3), text(size: 0.8em)[excitation, identifiability])
})

/// Estimates over the identification run: the biased motor pole of the
/// three-parameter model, the motor time constant of the grid search, and the
/// input gain of the one-parameter RLS.
#let ident-trajectories() = {
  let d = load("bringup_ident")
  let first-identified = range(d.t.len())
    .filter(i => not float.is-nan(d.gain_identified.at(i)) and d.gain_identified.at(i) > 0.5)
    .map(i => d.t.at(i))
    .first()
  let panel(body, y-label, y-min, y-max, y-step) = plot.plot(
    size: (12, 3.6), x-label: [$t$ in s], y-label: y-label,
    x-min: 0, x-max: 40, y-min: y-min, y-max: y-max,
    x-tick-step: 5, y-tick-step: y-step,
    legend: "inner-south-east",
    body,
  )
  cetz.canvas({
    import cetz.draw: *
    panel({
      ref-line(0, 20, calc.exp(-0.002 / 0.025))
      plot.add(finite(d, "t", "a_arx"), style: (stroke: palette.at(1) + 0.9pt), label: [$hat(a)$, rate model])
    }, [motor pole], -1, 1.2, 0.5)
    group({
      translate((0, -5.0))
      panel({
        ref-line(0, 20, 25)
        plot.add(finite(d, "t", "tau_search").map(((t, v)) => (t, v * 1000)), style: (stroke: palette.at(0) + 0.9pt),
          label: [$hat(tau)_m$, grid search])
      }, [$tau_m$ in ms], 15, 35, 5)
    })
    group({
      translate((0, -10.0))
      panel({
        ref-line(20, 40, 30)
        plot.add(((first-identified, 20), (first-identified, 40)), style: (stroke: (paint: palette.at(2), thickness: 0.6pt, dash: "dotted")))
        plot.add(finite(d, "t", "b_gain"), style: (stroke: palette.at(0) + 0.9pt), label: [$hat(b)$, one-parameter RLS])
      }, [$b$ in rad/s²/N], 20, 40, 5)
    })
  })
}

/// Residual and fitted gain over the candidate motor time constants.
#let lag-residual() = {
  let d = load("bringup_lag")
  let ms = d.tau.map(t => t * 1000)
  cetz.canvas({
    import cetz.draw: *
    plot.plot(
      size: (12, 3.6), x-label: [assumed $tau_m$ in ms], y-label: [RMS residual in rad/s²],
      x-min: 5, x-max: 65, x-tick-step: 10,
      y-format: v => str(calc.round(v, digits: 3)), y-tick-step: 0.01,
      {
        plot.add(((25, 7.17), (25, 7.235)), style: (stroke: (paint: gray, thickness: 0.6pt, dash: "dashed")))
        plot.add(ms.zip(d.rms), style: (stroke: palette.at(0) + 0.9pt), mark: "o", mark-size: 0.12,
          mark-style: (stroke: palette.at(0), fill: white))
      },
    )
    group({
      translate((0, -5.0))
      plot.plot(
        size: (12, 3.6), x-label: [assumed $tau_m$ in ms], y-label: [fitted $b$ in rad/s²/N],
        x-min: 5, x-max: 65, x-tick-step: 10, y-min: 15, y-max: 45, y-tick-step: 10,
        {
          plot.add(((25, 15), (25, 45)), style: (stroke: (paint: gray, thickness: 0.6pt, dash: "dashed")))
          plot.add(((5, 30), (65, 30)), style: (stroke: (paint: gray, thickness: 0.6pt, dash: "dashed")))
          plot.add(ms.zip(d.b), style: (stroke: palette.at(1) + 0.9pt), mark: "o", mark-size: 0.12,
            mark-style: (stroke: palette.at(1), fill: white))
        },
      )
    })
  })
}

/// The step with gains for the placeholder airframe and for the identified one,
/// flown on the simulated airframe.
#let bringup-step() = {
  let d = load("bringup_step", every: 2)
  cetz.canvas({
    import cetz.draw: *
    time-plot(d, (("theta_prior", [placeholder gains]), ("theta", [identified gains])),
      y-label: [$theta$ in rad], ref: "r", size: (12, 4), y-min: -0.01, y-max: 0.14, x-tick-step: 0.25, y-tick-step: 0.02)
    group({
      translate((0, -5.4))
      time-plot(d, (("td", [$Delta T$]), ("td_hat", [$Delta hat(T)$], "dashed")),
        y-label: [$Delta T$ in N], size: (12, 3.6), legend: "inner-north-east", x-tick-step: 0.25, y-tick-step: 0.25)
    })
  })
}
