// Pole figures for the chapter on state-space control: what a pole position
// means for the response, and where the designs put their poles. Closed-loop
// data written by matlab/attitude_control.m.
#import "common.typ": *

/// Pole positions in the s-plane, each with a sketch of its response
/// e^(σt) cos(ωt). Only the upper half plane is drawn; complex poles come with
/// their mirror image below the axis.
#let pole-gallery() = cetz.canvas({
  import cetz.draw: *
  let sx = 3.6 // cm per unit of σ
  let sy = 2.6 // cm per unit of ω
  let x0 = 2.4 * sx // canvas x of σ = 0
  let px(s) = x0 + s * sx

  // axes
  line((0, 0), (px(1.1), 0), mark: to)
  line((x0, -0.3), (x0, 3.2 * sy), mark: to)
  content((px(1.1) + 0.1, 0), anchor: "west", text(size: 0.85em)[$"Re" s = sigma$])
  content((x0, 3.2 * sy + 0.1), anchor: "south", text(size: 0.85em)[$"Im" s = omega$])
  rect((0, -0.3), (x0, 3.2 * sy), fill: rgb("#0072B2").transparentize(93%), stroke: none)
  content((0.2, 3.0 * sy), anchor: "west", text(size: 0.8em, fill: rgb("#0072B2"))[stable: decays])
  content((x0 + 0.2, 3.0 * sy), anchor: "west", text(size: 0.8em, fill: rgb("#D55E00"))[unstable: grows])

  let sketch(s, w) = {
    let (bx, by) = (px(s) - 0.8, w * sy + 0.35)
    let (bw, bh) = (1.6, 1.1)
    rect((bx, by), (bx + bw, by + bh), stroke: 0.3pt + gray, fill: white)
    line((bx, by + bh / 2), (bx + bw, by + bh / 2), stroke: 0.3pt + gray)
    let tmax = 5.0
    let ys = range(0, 121).map(i => {
      let t = i / 120 * tmax
      calc.exp(s * t) * calc.cos(w * t)
    })
    let peak = calc.max(..ys.map(calc.abs))
    let pts = ys.enumerate().map(((i, v)) => (bx + i / 120 * bw, by + bh / 2 + v / peak * 0.7 * (bh / 2)))
    let color = if s < 0 { rgb("#0072B2") } else if s == 0 { black } else { rgb("#D55E00") }
    line(..pts, stroke: 0.8pt + color)
    line((px(s), w * sy), (px(s), by), stroke: (paint: gray, thickness: 0.3pt, dash: "dotted"))
  }

  for s in (-2.0, -1.0, 0.0, 0.6) {
    for w in (0.0, 2.0) {
      sketch(s, w)
      content((px(s), w * sy), text(size: 1.1em, fill: if s > 0 { rgb("#D55E00") } else { black }, sym.times))
    }
  }
})

/// Closed-loop poles of the designs in the s-plane, and how the LQR poles move
/// with the input weight.
#let closed-loop-poles() = {
  let d = load("cl_poles")
  let loc = load("lqr_locus")
  let pts(kind, wn: none) = range(d.re.len())
    .filter(i => d.kind.at(i) == kind and (wn == none or d.wn.at(i) == wn))
    .map(i => (d.re.at(i), d.im.at(i)))
    .filter(((x, y)) => x >= -50 and calc.abs(y) <= 25)
  let marker(data, mark, color, label) = plot.add(data, style: (stroke: none), mark: mark, mark-size: 0.16,
    mark-style: (stroke: color + 0.9pt, fill: none), label: label)

  cetz.canvas(plot.plot(
    size: (12, 7),
    x-label: [$"Re" s$ in 1/s], y-label: [$"Im" s$ in 1/s],
    x-min: -50, x-max: 5, y-min: -25, y-max: 25,
    x-tick-step: 10, y-tick-step: 10,
    legend: "inner-north-west",
    {
      plot.add(range(loc.re.len()).map(i => (loc.re.at(i), loc.im.at(i))).filter(((x, y)) => x >= -50 and calc.abs(y) <= 25),
        style: (stroke: none), mark: "o", mark-size: 0.04, mark-style: (fill: gray, stroke: none),
        label: [LQR, $rho = 10^(-3) dots 10^3$])
      // constant damping zeta = 0.7: rays at +-45.6 degrees from the negative real axis
      plot.add(((0, 0), (-24.5, 25)), style: (stroke: (paint: gray, dash: "dashed", thickness: 0.5pt)))
      plot.add(((0, 0), (-24.5, -25)), style: (stroke: (paint: gray, dash: "dashed", thickness: 0.5pt)))
      marker(pts(0, wn: 8), "o", palette.at(0), [placement, $omega_n = 8$])
      marker(pts(0, wn: 15), "o", palette.at(1), [placement, $omega_n = 15$])
      marker(pts(0, wn: 25), "o", palette.at(2), [placement, $omega_n = 25$])
      marker(pts(1), "square", palette.at(3), [LQR, $rho = 1$])
      marker(pts(2), "x", black, [observer])
    },
  ))
}

/// The poles `(re, im)` of a pair with natural frequency `wn` and damping
/// `zeta`; a double real pole for `zeta = 1`.
#let pair(wn, zeta) = {
  let wd = wn * calc.sqrt(calc.max(0, 1 - zeta * zeta))
  ((-zeta * wn, wd), (-zeta * wn, -wd))
}

/// "Place the poles here, this happens": a pole map of several placements next
/// to their simulated responses, matched by color. `scenarios` is an array of
/// (label, poles, theta-column, u-column or none) tuples, optionally with a fifth
/// element: a theta column of the same gains on a mismatched plant, drawn
/// dotted in the same color. `poles` are the
/// poles that differ between the scenarios; `common` are the poles all of them
/// share, drawn in gray. `map` is (x-min, x-max, y-max, x-step, y-step) of the
/// pole map.
#let placement-scenarios(d, scenarios, common: (), map: (-50, 5, 20, 10, 10), x-step: 0.25, u-step: 0.25) = cetz.canvas({
  import cetz.draw: *
  let (x-min, x-max, y-max, xs, ys) = map
  plot.plot(
    size: (4.2, 4.2),
    x-label: [$"Re" s$], y-label: [$"Im" s$],
    x-min: x-min, x-max: x-max, y-min: -y-max, y-max: y-max,
    x-tick-step: xs, y-tick-step: ys,
    {
      plot.add(((x-min, 0), (x-max, 0)), style: (stroke: (paint: gray, thickness: 0.4pt)))
      plot.add(((0, -y-max), (0, y-max)), style: (stroke: (paint: gray, thickness: 0.4pt)))
      if common.len() > 0 {
        plot.add(common, style: (stroke: none), mark: "x", mark-size: 0.22, mark-style: (stroke: gray + 1.2pt))
      }
      for (i, sc) in scenarios.enumerate() {
        plot.add(sc.at(1), style: (stroke: none), mark: "x", mark-size: 0.22,
          mark-style: (stroke: palette.at(i) + 1.2pt))
      }
    },
  )

  let has-u = scenarios.any(sc => sc.at(3) != none)
  group({
    translate((6.0, if has-u { 1.6 } else { 0.8 }))
    let lines = scenarios.enumerate().map(((i, sc)) => (sc.at(2), sc.at(0), none, i))
    let dotted = scenarios.enumerate().filter(((i, sc)) => sc.len() > 4)
      .map(((i, sc)) => (sc.at(4), none, "dotted", i))
    time-plot(d, lines + dotted,
      y-label: [$theta$ in rad], size: (7.4, 2.6), x-tick-step: x-step, y-tick-step: 0.05,
      legend: if has-u { none } else { "inner-south-east" })
  })
  if has-u {
    group({
      translate((6.0, -2.4))
      time-plot(d, scenarios.map(sc => (sc.at(3), sc.at(0))),
        y-label: [$u$ in N], size: (7.4, 2.6), x-tick-step: x-step, y-tick-step: u-step, legend: "inner-south-east")
    })
  }
})
