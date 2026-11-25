// Shared loader and plot setup for the momentum theory energy plots
// (latex/pictures/mt_energy/*.tikz, data from latex/matlab/momentum_theory_energy.m).
#import "../common.typ": cetz
#import "@preview/cetz-plot:0.1.4": plot
#import "@preview/unify:0.8.1": unit, qty

/// Thrust levels plotted in the figures, by CSV file name.
#let forces = ("0.5", "1", "2", "4")

/// Columns of one CSV file as floats: r, A, Q, rho, mdot, v0, vi, v3, F, E.
#let load(force) = (
  csv("/data/mt_energy/" + force + "N.csv", row-type: dictionary)
    .map(row => row.pairs().map(((k, v)) => (k, float(v))).to-dict())
)

#let labels = (
  A: [$A_r$ in #unit("m^2")],
  v3: [$v_3$ in #unit("m/s")],
  E: [$E_"kin"$ in #unit("J")],
)

/// pgfplots default cycle list (color + marks).
#let styles = (
  (color: blue, mark: "o"),
  (color: red, mark: "square"),
  (color: rgb(75%, 50%, 25%), mark: "triangle"),
  (color: black, mark: "*"),
)

#let decade(v) = $10^#int(calc.round(calc.log(v)))$

/// The plot body (no canvas), so several plots can share one canvas.
#let mt-plot(x, y, log: false, legend: "inner-north-east", show-legend: true, size: (12, 6), name: none, ..args) = {
  let mode = if log { "log" } else { "lin" }
  let log-axis = if log {
    (x-tick-step: 1, x-minor-tick-step: 1, x-format: decade,
     y-tick-step: 1, y-minor-tick-step: 1, y-format: decade)
  } else {
    let fmt(v) = str(calc.round(v, digits: 4))
    (x-format: fmt, y-format: fmt)
  }
  plot.plot(
    size: size,
    name: name,
    x-mode: mode, y-mode: mode,
    x-label: labels.at(x), y-label: labels.at(y),
    legend: if show-legend { legend } else { none },
    ..log-axis,
    ..args,
    {
      for (i, f) in forces.enumerate() {
        let st = styles.at(i)
        plot.add(
          load(f).map(row => (row.at(x), row.at(y))),
          style: (stroke: st.color),
          mark: st.mark, mark-size: 0.08, mark-style: (stroke: st.color, fill: st.color),
          label: if show-legend [$F = #qty(f, "N")$] else { none },
        )
      }
    },
  )
}

/// A single plot in its own canvas.
#let mt-figure(..args) = cetz.canvas(mt-plot(..args))
