// Shared loader and plot setup for the momentum theory power plots
// (latex/pictures/mt_energy/*.tikz). Data: data/mt_energy/*.csv, written by
// matlab/momentum_theory_energy.m (copied from latex/data/).
#import "../common.typ": cetz
#import "@preview/cetz-plot:0.1.4": plot
#import "@preview/unify:0.8.1": unit, qty

/// Thrust levels plotted in the figures, by CSV file name.
#let forces = ("0.5", "1", "2", "4")

/// Columns of one CSV file as floats: r, A, Q, rho, mdot, v0, vi, v3, F, and
/// P (the CSV's `E` column: F vi, the kinetic energy added to the air per
/// second, i.e. the ideal power in W).
#let load(force) = {
  let rows = csv("/data/mt_energy/" + force + "N.csv", row-type: dictionary)
    .map(row => row.pairs().map(((k, v)) => (k, float(v))).to-dict())
    .map(row => row + (P: row.E))
  // The data must satisfy the chapter's momentum theory: F = mdot v3 and
  // P = sqrt(F^3 / (2 rho A)). (An earlier version of the script computed
  // every series for half its nominal thrust.)
  for row in rows {
    assert(calc.abs(row.mdot * row.v3 - row.F) < 1e-6 * row.F,
      message: force + "N.csv: mdot v3 != F; regenerate with matlab/momentum_theory_energy.m")
    assert(calc.abs(row.P - calc.sqrt(calc.pow(row.F, 3) / (2 * row.rho * row.A))) < 1e-6 * row.P,
      message: force + "N.csv: E != sqrt(F^3 / (2 rho A))")
  }
  rows
}

#let labels = (
  A: [$A_r$ in #unit("m^2")],
  v3: [$v_3$ in #unit("m/s")],
  P: [$P$ in #unit("W")],
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
