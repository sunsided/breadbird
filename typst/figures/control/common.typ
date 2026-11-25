// Shared loader and plot setup for the control figures. Data:
// data/control/*.csv, written by matlab/attitude_control.m.
#import "../common.typ": cetz, to
#import "@preview/cetz-plot:0.1.4": plot

/// Columns of data/control/<name>.csv as a dictionary of float arrays.
/// `every` keeps every n-th row, which is plenty for 500 Hz step responses.
#let load(name, every: 1) = {
  let rows = csv("../../data/control/" + name + ".csv")
  let header = rows.first()
  let body = rows.slice(1).enumerate().filter(((i, _)) => calc.rem(i, every) == 0).map(((_, r)) => r)
  header.enumerate().map(((i, h)) => (h, body.map(r => float(r.at(i))))).to-dict()
}

/// Line colors, colorblind-safe (Okabe-Ito) and distinct in grayscale.
#let palette = (rgb("#0072B2"), rgb("#D55E00"), rgb("#009E73"), rgb("#CC79A7"), black)

/// (x, y) pairs from two columns.
#let series(d, x, y) = d.at(x).zip(d.at(y))

/// A time plot of several columns of one CSV. `lines` is an array of
/// (column, label), (column, label, dash) or (column, label, dash, color index)
/// tuples; the color defaults to the position in `lines`. `ref` adds the
/// reference column as a thin gray line.
#let time-plot(d, lines, y-label: none, size: (12, 4.5), legend: "inner-south-east", ref: none, x-max: auto, y-min: auto, y-max: auto, x-label: [$t$ in s], ..args) = plot.plot(
  size: size,
  x-label: x-label,
  y-label: y-label,
  x-min: 0, x-max: x-max,
  y-min: y-min, y-max: y-max,
  legend: legend,
  x-format: v => str(calc.round(v, digits: 2)),
  y-format: v => str(calc.round(v, digits: 3)),
  ..args,
  {
    if ref != none {
      plot.add(series(d, "t", ref), style: (stroke: (paint: gray, thickness: 0.5pt)))
    }
    for (i, l) in lines.enumerate() {
      let dash = if l.len() > 2 { l.at(2) } else { none }
      let color = palette.at(if l.len() > 3 { l.at(3) } else { i })
      plot.add(series(d, "t", l.at(0)), style: (stroke: (paint: color, thickness: 0.9pt, dash: dash)), label: l.at(1))
    }
  },
)

/// A labeled box for block diagrams.
#let block-node(pos, body, name, w: 1.6, h: 0.8) = {
  import cetz.draw: *
  rect((pos.at(0) - w / 2, pos.at(1) - h / 2), (pos.at(0) + w / 2, pos.at(1) + h / 2), name: name, radius: 0.06)
  content(name, body)
}

/// A summing junction.
#let sum-node(pos, name) = {
  import cetz.draw: *
  circle(pos, radius: 0.18, name: name)
}
