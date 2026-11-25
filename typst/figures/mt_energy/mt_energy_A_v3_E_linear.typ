// latex/pictures/mt_energy/mt_energy_A_v3_E_linear.tikz (not used in the
// document). cetz-plot has no 3D axes, so the pgfplots `addplot3` is redrawn as
// a projected axis box.
#import "common.typ": cetz, forces, load, labels, styles
#import "@preview/unify:0.8.1": qty
#import "../propeller_thrust.typ": projector

#let mt-energy-A-v3-E-linear() = cetz.canvas({
  import cetz.draw: *
  let data = forces.map(load)
  // Axis ranges over all series, per column.
  let range-of(c) = {
    let vs = data.map(rows => rows.map(r => r.at(c))).flatten()
    (calc.min(..vs), calc.max(..vs))
  }
  let (x0, x1) = range-of("A")
  let (y0, y1) = range-of("v3")
  let (z0, z1) = range-of("E")
  let size = 6
  let proj = projector(60deg, 25deg)
  // Data coordinates -> screen.
  let p(x, y, z) = proj(
    size * (x - x0) / (x1 - x0),
    size * (y - y0) / (y1 - y0),
    size * (z - z0) / (z1 - z0),
  )

  // Far walls (seen from the front corner x0/y0): floor, x1 and y1.
  let wall = (stroke: 0.5pt, fill: luma(97%))
  line(p(x0, y0, z0), p(x1, y0, z0), p(x1, y1, z0), p(x0, y1, z0), close: true, ..wall)
  line(p(x1, y0, z0), p(x1, y1, z0), p(x1, y1, z1), p(x1, y0, z1), close: true, ..wall)
  line(p(x0, y1, z0), p(x1, y1, z0), p(x1, y1, z1), p(x0, y1, z1), close: true, ..wall)

  // Ticks: 5 per axis, labelled on the front floor edges and the leftmost edge.
  let fmt(v) = text(8pt, str(calc.round(v, digits: 3)))
  for i in range(5) {
    let t = i / 4
    let x = x0 + t * (x1 - x0)
    let y = y0 + t * (y1 - y0)
    let z = z0 + t * (z1 - z0)
    line(p(x, y0, z0), p(x, y1, z0), stroke: luma(85%) + 0.3pt)
    line(p(x0, y, z0), p(x1, y, z0), stroke: luma(85%) + 0.3pt)
    line(p(x0, y1, z), p(x1, y1, z), p(x1, y0, z), stroke: luma(85%) + 0.3pt)
    line(p(x, y1, z0), p(x, y1, z1), stroke: luma(85%) + 0.3pt)
    line(p(x1, y, z0), p(x1, y, z1), stroke: luma(85%) + 0.3pt)
    content(p(x, y0, z0), fmt(x), anchor: "north-west", padding: 0.08)
    content(p(x0, y, z0), fmt(y), anchor: "north-east", padding: 0.08)
    if i > 0 { content(p(x0, y1, z), fmt(z), anchor: "east", padding: 0.12) }
  }
  content(p((x0 + x1) / 2, y0, z0), labels.A, anchor: "north-west", padding: 0.5)
  content(p(x0, (y0 + y1) / 2, z0), labels.v3, anchor: "north-east", padding: 0.5)
  content(p(x0, y1, (z0 + z1) / 2), labels.E, anchor: "south", padding: 1.0, angle: 90deg)

  for (i, rows) in data.enumerate() {
    line(..rows.map(r => p(r.A, r.v3, r.E)), stroke: styles.at(i).color + 0.8pt)
  }

  // Legend (pgfplots: legend pos=north east).
  let (lx, ly) = (p(x1, y0, z1).at(0), p(x0, y1, z1).at(1))
  for (i, f) in forces.enumerate() {
    let y = ly - i * 0.5
    line((lx + 0.3, y), (lx + 0.9, y), stroke: styles.at(i).color + 0.8pt)
    content((lx + 1, y), [$F = #qty(f, "N")$], anchor: "west")
  }
})
