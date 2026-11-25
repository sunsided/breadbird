// Dominant closed-loop poles in the z-plane at 50 Hz and 500 Hz sampling.
// Data written by matlab/attitude_control.m.
#import "common.typ": *

#let z-plane-plot(d, fs, size, x-min, x-max, y-min, y-max, legend, step) = plot.plot(
  size: size,
  x-label: [$"Re" z$], y-label: [$"Im" z$],
  x-min: x-min, x-max: x-max, y-min: y-min, y-max: y-max,
  legend: legend,
  x-tick-step: step, y-tick-step: step,
  x-format: v => str(calc.round(v, digits: 2)),
  y-format: v => str(calc.round(v, digits: 2)),
  {
    plot.add(t => (calc.cos(t), calc.sin(t)), domain: (0, 2 * calc.pi), samples: 200,
      style: (stroke: (paint: gray, thickness: 0.5pt)))
    for (i, f) in fs.enumerate() {
      let pts = range(d.fs.len()).filter(j => d.fs.at(j) == f)
        .map(j => ((d.re.at(j), d.im.at(j)), (d.re.at(j), -d.im.at(j)))).flatten().chunks(2)
        // cetz-plot does not clip marks: keep the zoomed view to its window
        .filter(((x, y)) => x >= x-min and x <= x-max and y >= y-min and y <= y-max)
      plot.add(pts, style: (stroke: none), mark: if i == 0 { "o" } else { "x" }, mark-size: 0.12,
        mark-style: (stroke: palette.at(i), fill: none), label: [#f Hz])
    }
  },
)

#let z-plane() = {
  let d = load("z_poles")
  cetz.canvas({
    import cetz.draw: *
    z-plane-plot(d, (50, 500), (6, 6), -1.1, 1.1, -1.1, 1.1, "inner-south-west", 0.5)
    group({
      translate((8, 0))
      z-plane-plot(d, (50, 500), (6, 6), 0.55, 1.05, -0.25, 0.25, none, 0.1)
    })
  })
}
