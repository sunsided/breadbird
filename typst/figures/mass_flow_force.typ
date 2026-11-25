// latex/pictures/mass_flow_force.tikz
#import "common.typ": *

#let mass-flow-force() = cetz.canvas({
  import cetz.draw: *
  let len = 2

  // the disk
  line((0, -1), (0, 1), stroke: 2pt)

  // force arrow
  line((0, 0), (1, 0), stroke: magenta + 1pt, mark: (end: "straight", fill: magenta))
  content((1, 0), $arrow(F) = dot(m) arrow(v)$, anchor: "west", padding: 0.1)

  // disk radius
  line((0.25, 1), (0.25, 0), mark: dim)
  content((0.25, 0.5), $r$, anchor: "west", padding: 0.1)

  // fluid velocity vectors on the disk
  for x in (0.2, 0.4, 0.6, 0.8, 1.0) {
    line((-len, x), (-0.125, x), stroke: dotted, mark: to)
    line((-len, -x), (-0.125, -x), stroke: dotted, mark: to)
  }
  line((-len, 0), (-0.125, 0), stroke: (dash: "dotted", thickness: 0.6pt), mark: to)
  content(((-len - 0.125) / 2, 0), text(red, $arrow(v)$), frame: "rect", fill: white, stroke: none, padding: 0.07)

  // fluid velocity vectors above and below the disk
  for x in (1.2, 1.4) {
    line((-len, x), (len, x), stroke: dotted, mark: to)
    line((-len, -x), (len, -x), stroke: dotted, mark: to)
  }

  // m-dot brace
  cetz.decorations.flat-brace((-len, -1), (-len, 1), amplitude: 0.18, stroke: magenta)
  content((-len - 0.35, 0), $rho (pi r^2) arrow(v) = dot(m)$, anchor: "east", padding: 0.1)
})
