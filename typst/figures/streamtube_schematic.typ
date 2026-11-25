// latex/pictures/streamtube_schematic.tikz
#import "common.typ": *

#let streamtube-schematic() = cetz.canvas({
  import cetz.draw: *

  // One half of the stream tube; `s` mirrors it (TikZ yscale=-1), `dy` shifts it.
  let streamtube(dy, s) = {
    let p(x, y) = (x, dy + s * y)
    line(p(-5, 0.5), p(-4, 0.5))
    bezier(p(-4, 0.5), p(4, -0.5), p(0, 0.5), p(0, -0.5))
    line(p(4, -0.5), p(5, -0.5))
  }
  streamtube(1.5, 1)    // upper part of the stream tube
  streamtube(-1.5, -1)  // lower part of the stream tube

  // Propeller plane (actuator disk)
  line((0, 1.5), (0, -1.5), stroke: 1pt)
  content((0, 2.4), [actuator disk], anchor: "south", name: "actuator_disk_label")
  line((0, 2.4), (0, 1.7), mark: to)

  // propeller hub
  line((-0.2, 0), (0.1, 0.15), (0.1, -0.15), close: true, fill: white)

  // upstream and downstream event horizon
  line((-5, 2), (-5, -2), stroke: dotted)
  line((5, 1), (5, -1), stroke: dotted)

  // flow arrow
  line((-4.75, 0), (4.75, 0), stroke: dotted)

  // radii
  line((-5.25, 2), (-5.25, 0), mark: dim)
  content((-5.25, 1), $r_u$, anchor: "east", padding: 0.1)
  line((5.25, 1), (5.25, 0), mark: dim)
  content((5.25, 0.5), $r_d$, anchor: "west", padding: 0.1)
  line((-0.75, 1.5), (-0.75, 0), mark: dim)
  content((-0.75, 0.75), $r_r$, anchor: "east", padding: 0.1)

  // area markers
  marker((-5, -2.5), [0], name: "area_0")
  marker((-0.25, -2.5), [1], name: "area_1")
  marker((0.25, -2.5), [2], name: "area_2")
  marker((5, -2.5), [3], name: "area_3")

  // inflow and outflow labels
  line((-4.75, -2.5), (-0.5, -2.5), mark: bars)
  content((-2.6, -2.5), [inflow], anchor: "north", padding: 0.1)
  line((0.5, -2.5), (4.75, -2.5), mark: bars)
  content((2.6, -2.5), [outflow], anchor: "north", padding: 0.1)

  let red-arrow = (stroke: red + 1pt, mark: (end: "straight", fill: red))

  // induced pressure vector
  line((-0.25, 0.75), (0.25, 0.75), stroke: cyan + 1pt, mark: (start: ("|", "straight"), end: ("|", "straight"), scale: 0.6))
  content((0.25, 0.75), $Delta p = p_2 - p_1$, anchor: "west", padding: 0.1)

  // induced velocity vector
  line((-0.25, -0.75), (0.25, -0.75), ..red-arrow)
  content((-0.25, -0.75), $arrow(v)_i = arrow(v)_1$, anchor: "east", padding: 0.1)
  content((-0.25, -0.75), text(cyan, $p_1$), anchor: "north-east", padding: (top: 0.35, right: 0.1))
  content((0.25, -0.75), $arrow(v)_2 = arrow(v)_i$, anchor: "west", padding: 0.1)
  content((0.25, -0.75), text(cyan, $p_2$), anchor: "north-west", padding: (top: 0.35, left: 0.1))

  // upstream velocity vector
  line((-5.25, -0.75), (-4.75, -0.75), ..red-arrow)
  content((-5.25, -0.75), $arrow(v)_0$, anchor: "east", padding: 0.1)
  content((-5.25, -0.75), text(cyan, $p_infinity = p_0$), anchor: "north-east", padding: (top: 0.35, right: 0.1))

  // downstream velocity vector
  line((4.75, -0.75), (5.25, -0.75), ..red-arrow)
  content((5.25, -0.75), $arrow(v)_3 > arrow(v)_0$, anchor: "west", padding: 0.1)
  content((5.25, -0.75), text(cyan, $p_3 = p_infinity$), anchor: "north-west", padding: (top: 0.35, left: 0.1))

  // inflow stream lines
  let sl = (stroke: (paint: gray, dash: "dotted"), mark: (end: "straight", stroke: (paint: gray, dash: none)))
  bezier((-4.5, 1.5), (-0.5, 1), (-2.5, 1.5), (-1, 1), ..sl)
  bezier((-4.5, 0.75), (-0.5, 0.5), (-3, 0.75), (-2, 0.5), ..sl)
  bezier((-4.5, -1.5), (-0.875, -1), (-2.5, -1.5), (-1.625, -1), ..sl)
  bezier((-4.5, -0.75), (-0.5, -0.5), (-3, -0.75), (-2, -0.5), ..sl)
})
