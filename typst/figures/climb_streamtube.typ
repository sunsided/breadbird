// Stream tube of a rotor in axial climb (new; drawn in the style of
// streamtube_schematic.typ).
#import "common.typ": *

#let climb-streamtube() = cetz.canvas({
  import cetz.draw: *

  // One half of the stream tube; `s` mirrors it, `dy` shifts it. The tube
  // contracts less than in hover, since the climb velocity adds to all
  // velocities in the tube.
  let streamtube(dy, s) = {
    let p(x, y) = (x, dy + s * y)
    line(p(-5, 0.3), p(-4, 0.3))
    bezier(p(-4, 0.3), p(4, -0.3), p(0, 0.3), p(0, -0.3))
    line(p(4, -0.3), p(5, -0.3))
  }
  streamtube(1.5, 1)
  streamtube(-1.5, -1)

  // actuator disk with hub
  line((0, 1.5), (0, -1.5), stroke: 1pt)
  line((-0.2, 0), (0.1, 0.15), (0.1, -0.15), close: true, fill: white)
  line((-4.75, 0), (4.75, 0), stroke: dotted)

  // reference planes
  line((-5, 1.8), (-5, -1.8), stroke: dotted)
  line((5, 1.2), (5, -1.2), stroke: dotted)

  let red-arrow = (stroke: red + 1pt, mark: (end: "straight", fill: red))
  line((-5.25, -0.75), (-4.75, -0.75), ..red-arrow)
  content((-5, -0.75), $v_c$, anchor: "north", padding: 0.05, frame: "rect", fill: white, stroke: none)
  line((-0.25, -0.75), (0.25, -0.75), ..red-arrow)
  content((0, -0.75), $v_c + u$, anchor: "north", padding: 0.05, frame: "rect", fill: white, stroke: none)
  line((4.75, -0.75), (5.25, -0.75), ..red-arrow)
  content((5, -0.75), $v_c + 2u$, anchor: "north", padding: 0.05, frame: "rect", fill: white, stroke: none)

  content((0, 1.7), [rotor disk], anchor: "south")
  content((-5, 1.9), [far upstream], anchor: "south")
  content((5, 1.3), [far wake], anchor: "south")

  // climb direction of the rotor itself (relative to the still air)
  line((0.5, 0.75), (-0.5, 0.75), stroke: blue + 1pt, mark: (end: "straight", fill: blue))
  content((-0.5, 0.75), text(blue, $v_c$), anchor: "east", padding: 0.1)
})
