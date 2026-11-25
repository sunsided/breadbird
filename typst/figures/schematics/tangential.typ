// latex/pictures/schematics/tangential.tikz (not used in the document)
#import "../common.typ": *

#let tangential() = cetz.canvas({
  import cetz.draw: *
  let r = (1.4142, 1.4142)

  arc((2, 0), start: 0deg, stop: 90deg, radius: 2, stroke: (paint: gray, dash: "dotted", thickness: 0.4pt))
  line((0, 0), r, mark: bar-to)
  content((1, 1), [r], anchor: "north-west", padding: 0.1)
  line(r, (r.at(0) + 0.7071, r.at(1) - 0.7071), stroke: purple, mark: (end: "straight", stroke: purple))
  content((r.at(0) + 0.7071, r.at(1) - 0.7071), text(purple, $arrow(a)_T$), anchor: "south-west", padding: 0.1)

  line((-0.2, 0), (0.2, 0), stroke: gray + 0.1pt)
  line((0, -0.2), (0, 0.2), stroke: gray + 0.1pt)

  arc((0.7071, 0), start: 0deg, stop: 45deg, radius: 0.7071, stroke: teal + 0.4pt, mark: (start: "straight", stroke: teal))
  content((0.7071, 0), text(teal, $alpha$), anchor: "north", padding: 0.1)
})
