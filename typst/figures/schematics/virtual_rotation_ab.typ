// latex/pictures/schematics/virtual_rotation_ab.tikz (not used in the document)
#import "../common.typ": *

#let virtual-rotation-ab() = cetz.canvas({
  import cetz.draw: *
  let p = projection(
    (-calc.sin(-60deg), calc.cos(60deg)),
    (calc.sin(60deg), -calc.cos(60deg)),
    (0, 1),
  )
  let radius = 3
  let rotor = 1.5
  let add(a, b) = a.zip(b).map(((u, v)) => u + v)

  // Coordinate axes at `o`: X, Y, Z directions, colors faded with `light`.
  let axes(o, xd, yd, light: false) = {
    let col(c) = if light { c.lighten(75%) } else { c }
    let st(c) = (stroke: col(c) + (if light { 0.4pt } else { 1.2pt }), mark: (end: "straight", stroke: col(c)))
    for (d, c, l) in ((xd, red, $X$), (yd, green, $Y$), ((0, 0, 1), blue, $Z$)) {
      line(p(..o), p(..add(o, d)), ..st(c))
      content(p(..add(o, d.map(x => 1.2 * x))), l)
    }
  }

  // Semicircle in the xz plane, behind the quadrocopter.
  let half = range(0, 181, step: 5).map(a => p(radius * calc.cos(a * 1deg), 0, radius * calc.sin(a * 1deg)))
  line(..half, p(-radius, 0, 0), stroke: 1.2pt, fill: red.lighten(90%), mark: (start: "straight", end: "straight"))
  line(p(0, 0, 0), p(0, 0, radius), stroke: (dash: "dotted", thickness: 0.4pt))
  content(p(0, 0, radius), $theta$, anchor: "south", padding: 0.1)

  // Quadrocopter arms
  content(p(0, 0, 0), $R$, anchor: "north", padding: 0.15)
  let thin = (stroke: 0.2pt)
  line(p(0, 0, 0), p(radius, 0, 0), ..thin)
  content(p(radius, 0, 0), $A$, anchor: "south-east", padding: 0.1)
  line(p(0, 0, 0), p(-radius, 0, 0), ..thin)
  content(p(-radius, 0, 0), $B$, anchor: "north-west", padding: 0.1)
  line(p(0, 0, 0), p(0, -radius, 0), ..thin)
  content(p(0, -radius, 0), $C$, anchor: "north-east", padding: 0.1)
  line(p(0, 0, 0), p(0, radius, 0), ..thin)
  content(p(0, radius, 0), $D$, anchor: "south-west", padding: 0.1)

  // Rotor circles (TikZ circles follow the x/y vectors, i.e. the xy plane)
  for o in ((radius, 0, 0), (-radius, 0, 0), (0, -radius, 0), (0, radius, 0)) {
    line(..circle3d(p, (u, v) => add(o, (u, v, 0)), rotor), close: true,
      stroke: (paint: gray, dash: "dotted", thickness: 0.2pt))
  }

  axes((radius, 0, 0), (0, 1, 0), (1, 0, 0))                  // A
  axes((-radius, 0, 0), (0, -1, 0), (-1, 0, 0))               // B
  axes((0, -radius, 0), (1, 0, 0), (0, -1, 0), light: true)   // C
  axes((0, radius, 0), (-1, 0, 0), (0, 1, 0), light: true)    // D
  axes((0, 0, 0), (0, 1, 0), (1, 0, 0))                       // reference
})
