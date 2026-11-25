// latex/pictures/propeller_thrust.tikz
#import "common.typ": *

/// tikz-3dplot \tdplotsetmaincoords{theta}{phi}: rotate about z by phi, then
/// about x by theta, and project onto the screen.
#let projector(theta, phi) = (x, y, z) => (
  x * calc.cos(phi) - y * calc.sin(phi),
  calc.cos(theta) * (x * calc.sin(phi) + y * calc.cos(phi)) + z * calc.sin(theta),
)

/// Points of a circle of radius r around the z axis at height z.
#let ring(p, r, z, n: 72) = range(n).map(i => {
  let t = i * 360deg / n
  p(r * calc.cos(t), r * calc.sin(t), z)
})

/// The two-bladed rotor (TikZ pic `rotor` from latex/tex/header/graphics.tex),
/// drawn in the xy plane.
#let rotor(p, color: black) = {
  import cetz.draw: *
  let (offset, radius, width, len, belly) = (0.125, 0.16, 0.5, 3, 2)
  let blade = (stroke: color, fill: color.transparentize(70%), tension: 0.4, close: false)
  for s in (1, -1) {
    let pts = (
      (s * offset, offset), (s * (offset + belly), width), (s * len, 0),
      (s * (offset + belly), -width), (s * offset, -offset),
    ).map(((x, y)) => p(x, y, 0))
    catmull(..pts, ..blade)
  }
  line(..ring(p, radius, 0, n: 24), close: true, fill: color.transparentize(20%))
}

#let propeller-thrust() = grid(
  columns: (1fr, 1fr),
  align: bottom + center,
  // left: stream tube around the rotor disk
  cetz.canvas({
    import cetz.draw: *
    let p = projector(75deg, 145deg)

    line(..ring(p, 3, 0), close: true, stroke: dotted, fill: black.transparentize(90%))
    rotor(p)
    line(p(0, 0, 2), p(0, 0, -2), stroke: dotted, mark: tail-to + (stroke: (dash: none)))

    let levels = (
      (2, calc.tanh(2) + 3),
      (1, calc.tanh(2) + 3),
      (0.35, calc.tanh(1) + 3),
      (-0.35, calc.tanh(-1) + 3),
      (-1, calc.tanh(-2) + 3),
      (-2, calc.tanh(-2) + 3),
    )
    for (z, r) in levels {
      line(..ring(p, r, z), close: true, stroke: black.transparentize(70%))
    }
    for t in range(-180, 180, step: 20) {
      let (c, s) = (calc.cos(t * 1deg), calc.sin(t * 1deg))
      hobby(..levels.map(((z, r)) => p(c * r, s * r, z)), stroke: gray.transparentize(80%))
    }
  }),
  // right: pressure and velocity along the stream tube
  cetz.canvas({
    import cetz.draw: *
    line((0, 2), (0, 0), (4, 0), mark: both)
    marker((0, 2), [0], anchor: "south")
    content((0, 0), text(cyan, $p_1$), anchor: "east", padding: 0.1)
    content((2, 0), text(red, $v_i$), anchor: "north-east", padding: 0.1)
    content((4, 0), text(cyan, $Delta p$), anchor: "south-west", padding: 0.1)
    content((4, 0), text(red, $Delta v$), anchor: "north-west", padding: 0.1)
    line((0, 0), (0, -2), mark: to)
    marker((0, -2), [3], anchor: "north")

    catmull((2, 2), (1.5, 0.5), (0, 0), stroke: cyan)
    catmull((2, -2), (2.5, -0.5), (4, 0), stroke: cyan)
    catmull((3.75, -2), (3, -0.5), (1, 0.5), (0.25, 2), stroke: (paint: red, dash: "dashed"))

    line((2, -2), (2, 2), stroke: (paint: cyan, dash: "dotted"))
    content((2, 2), text(cyan, $p_infinity$), anchor: "south", padding: 0.1)
    line((0.25, -2), (0.25, 2), stroke: (paint: red, dash: "dotted"))
    content((0.25, 2), text(red, $v_0$), anchor: "south-west", padding: 0.1)
    line((3.75, -2), (3.75, 2), stroke: (paint: red, dash: "dotted"))
    content((3.75, 2), text(red, $v_3$), anchor: "south", padding: 0.1)
  }),
)
