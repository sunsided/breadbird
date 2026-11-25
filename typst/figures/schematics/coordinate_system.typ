// latex/pictures/schematics/coordinate_system.tikz (not used in the document)
#import "../common.typ": *

#let coordinate-system() = cetz.canvas({
  import cetz.draw: *
  let p = projection(
    (calc.sin(-60deg), -calc.cos(60deg)),
    (calc.sin(60deg), -calc.cos(60deg)),
    (0, 1),
    scale: 1.5,
  )
  let r = 0.25

  for (v, c, l) in (((1, 0, 0), red, $X$), ((0, 1, 0), green, $Y$), ((0, 0, 1), blue, $Z$)) {
    line(p(0, 0, 0), p(..v), stroke: c + 0.8pt, mark: (end: "straight", stroke: c))
    content(p(..v.map(x => 1.3 * x)), l)
  }

  // Rotation markers: circle in a plane with an arrow at TikZ path position
  // `at` (fraction of the circle) and a label.
  let rotation(pt, c, label, anchor, at: 0, reverse: true) = {
    line(..circle3d(p, pt, r), close: true, stroke: c + 0.4pt)
    let t = at * 360deg
    let start = p(..pt(r * calc.cos(t), r * calc.sin(t)))
    let d = if reverse { -0.08 } else { 0.08 }
    let tip = p(..pt(r * calc.cos(t) - d * calc.sin(t), r * calc.sin(t) + d * calc.cos(t)))
    line(start, tip, stroke: c + 0.4pt, mark: (end: "straight", stroke: c, scale: 0.8))
    content(start, label, anchor: anchor, padding: 0.1)
  }
  rotation((u, v) => (0.7, v, u), red, $Phi$, "south")             // zy plane at x=0.7
  rotation((u, v) => (u, 0.7, v), green, $Theta$, "east")          // xz plane at y=0.7
  rotation((u, v) => (u, v, 0.7), blue, $Psi$, "west", at: 0.4, reverse: false) // xy plane at z=0.7
})
