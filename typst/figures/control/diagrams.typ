// Block diagrams for the chapter on state-space control.
#import "common.typ": *

/// One attitude axis from the commanded differential thrust u to the angle.
#let axis-chain() = cetz.canvas({
  import cetz.draw: *
  let y = 0
  content((-1.0, y), $u[k]$, name: "u")
  block-node((0.6, y), $z^(-1)$, "delay", w: 1.0)
  block-node((3.4, y), $display(1 / (tau_m s + 1))$, "motor", w: 2.0, h: 1.1)
  block-node((5.9, y), $b$, "gain", w: 0.8)
  block-node((7.7, y), $integral$, "int1", w: 0.9)
  block-node((9.7, y), $integral$, "int2", w: 0.9)
  content((11.2, y), $theta$, name: "theta")

  line("u.east", "delay.west", mark: to)
  line("delay.east", "motor.west", mark: to)
  line("motor.east", "gain.west", mark: to)
  line("gain.east", "int1.west", mark: to)
  line("int1.east", "int2.west", mark: to)
  line("int2.east", "theta.west", mark: to)
  content((1.75, y + 0.35), text(size: 0.8em, $u_"prev"$))
  content((5.0, y + 0.35), text(size: 0.8em, $Delta T$))
  content((6.8, y + 0.35), text(size: 0.8em, $dot(omega)$))
  content((8.7, y + 0.35), text(size: 0.8em, $omega$))
  content((0.6, y - 0.75), text(size: 0.75em)[delay])
  content((3.4, y - 0.9), text(size: 0.75em)[motors])
  content((5.9, y - 0.75), text(size: 0.75em)[$l \/ J$])
})

/// State feedback with integral action and an observer.
#let feedback-loop() = cetz.canvas({
  import cetz.draw: *
  content((-0.6, 0), $r$, name: "r")
  sum-node((0.6, 0), "s1")
  block-node((2.1, 0), $T / (z - 1)$, "int", w: 1.5, h: 0.9)
  block-node((3.9, 0), $k_i$, "ki", w: 0.8)
  sum-node((5.2, 0), "s2")
  block-node((6.6, 0), [sat], "sat", w: 1.0)
  block-node((8.6, 0), [plant], "plant", w: 1.4)
  content((10.5, 0), $y = (theta, omega)$, name: "y")
  block-node((7.5, -2.0), [observer], "obs", w: 1.6)
  block-node((5.2, -2.0), $K$, "k", w: 0.8)

  line("r.east", "s1.west", mark: to)
  line("s1.east", "int.west", mark: to)
  line("int.east", "ki.west", mark: to)
  line("ki.east", "s2.west", mark: to)
  line("s2.east", "sat.west", mark: to)
  line("sat.east", "plant.west", mark: to, name: "u")
  content((7.75, 0.3), $u$)
  line("plant.east", "y.west", mark: to)
  line((9.6, 0), (9.6, -2.0), "obs.east", mark: to)
  line((7.5, 0), "obs.north", mark: to)
  line("obs.west", "k.east", mark: to)
  content((6.2, -1.7), $hat(x)$)
  line("k.north", "s2.south", mark: to)
  content((5.45, -0.5), text(size: 0.8em, $-$))
  line((9.6, 0), (9.6, 1.0), (0.6, 1.0), "s1.north", mark: to)
  content((0.35, 0.45), text(size: 0.8em, $-$))
  content((5.1, 1.25), text(size: 0.8em, $theta$))
})
