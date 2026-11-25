// Block diagram of the airflow estimator (new): sensors -> estimated
// quantities -> rotor state.
#import "common.typ": *

#let sensing-estimator() = cetz.canvas({
  import cetz.draw: *

  let node(pos, body, name, w: 3.4) = {
    rect((pos.at(0) - w / 2, pos.at(1) - 0.4), (pos.at(0) + w / 2, pos.at(1) + 0.4), name: name, radius: 0.08)
    content(name, text(size: 0.9em, body))
  }

  // sensors
  node((0, 3), [Hall sensor], "hall")
  node((0, 1.8), [current, voltage], "current")
  node((0, 0.6), [pressure ports], "ports")
  node((0, -0.6), [barometer, temp.], "baro")

  // measured quantities
  node((4.4, 3), $n$, "n", w: 1.6)
  node((4.4, 1.8), $P = K_T (I - I_0) omega_R$, "p", w: 3.2)
  node((4.4, 0.6), $T = A_r Delta p$, "t", w: 3.2)
  node((4.4, -0.6), $rho$, "rho", w: 1.6)

  // estimator
  rect((7.0, -0.9), (11.0, 3.3), name: "est", radius: 0.08, fill: luma(96%))
  content((9.0, 2.85), text(size: 0.9em, weight: "bold")[estimator])
  // Not document equations: no numbers.
  content((9.0, 1.0), text(size: 0.9em, {
    set math.equation(numbering: none)
    $
    C_T = T \/ (rho n^2 D^4) \
    C_P = P \/ (rho n^3 D^5) \
    zeta = sqrt(T^3 \/ (2 rho A_r)) \/ P \
    v_c + u approx zeta_0 P \/ T
    $
  }))

  for (a, b) in (("hall", "n"), ("current", "p"), ("ports", "t"), ("baro", "rho")) {
    line(a + ".east", b + ".west", mark: to)
  }
  for b in ("n", "p", "t", "rho") {
    line(b + ".east", ((7.0, 0), "|-", b + ".east"), mark: to)
  }
})
