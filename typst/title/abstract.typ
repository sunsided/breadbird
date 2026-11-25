// Abstract page (latex/tex/title/abstract.tex).
#let abstract-page(title, author) = page(header: none, {
  // ToC entry only (LaTeX: \addtotoc{Abstract}); the page shows its own title.
  {
    show heading: none
    heading(level: 1)[Abstract]
  }
  v(1fr)
  align(center, {
    set par(spacing: 0pt)
    text(size: 2em, emph[Abstract])
    v(1em)
    strong(title)
    v(0.6em)
    [by #author]
    v(1em)
  })
  [This report collects the theoretical foundations for the autonomous flight control of a quadrotor UAV built around the STM32F3-Discovery evaluation board.
  So far, it covers the thrust of a propeller: momentum theory for hover, axial climb and descent, the tradeoff between rotor size and power, the efficiency limits of the model, and the relation of thrust and power to rotor speed using measured propeller coefficients, ending with the matching of motor and propeller and with estimating the airflow from motor and pressure measurements.
  The control part has not been written yet.]
  v(3fr)
})
