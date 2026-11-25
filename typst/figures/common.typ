// Shared CeTZ helpers for the figures (TikZ arrow styles etc.).
#import "@preview/cetz:0.5.2"

/// TikZ `->` (thin open arrow head).
#let to = (end: "straight", stroke: (dash: none))
/// TikZ `<->`.
#let both = (start: "straight", end: "straight", stroke: (dash: none))
/// TikZ `|<->|` (dimension line).
#let dim = (start: ("|", "straight"), end: ("|", "straight"))
/// TikZ `|-|`.
#let bars = (start: "|", end: "|")
/// TikZ `|->`.
#let bar-to = (start: "|", end: "straight")
/// TikZ `>->`.
#let tail-to = (start: "straight", end: "straight")

#let dotted = (dash: "dotted")
#let dashed = (dash: "dashed")

/// TikZ `\node[draw,circle,inner sep=0.5]` area marker.
#let marker(pos, n, name: none, anchor: "center") = cetz.draw.content(
  pos, n, name: name, anchor: anchor, frame: "circle", stroke: 0.4pt, padding: 0.05,
)

// TikZ/xcolor named colors that Typst lacks or defines differently.
#let cyan = rgb(0%, 100%, 100%)
#let magenta = rgb(100%, 0%, 100%)
#let purple = rgb(75%, 0%, 25%)
#let teal = rgb(0%, 50%, 50%)

/// Projection for TikZ pictures with explicit unit vectors
/// (`x = {(..)}, y = {(..)}, z = {(..)}`): returns (x, y, z) => screen point.
#let projection(xv, yv, zv, scale: 1) = (x, y, z) => (
  scale * (x * xv.at(0) + y * yv.at(0) + z * zv.at(0)),
  scale * (x * xv.at(1) + y * yv.at(1) + z * zv.at(1)),
)

/// Closed polygon approximating a circle; `pt(u, v)` maps the circle's canvas
/// plane (TikZ `canvas is .. plane`) to 3D, `p` projects to the screen.
#let circle3d(p, pt, r, n: 72) = range(n).map(i => {
  let t = i * 360deg / n
  p(..pt(r * calc.cos(t), r * calc.sin(t)))
})
