// Shared helpers, replacing latex/packages/commands.sty and the helper
// commands from latex/tex/header/*.tex.

#import "@preview/unify:0.8.1": qty, unit, num
#import "@preview/in-dexter:0.7.2": index, make-index

#let darkgreen = rgb(0%, 50%, 0%)
#let darkred = rgb(50%, 0%, 0%)
#let darkblue = rgb(0%, 0%, 50%)
#let darkgray = rgb(20%, 20%, 20%)

/// Page header text on the left (LaTeX: \lhead). Set wherever the LaTeX
/// source called \lhead{...}.
#let lhead = state("lhead", none)
#let set-lhead(body) = lhead.update(body)

/// Unit in running text (LaTeX: \withunit{...}), shown in dark gray.
#let withunit(u) = text(fill: darkgray, unit(u, per: "fraction"))

/// Quantity in running text (LaTeX: \withunitx{value}{unit}).
#let withunitx(value, u) = text(fill: darkgray, qty(value, u, per: "fraction"))

/// Narrow circled number (LaTeX: \ncircled{n}).
#let ncircled(n) = box(baseline: 0.25em, circle(
  stroke: 0.5pt,
  inset: 0.5pt,
  align(center + horizon, text(size: 0.85em, n)),
))

/// Add text to the index and print it (LaTeX: \indexed{...}).
#let indexed(body) = [#body#index(body)]

/// Small caps name that is also indexed (LaTeX: \name{...}).
#let name(body) = [#smallcaps(body)#index(body)]

/// Long caption in the document, short caption in the list of figures
/// (LaTeX: \caption[short]{long}).
#let in-outline = state("in-outline", false)
#let flex-caption(short, long) = context if in-outline.get() { short } else { long }

/// The thin rule the LaTeX figures put between figure body and caption.
#let caption-rule = line(length: 35em, stroke: 0.5pt)

/// To-do note (LaTeX: todonotes, preset to inline, tiny). The note is also
/// collected for `list-of-todos`.
#let todo(body, inline: true, size: 0.7em, color: orange.lighten(60%)) = {
  let note = text(size: size, body)
  [#metadata(body)<todo>]
  if inline {
    block(width: 100%, fill: color, stroke: color.darken(30%) + 0.5pt, inset: 4pt, radius: 2pt, note)
  } else {
    // shortcut: todonotes would put this in the margin; shown inline as a box instead.
    box(fill: color, stroke: color.darken(30%) + 0.5pt, inset: (x: 2pt), outset: (y: 2pt), radius: 2pt, note)
  }
}

/// List of to-dos (LaTeX: \listoftodos); renders nothing when there are none.
#let list-of-todos() = context {
  let items = query(<todo>)
  if items.len() > 0 {
    heading(numbering: none)[Things to do]
    for (i, item) in items.enumerate() {
      let loc = item.location()
      block(link(loc, grid(
        columns: (2em, 1fr, auto),
        gutter: 0.5em,
        [#(i + 1).], item.value, str(counter(page).at(loc).first()),
      )))
    }
  }
}
