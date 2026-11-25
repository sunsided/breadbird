// Page layout and styling, replacing latex/packages/thesis.cls and
// latex/tex/header/*.tex.

#import "@preview/equate:0.3.3": equate
#import "commands.typ": lhead, in-outline, darkblue, darkred, darkgreen

#let thesis(title: none, author: none, keywords: (), body) = {
  set document(title: title, author: author, keywords: keywords)

  set page(
    paper: "a4",
    margin: (left: 1.5in, right: 1.0in, top: 0.6in + 20pt + 0.25in, bottom: 0.8in),
    header-ascent: 0.25in,
    footer: none,
    header: context {
      let n = counter(page).display()
      // Chapter opening pages only show the page number (LaTeX: plain style).
      let opening = query(heading.where(level: 1)).any(h => h.location().page() == here().page())
      if lhead.get() != none and not opening [#emph(lhead.get()) #h(1fr) #n] else [#h(1fr) #n]
    },
  )

  set text(size: 11pt, lang: "en", region: "US", font: "New Computer Modern")
  // thesis.cls: onehalfspacing, \setstretch{1.3}, parindent 0, parskip 2ex.
  set par(justify: true, leading: 0.8em, spacing: 1.4em, first-line-indent: 0pt)

  // Chapters (level 1) and sections; equations and figures are numbered per
  // chapter/section like \numberwithin{equation}{section} in a book class.
  set heading(numbering: "1.1")
  show heading: set text(font: "New Computer Modern")
  show heading.where(level: 1): it => {
    pagebreak(weak: true)
    counter(math.equation).update(0)
    counter(figure.where(kind: image)).update(0)
    counter(figure.where(kind: table)).update(0)
    v(3em)
    if it.numbering != none {
      text(size: 1.4em, weight: "bold")[Chapter #counter(heading).display("1")]
      v(0.6em)
    }
    text(size: 1.8em, weight: "bold", it.body)
    v(1.5em)
  }
  show heading.where(level: 2): it => {
    counter(math.equation).update(0)
    it
  }

  show: equate.with(breakable: true, number-mode: "line")
  set math.equation(numbering: (..n) => {
    let h = counter(heading).get()
    numbering("(1.1.1)", h.at(0, default: 0), h.at(1, default: 0), ..n.pos())
  })
  set figure(numbering: n => numbering("1.1", counter(heading).get().first(), n))
  show figure.caption: set text(size: 0.9em)
  show figure.caption: it => block(width: 100%, inset: (x: 20pt), align(center, it))

  // cleveref-like reference text.
  show ref: it => {
    let el = it.element
    if el != none and el.func() == heading and el.level == 2 {
      link(el.location(), [Section #numbering(el.numbering, ..counter(heading).at(el.location()))])
    } else if el != none and el.func() == heading and el.level > 2 {
      link(el.location(), [Subsection #numbering(el.numbering, ..counter(heading).at(el.location()))])
    } else { it }
  }
  set math.equation(supplement: [Eq.])

  show link: it => if type(it.dest) == str { text(fill: darkred, it) } else { it }
  show ref: set text(fill: darkblue)
  show cite: set text(fill: darkblue)

  show outline: it => { in-outline.update(true); it; in-outline.update(false) }
  // thesis.cls lists Contents and the lists of figures/tables in the ToC.
  show outline: set heading(outlined: true)
  set outline.entry(fill: repeat[.#h(4pt)])
  show outline.entry.where(level: 1): set block(above: 1.2em)

  set list(spacing: 0.65em)
  set enum(spacing: 0.65em)
  set footnote.entry(separator: line(length: 30%, stroke: 0.5pt))

  body
}

/// Front matter: roman page numbers, unnumbered headings (LaTeX: \frontmatter).
#let frontmatter(body) = {
  set page(numbering: "i")
  counter(page).update(1)
  set heading(numbering: none)
  body
}

/// Main matter: arabic page numbers (LaTeX: \mainmatter).
#let mainmatter(body) = {
  set page(numbering: "1")
  counter(page).update(1)
  body
}

/// Back matter: unnumbered chapters (LaTeX: \backmatter).
#let backmatter(body) = {
  set heading(numbering: none)
  body
}

/// Pages without header (LaTeX: \pagestyle{empty}).
#let empty-page(body) = page(header: none, body)
