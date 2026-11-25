// Abstract page (latex/tex/title/abstract.tex).
#import "../commands.typ": todo
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
  todo[This technical report describes something]
  v(3fr)
})
