// Entry point (latex/breadbird.tex). Build: typst compile main.typ
#import "template.typ": thesis, frontmatter, mainmatter, backmatter
#import "commands.typ": set-lhead, list-of-todos, make-index
#import "title/title.typ": title-page
#import "title/abstract.typ": abstract-page

#let title = [Autonomous flight control of a quadrotor UAV using the STM32F3-Discovery microcontroller evaluation board]
#let author = [Markus #smallcaps[Mayer]]

#show: thesis.with(
  title: "Autonomous flight control of a quadrotor UAV using the STM32F3-Discovery microcontroller evaluation board",
  author: "Markus Mayer",
)

#show: frontmatter
#title-page(title, author, "https://github.com/sunsided")
#include "title/quote.typ"
#abstract-page(title, author)
// #include "title/acknowledgements.typ"

#set-lhead[Contents]
#outline(title: [Contents])
#pagebreak(weak: true)
#set-lhead[List of Figures]
#outline(title: [List of Figures], target: figure.where(kind: image))
#pagebreak(weak: true)
#set-lhead[List of Tables]
#outline(title: [List of Tables], target: figure.where(kind: table))


#show: mainmatter
#set heading(numbering: "1.1")
#include "chapters/theory.typ"
#include "chapters/attitude.typ"
#include "chapters/control.typ"
#include "chapters/identification.typ"

#show: backmatter

#set-lhead[Index]
= Index
#columns(2, make-index(use-bang-grouping: true, use-page-counter: true))

#set-lhead[Bibliography]
#bibliography("bibliography.bib", title: [Bibliography], style: "ieee", full: true)

#pagebreak(weak: true)
#set-lhead[Things to do]
#list-of-todos()
