# Typst sources

Typst port of the LaTeX sources in [`../latex/`](../latex/).

With [Task](https://taskfile.dev) from the repository root: `task build`, `task dev` (watch),
`task build:figures`, `task clean`. Or directly from this directory:

```sh
typst compile main.typ                         # -> main.pdf
typst compile --root . figures/gallery.typ     # all 15 figures, including unused ones
```

Packages (`cetz`, `cetz-plot`, `unify`, `in-dexter`, `equate`) are fetched from
Typst Universe on first compile.

- `main.typ`: document entry point (front, main and back matter)
- `template.typ`: page layout, headings, numbering (replaces `thesis.cls` and `tex/header/`)
- `commands.typ`: helpers such as `withunit`, `ncircled`, `name`, `todo`
- `chapters/`: content
- `figures/`: CeTZ redraws of the TikZ/pgfplots pictures
- `data/`: CSV data for the power plots, written by `../matlab/momentum_theory_energy.m` (`octave --no-gui matlab/momentum_theory_energy.m` from the repository root)
