# Typst sources

Typst port of the original LaTeX sources, which were removed afterwards. Comments
naming a `latex/...` file point to the file a piece was ported from; see the git
history for those.

With [Task](https://taskfile.dev) from the repository root: `task build`, `task dev` (watch),
`task build:figures`, `task clean`. Or directly from this directory:

```sh
typst compile main.typ                         # -> main.pdf
typst compile --root . figures/gallery.typ     # all 33 figures, including unused ones
```

Packages (`cetz`, `cetz-plot`, `unify`, `in-dexter`, `equate`) are fetched from
Typst Universe on first compile.

- `main.typ`: document entry point (front, main and back matter)
- `template.typ`: page layout, headings, numbering (replaces `thesis.cls` and `tex/header/`)
- `commands.typ`: helpers such as `withunit`, `ncircled`, `name`, `todo`
- `chapters/`: content
- `figures/`: CeTZ redraws of the TikZ/pgfplots pictures
- `data/`: CSV data for the plots
  - `data/mt_energy/`: power plots, written by `../matlab/momentum_theory_energy.m`
  - `data/control/`: control chapter, written by `../matlab/attitude_control.m`; `bringup_*.csv` for the identification chapter, written by `cargo run --release --example bringup` in `../control/`

  Run the Octave scripts from the repository root, e.g. `octave --no-gui matlab/attitude_control.m`.
