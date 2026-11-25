// latex/pictures/mt_energy/mt_energy_3combined_linear.tikz: three stacked
// log-log plots (A-E, v3-E, A-v3); only the first one has a legend.
#import "common.typ": mt-figure

#let mt-energy-3combined() = stack(
  spacing: 0.8em,
  mt-figure("A", "P", log: true, size: (12, 5)),
  mt-figure("v3", "P", log: true, size: (12, 5), show-legend: false),
  mt-figure("A", "v3", log: true, size: (12, 5), show-legend: false),
)
