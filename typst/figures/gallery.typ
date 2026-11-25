// Renders every figure on its own page, including the ones the document does
// not use. Build (from typst/): typst compile --root . figures/gallery.typ
#set page(width: auto, height: auto, margin: 1cm)
#set text(font: "New Computer Modern", size: 11pt)
#let fig(name, body) = page[#text(gray, raw(name)) #v(0.5em) #body]

#import "streamtube_schematic.typ": streamtube-schematic
#import "mass_flow_force.typ": mass-flow-force
#import "schematics/tangential.typ": tangential

#fig("streamtube_schematic", streamtube-schematic())
#fig("mass_flow_force", mass-flow-force())
#fig("schematics/tangential", tangential())
#import "propeller_thrust.typ": propeller-thrust
#fig("propeller_thrust", block(width: 14.6cm, propeller-thrust()))
#import "mt_energy/mt_energy_A_E_linear.typ": mt-energy-A-E-linear
#import "mt_energy/mt_energy_A_E_log.typ": mt-energy-A-E-log
#import "mt_energy/mt_energy_A_v3_linear.typ": mt-energy-A-v3-linear
#import "mt_energy/mt_energy_A_v3_log.typ": mt-energy-A-v3-log
#import "mt_energy/mt_energy_v3_E_linear.typ": mt-energy-v3-E-linear
#import "mt_energy/mt_energy_v3_E_log.typ": mt-energy-v3-E-log
#import "mt_energy/mt_energy_A_v3_E_linear.typ": mt-energy-A-v3-E-linear
#import "mt_energy/mt_energy_3combined_linear.typ": mt-energy-3combined
#fig("mt_energy/mt_energy_3combined_linear", mt-energy-3combined())
#fig("mt_energy/mt_energy_A_E_linear", mt-energy-A-E-linear())
#fig("mt_energy/mt_energy_A_E_log", mt-energy-A-E-log())
#fig("mt_energy/mt_energy_A_v3_linear", mt-energy-A-v3-linear())
#fig("mt_energy/mt_energy_A_v3_log", mt-energy-A-v3-log())
#fig("mt_energy/mt_energy_v3_E_linear", mt-energy-v3-E-linear())
#fig("mt_energy/mt_energy_v3_E_log", mt-energy-v3-E-log())
#fig("mt_energy/mt_energy_A_v3_E_linear", mt-energy-A-v3-E-linear())
#import "schematics/coordinate_system.typ": coordinate-system
#import "schematics/virtual_rotation_ab.typ": virtual-rotation-ab
#fig("schematics/coordinate_system", coordinate-system())
#fig("schematics/virtual_rotation_ab", virtual-rotation-ab())
#import "climb_streamtube.typ": climb-streamtube
#fig("climb_streamtube", climb-streamtube())
#import "sensing_estimator.typ": sensing-estimator
#fig("sensing_estimator", sensing-estimator())
#import "control/diagrams.typ": axis-chain, feedback-loop
#fig("control/diagrams: axis-chain", axis-chain())
#fig("control/diagrams: feedback-loop", feedback-loop())
#import "control/poles.typ": pole-gallery, closed-loop-poles
#fig("control/poles: pole-gallery", pole-gallery())
#fig("control/poles: closed-loop-poles", closed-loop-poles())
#import "control/z_plane.typ": z-plane
#fig("control/z_plane", z-plane())
#import "control/responses.typ": open-loop, placement-wn, placement-zeta, placement-motor, lqr-rho, integral-action, windup, rate-tuning, twiddle-cost, cascade
#fig("control/responses: open-loop", open-loop())
#fig("control/responses: placement-wn", placement-wn())
#fig("control/responses: placement-zeta", placement-zeta())
#fig("control/responses: placement-motor", placement-motor())
#fig("control/responses: lqr-rho", lqr-rho())
#fig("control/responses: integral-action", integral-action())
#fig("control/responses: windup", windup())
#fig("control/responses: rate-tuning", rate-tuning())
#fig("control/responses: twiddle-cost", twiddle-cost())
#fig("control/responses: cascade", cascade())
#import "control/identification.typ": ident-setup, ident-trajectories, lag-residual, bringup-step
#fig("control/identification: ident-setup", ident-setup())
#fig("control/identification: ident-trajectories", ident-trajectories())
#fig("control/identification: lag-residual", lag-residual())
#fig("control/identification: bringup-step", bringup-step())
