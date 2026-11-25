# Autonomous flight control of a quadrotor UAV using the STM32F3-Discovery microcontroller evaluation board

> This is old work in progress. It may act as an okay reference for technical documentation
> in Typst, but YMMV.

See the [typst/](typst/) directory for the main content; each tag (a date such as `2026-11-25`, or `v*`) publishes the compiled PDF as a [GitHub Release](https://github.com/sunsided/breadbird/releases). The report started out in LaTeX; the original sources are in the git history before their removal.

- [`matlab/`](matlab/): Octave/MATLAB scripts that compute the data for the plots. `attitude_control.m` is the design desk of the control chapter; `test_design.m` checks its helpers against the Octave control package.
- [`control/`](control/): `breadbird-control`, a `no_std` Rust crate with the controller design and identification of the control chapters, on top of [minikalman](https://crates.io/crates/minikalman) and [miniident](https://crates.io/crates/miniident).

<div style="text-align: center" align="center">
    <img src="readme/actuator_disc_and_stream_tube.png" alt="Actuator Disk and Stream Tube geometry" />
</div>

