# breadbird-control

Controller design and plant identification for one attitude axis of the quadrotor, the
practical side of the chapters "State-space control" and "Identifying the plant" in
[`../typst/`](../typst/). It sits on top of
[`minikalman`](https://crates.io/crates/minikalman) (RLS, Luenberger observer) and
[`miniident`](https://crates.io/crates/miniident) (PRBS, excitation and identifiability
checks), which have no controller design of their own.

- `model`: the axis (input gain `b = l/J`, motor time constant, one sample of delay),
  discretized in closed form.
- `ident`: regressors for RLS in physical units, and `MotorLagSearch`, a bank of
  one-parameter fits that finds the motor time constant without the bias of the
  three-parameter model.
- `design`: Ackermann pole placement, the discrete Riccati equation by doubling, LQR with
  integral action, the steady-state Kalman (observer) gain. `f64`, run once.
- `law`, `pid`: state feedback with integral action and anti-windup, and an incremental
  PID. `f32`, run every sample.
- `twiddle`: coordinate search for controller parameters against a simulated plant.

`no_std`, no allocation, builds for `thumbv7em-none-eabihf` (STM32F3).

```sh
cargo test                                   # unit tests and the cross-check against Octave
cargo run --release --example bringup        # identify, design and fly a simulated axis
cargo build --lib --target thumbv7em-none-eabihf
```

The integration tests compare the design with the gains of `../matlab/attitude_control.m`,
whose helpers `../matlab/test_design.m` checks against the Octave control package.
`MINIIDENT_FINDINGS.md` collects what came up while using `miniident` here.
