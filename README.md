# auto_pilot

**Industrial-grade open-source flight controller in Rust.** Memory-safe, formally verifiable, ROS 2 / Zenoh native, NMPC + INDI control with neural-network residual compensation.

> 对标 PX4 / ArduPilot / Agilicious，在可靠性、敏捷性、现代语言栈三个维度建立差异化。

---

## Status

**Host-side SITL is working end-to-end.** Every algorithm and integration in the list below has concrete SITL evidence + regression tests. No real hardware has been attached yet — the whole stack runs in simulation on a developer's laptop.

| Signal | Count |
|---|---|
| Crates | 12 (11 `no_std`-compatible, 1 host-only) |
| Unit + integration tests | ~257 (default feature) + 5 (`zenoh-host` feature) |
| Formal proofs (Kani) | **34** across 5 crates, run on every CI push |
| Continuous-integration jobs | 7 (fmt, clippy, test, test-zenoh, build-thumbv7em, kani, geiger) |
| Workspace git commits | 170+ |
| Phase III benchmarks (SITL) | **3 of 4** (figure-8, motor failure, flip; Swift gate-pass is stretch) |

## Four design pillars — status

1. **`core-*` — Rust `no_std` kernel + formal verification.** ✅  
   Embassy async runtime, zero-heap in safety-critical paths, workspace-level `deny(unwrap, panic, indexing_slicing)`, **34** Kani proofs covering: 24-state EKF normalisation, FDIR monotone transitions, flight-mode state machines (RTL / Landing / Takeoff), ROS 2 message schema invariants, **MotorFaultDetector latch monotonicity + persistence reset invariants (M20d)**.
2. **`algo-*` — NMPC + INDI hybrid control.** ✅  
   Five interchangeable position controllers (PI / LQR / MPC / LQI / MPC-I) behind a single `PositionController<H>` enum. Finite-horizon box-constrained MPC with projected-gradient solver, LQR terminal cost, warm-start across ticks. Agilicious-style angular-rate INDI inner loop. **M19 `FailoverAllocator` for single-motor loss (4 pre-computed 3-motor pseudoinverses, Mueller 2014 "drop yaw"); M21 yaw handoff in `outer_step` to keep position tracking under spin.**
3. **`nn-runtime` — onboard NN residual + safety envelope.** ✅  
   `InferenceBackend` trait with a pure-Rust affine reference impl; `ResidualPolicy` bundles inference + envelope check; SITL demo shows a hand-tuned residual **halves** tracking error of bare MPC under wind.
4. **`core-bus` — ROS 2 / Zenoh native middleware.** ✅ host side.  
   7 typed messages with postcard codec, end-to-end Zenoh pub/sub across two peer sessions, SITL runner broadcasts the full telemetry catalogue at documented rates (IMU 1 kHz, attitude 250 Hz, etc.). Firmware zenoh-pico integration pending hardware.

### Phase III benchmarks — SITL reproduction of Agilicious targets

| Benchmark | Milestone | SITL contract |
|---|---|---|
| 8-figure waypoint (lemniscate) | M17 | MPC-I position RMS < 0.5 m, max err < 1.5 m, beats PI cascade on moving target |
| Single-motor failure | M18–M21 | Detector declares within 250 ms; `FailoverAllocator` drops yaw; altitude < 3 m, xy < 3 m, tilt < 60° |
| Pitch-roll flip | M22 | INDI drives past inverted (q_w < 0); recovery back to tilt < 30° in 1.5 s; altitude loss < 15 m |
| Swift-class gate-pass | (stretch) | `ResidualPolicy` plumbing ready; awaits external NN training pipeline |

## Controller shootout

From `cargo run -p sim-hil --example controller_shootout --release` — realistic sim with 1.5 m/s wind, 0.05 N·s/m linear drag, 0.02 N·s²/m² quadratic drag, 20 ms motor lag, 15 s flight:

| Controller     | horizontal err (m) | altitude err (m) |
|----------------|-------------------:|-----------------:|
| PI cascade     |              0.015 |            0.000 |
| LQR            |              5.179 |            0.077 |
| MPC            |              2.736 |            0.001 |
| LQI            |              0.239 |            0.007 |
| MPC-I          |              0.226 |            0.007 |
| MPC + residual |              0.221 |            0.002 |

Bias-blind LQR / MPC drift on constant wind; integrator-based LQI / MPC-I clamp to ~0.2 m; MPC + hand-tuned residual matches MPC-I without any trained model (an ONNX-loaded network is expected to extend the lead).

## Why another flight controller?

- **PX4** / **ArduPilot** are written in C/C++; memory-safety bugs keep appearing. Control is classical cascaded PID; INDI and NMPC live only in experimental branches. Message bus (uORB) is 2014-era and needs a translation layer to reach ROS 2.
- **Agilicious** (UZH RPG) implements NMPC + INDI + DFBC — but runs on a Jetson + ROS 1, not embedded-grade.
- **Rust FC attempts to date** (`RustyFlight`, `RustFlightX`, `RustFC`, …) are toy-scale or abandoned.

`auto_pilot` is a production-grade embedded flight controller in Rust with SOTA algorithms. Every creative claim above has a regression test or Kani proof you can run in < 60 seconds on a laptop.

## Target hardware (first release)

**Multirotor, 3–30 kg**, industrial inspection / mapping.

Software is compatible with:

- **FMU**: Pixhawk 6X / Cube Orange+ (STM32H753) or NXP iMX RT1176. `cargo build --bin firmware --target thumbv7em-none-eabihf` produces a 91 KiB linkable `.elf` on the H753 memory layout.
- **AI coprocessor**: Jetson Orin Nano / Rockchip RK3588 for the residual NN if the model outgrows the FMU's compute.
- **Middleware**: Zenoh over 1 GbE between FMU ↔ coprocessor; ROS 2 subscribers tap into the same Zenoh session from a ground station.

The firmware-side peripheral drivers (SPI IMU, UART MAVLink, DShot ESC) are designed and `embedded-hal`-generic but **not yet flashed to a real board**.

## Workspace layout

```text
auto_pilot/
├── crates/
│   ├── core-hal/          # HAL + driver ICM-42688 (SPI-generic)
│   ├── core-rtos/         # scheduling priorities + WCET budgets
│   ├── core-bus/          # ROS 2 / Zenoh messages + postcard codec
│   ├── algo-ekf/          # 24-state EKF with drag-aware predict, Kani-verified
│   ├── algo-nmpc/         # 5-controller lineup: PI / LQR / MPC / LQI / MPC-I
│   ├── algo-indi/         # angular-rate INDI inner loop
│   ├── algo-alloc/        # X-quad control allocation
│   ├── algo-fdir/         # fault detection, 6 Kani proofs on HealthLevel
│   ├── nn-runtime/        # residual policy + safety envelope
│   ├── comms-mavlink/     # MAVLink 2 encode + parse
│   ├── sim-hil/           # host SITL harness + Zenoh bridge
│   └── app-copter/        # multirotor firmware + host entry + mode state machines
├── docs/progress/         # per-milestone design docs (~75 files)
├── .github/workflows/     # 7-job CI: fmt / clippy / test / zenoh / thumbv7em / kani / geiger
└── CLAUDE.md              # project conventions for AI assistants + humans
```

## Getting started

### Prerequisites

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add thumbv7em-none-eabihf
```

The workspace pins Rust 1.88.0 via `rust-toolchain.toml`. A Docker dev container is provided at the repo root for reproducible builds.

### Build + test (host)

```bash
cargo test --workspace                          # 248 tests, no extra features
cargo test -p sim-hil --features zenoh-host     # + 5 Zenoh round-trip tests
```

### Run the controller shootout

```bash
cargo run -p sim-hil --example controller_shootout --release
```

### Run the MAVLink SITL demo (QGroundControl on UDP 14550)

```bash
cargo run -p sim-hil --example sitl_mavlink
# Open QGC; it auto-discovers the vehicle. Arm / Land / Takeoff / RTL all wired.
```

### Build firmware for STM32H753

```bash
cargo build -p app-copter --bin firmware \
    --target thumbv7em-none-eabihf --release
# Produces a 91 KiB ELF. Flashing with probe-rs is configured in
# .cargo/config.toml but verification requires real hardware.
```

### Formal verification

```bash
cargo kani -p core-bus core-rtos algo-fdir algo-ekf app-copter
# 34 proofs, ~10 s total on a laptop.
```

### Lint + format

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
```

## Differentiation vs PX4 / ArduPilot

| Dimension | PX4 / ArduPilot | `auto_pilot` |
|---|---|---|
| Language | C / C++ | Rust `no_std` |
| Safety-critical unwrap / panic policy | review + lint | `deny` at workspace level |
| Formal proofs | none (Kani ecosystem didn't exist) | 28 harnesses on every CI push |
| Default controller | cascaded PID | 5-way choice incl. MPC + LQI + NN residual |
| Message bus | uORB 2014 + uxrce-DDS bridge (2020) | Zenoh 1.9 native (day-1) |
| NN / RL on-vehicle | none | residual policy + safety envelope |
| Onboard WCET analysis | runtime timing | DWT-based static budgets + CI check (planned) |

## Roadmap

See [`docs/progress/README.md`](./docs/progress/README.md) for a per-commit log of 39+ completed milestones from `M5.6` (MAVLink ARM) through `M22` (Phase III roll-flip SITL). Each row has a commit hash and a short narrative.

The blueprint in [`docs/plan.md`](./docs/plan.md) remains the north star; we've reached the "everything verified in SITL" tier and are holding on real-hardware work until a dev board is available.

## License

[Apache-2.0](./LICENSE) — commercial derivatives permitted.

## Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md) and [CLAUDE.md](./CLAUDE.md).

Safety-critical code requires a Kani proof or a written `// FORMAL:` justification for why one isn't applicable. Every merged PR ships with (1) at least one test, (2) green CI on all 7 jobs, (3) a progress-doc entry in `docs/progress/`.
