# auto_pilot

**工业级 Rust 开源飞控** — memory-safe, formally verifiable, ROS 2 / Zenoh native, NMPC + INDI control, neural-network-assisted.

> 对标 PX4 / ArduPilot / Agilicious，在可靠性、敏捷性、现代语言栈三个维度全面超越。

---

## Status

⚠️ **Early-stage scaffolding.** Not yet flight-worthy. Not for production use.

Current milestone: **M0 — workspace bootstrap** (directory layout, crate skeletons, CI).

See [plan document](./docs/plan.md) for the full roadmap.

## Why another flight controller?

- **PX4** / **ArduPilot** are written in C/C++; memory bugs are still discovered every release. Control law is classical cascaded PID; INDI and NMPC live only in experimental branches.
- **Agilicious** (UZH) implements NMPC + INDI + DFBC — but runs on a Jetson + ROS 1, not embedded-grade.
- **Rust FC attempts to date** (`RustyFlight`, `RustFlightX`, `RustFC`, …) are toy-scale or abandoned.

`auto_pilot` fills the gap: **production-grade embedded flight control in Rust** with SOTA algorithms.

## Four design pillars

1. **`core-*` — Rust `no_std` kernel.** Embassy / Hubris RTOS, zero-heap, `cargo geiger` ≈ 0 unsafe outside HAL, [Kani](https://github.com/model-checking/kani) formal proofs for safety-critical invariants (EKF covariance PSD, allocation matrix conditioning, state-machine reachability).
2. **`algo-*` — NMPC (outer) + INDI (inner) hybrid control.** Onboard QP with [Clarabel.rs](https://github.com/oxfordcontrol/Clarabel.rs). Three-layer degradation: NMPC → DFBC → PID.
3. **`nn-runtime` — onboard NN/RL inference.** [tract](https://github.com/sonos/tract) ONNX on AI coprocessor; **residual learning** on top of NMPC; safety guard rejects out-of-envelope outputs.
4. **`core-bus` — ROS 2 / Zenoh native.** No `uORB ↔ DDS` translation; [Zenoh-Pico](https://github.com/eclipse-zenoh/zenoh-pico) on FMU, MAVLink 2 for GCS compatibility.

## Target platform (first release)

**Multirotor, 3–30 kg**, industrial inspection / mapping.

Software-compatible with:
- **FMU**: Pixhawk 6X / Cube Orange+ (STM32H753) or NXP iMX RT1176.
- **AI coprocessor**: Jetson Orin Nano Super / Rockchip RK3588.
- Communication: Zenoh over 1 GbE between FMU ↔ coprocessor.

## Workspace layout

```
auto_pilot/
├── crates/
│   ├── core-hal/          # embassy-stm32 HAL wrapper
│   ├── core-rtos/         # scheduling + task partitioning
│   ├── core-bus/          # Zenoh-Pico integration
│   ├── algo-ekf/          # 24-state EKF, no_std, Kani-verified
│   ├── algo-nmpc/         # NMPC + QP
│   ├── algo-indi/         # angular-rate INDI
│   ├── algo-alloc/        # control allocation (QP with actuator bounds)
│   ├── algo-fdir/         # fault detection / isolation / recovery
│   ├── nn-runtime/        # tract ONNX inference
│   ├── comms-mavlink/     # MAVLink 2
│   ├── sim-hil/           # HITL shim (Gazebo / AerialGym)
│   └── app-copter/        # multirotor application binary
├── sim/                   # simulation configs + training scripts
├── formal/                # Kani / Creusot proofs
├── tests/                 # integration & HIL
└── docs/                  # design docs & plan
```

## Getting started

### Prerequisites

```bash
# Install rustup + stable toolchain (pinned by rust-toolchain.toml)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add thumbv7em-none-eabihf
```

### Build (host side)

```bash
cargo check --workspace --target x86_64-unknown-linux-gnu
cargo test  --workspace --target x86_64-unknown-linux-gnu
```

### Build (firmware)

```bash
cargo build -p app-copter --target thumbv7em-none-eabihf --release
```

### Lint

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
```

## Roadmap (condensed)

| Milestone | ETA | Deliverable |
|-----------|-----|-------------|
| M0 | month 1 | workspace scaffold, HAL skeleton, CI |
| M1 | month 3 | IMU/GPS drivers, 24-state EKF, SITL hover |
| M2 | month 6 | INDI inner loop, QP allocation, HITL |
| M3 | month 9 | NMPC outer loop, single-motor-failure recovery |
| M4 | month 12 | NN coprocessor integration, first autonomous flight |
| M5 | month 18 | Agile maneuvers (Swift-class), v1.0 release, benchmarks vs PX4/ArduPilot |

## License

[Apache-2.0](./LICENSE) — commercial derivatives permitted.

## Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md). Safety-critical code requires a Kani proof or a written justification for why one is not applicable.
