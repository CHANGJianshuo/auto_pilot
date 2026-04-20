# Project plan (condensed)

Full plan: `/home/chang/.claude/plans/clever-watching-metcalfe.md` (local to Claude harness).
Published plan below is the shareable subset.

## Goal

Production-grade Rust open-source flight controller for industrial multirotor
(3–30 kg) inspection / mapping. Match or beat PX4 and ArduPilot on
reliability, agility, and ecosystem.

## Four pillars

1. **Rust `no_std` kernel** — memory safety + Kani-verified invariants.
2. **NMPC + INDI hybrid control** — onboard, with PID baseline as last resort.
3. **Onboard NN/RL policy inference** — residual learning on top of NMPC, safety-guarded.
4. **ROS 2 / Zenoh native** — no uORB↔DDS translation, direct ROS 2 IDL.

## Comparison with existing stacks

| Project | Strength | Why not adopt directly |
|---|---|---|
| PX4 | Modular, ROS 2 bridged, BSD | C/C++ memory risk; PID-first; uORB→DDS translation layer |
| ArduPilot | Most mature, widest hw support | GPL-3 traction in commercial chains; legacy code style |
| Betaflight / iNav / EmuFlight | Best-in-class FPV inner loop | No position / mission scope |
| Agilicious (UZH) | NMPC + INDI + DFBC baselines | Jetson + ROS 1, not embedded-grade |
| indiflight (TU Delft) | Production INDI on Betaflight fork | Still FPV scope, no navigation |
| Swift (UZH) | RL champion-level racing | Code not fully open, racing-only |
| Rust FCs (RustyFlight, RustFC, RustFlightX) | Exist | Toy scale or abandoned |
| Auterion Skynode | Commercial-grade FMU + AI companion | Closed source on top of PX4 |

Gap: **no production-grade Rust FC exists.** That is the opening.

## Milestones

| ID | Target month | Deliverable |
|---|---|---|
| M0 | 1  | workspace scaffold, HAL skeleton, CI |
| M1 | 3  | IMU / GPS / mag drivers, 24-state EKF, SITL hover |
| M2 | 6  | INDI inner loop, QP allocation, HITL hover |
| M3 | 9  | NMPC outer loop, 1-motor-fail recovery in sim |
| M4 | 12 | AI coprocessor link, residual NN on hardware, first autonomous flight |
| M5 | 18 | Swift-class agile maneuvers, v1.0 release, public benchmark vs PX4 / ArduPilot |

## Expected benchmarks (target at v1.0)

- Position RMS (1 m/s wind, hover): < 5 cm (PX4 baseline ~15 cm, Agilicious ~3 cm)
- Max body rate: > 1500 deg/s
- Max linear accel: > 30 m/s²
- Single-motor failure: maintain altitude, return to home
- 1 kHz inner-loop WCET: < 500 µs
- Safety audit: `cargo geiger` ≈ 0 unsafe outside `core-hal`, Kani proofs green

## Architecture

See `README.md` for workspace layout and each crate's role.

## Risks

| Risk | Mitigation |
|---|---|
| Rust embedded linalg maturity | `nalgebra` no_std + libm; fall back to `cortex-m-dsp` FFI if needed |
| RTOS real-time characteristics | Benchmark Embassy vs. Hubris vs. RTIC in M0; pick based on data |
| NMPC compute budget on MCU | Horizon length tuning; offload to coprocessor if needed |
| sim-to-real gap for NN policy | Swift-style empirical noise model + domain randomization |
| Small team bandwidth | Narrow to multirotor only until v1.0; defer fixed-wing / VTOL |
