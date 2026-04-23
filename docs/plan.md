# Project plan

## Goal

Production-grade Rust open-source flight controller for industrial multirotor (3–30 kg) inspection / mapping. Match or beat PX4 and ArduPilot on reliability, agility, and ecosystem.

## Status summary (2026-04-23)

Everything in the "four pillars" section below has concrete SITL evidence and regression tests. No real hardware has been attached — the entire stack runs in simulation on a developer laptop. See [`docs/progress/README.md`](./progress/README.md) for the per-commit milestone log (32 completed).

| Signal | Value |
|---|---|
| Commits | 142+ |
| Crates | 12 (11 `no_std`-capable) |
| Unit + integration tests | 248 (default) + 5 (`zenoh-host`) |
| Kani formal proofs | 28 across 5 crates |
| CI jobs | 7 (fmt / clippy / test / test-zenoh / build-thumbv7em / kani / geiger) |
| Firmware ELF on STM32H753 | 91 KiB (release) |

## Four pillars

### 1. Rust `no_std` kernel + formal verification — ✅ foundation complete

All flight-critical crates (`core-*`, `algo-*`, `comms-mavlink`, `nn-runtime`, `app-copter`) compile for `thumbv7em-none-eabihf`. Workspace-level `deny(unwrap, panic, indexing_slicing, as_conversions)` with narrow overrides. Kani proofs cover:

* EKF quaternion normalisation degenerate path (M1.1)
* Priority total-order round-trip (M1.2)
* HealthLevel state-machine monotonicity (M1.4)
* SensorRejectionCounter single-step observe (M1.14)
* `core-bus` HealthLevel u8 ordering + SensorFaultBit disjointness (M12)
* `RtlPhase::advance` absorbing / exit-shape invariants (M16a)
* `LandingState::advance` + `TakeoffState::advance` absorbing / exit-shape (M16b, M16c)
* `core-rtos` Priority ordering + rank round-trip (M1.2)
* `algo-fdir` HealthLevel transition invariants (M1.4)

**Pending:** `preflight_check` completeness, `ArmState` transition Kani, `PositionController::step` output finiteness. f32-heavy targets still await better CBMC float support.

### 2. NMPC + INDI hybrid control — ✅ five-way controller lineup

Position loop (`algo-nmpc::PositionController<H>` enum):

| Controller | Mechanism | Use case |
|---|---|---|
| **PI cascade** | Hand-tuned PI with conditional anti-windup | Baseline, maximum interpretability |
| **LQR** | Infinite-horizon Riccati, closed-form gains | Model-perfect, no integrator |
| **MPC** | Finite horizon (H = 10, 1 kHz), box-constrained QP, projected-gradient solver, LQR terminal cost | Constraint awareness |
| **LQI** | LQR + integrator state, augmented 3-D Riccati | Steady-state bias-kill |
| **MPC-I** | MPC + integrator, augmented QP | Both constraint + bias |
| **MPC + NN residual** | MPC accel + `ResidualPolicy<B>` + safety envelope | Pillar 3 integration |

Attitude + rate: `algo-indi` incremental nonlinear dynamic inversion + LPF on ω, α. `algo-alloc` X-quad control allocation with per-motor saturation.

Shootout in realistic sim (1.5 m/s wind, drag, motor lag, 15 s):

```
PI cascade     : 0.015 m  (integrator matches drag time constant by luck)
LQR            : 5.179 m  (bias-blind)
MPC            : 2.736 m  (bias-blind, horizon too short for steady bias)
LQI            : 0.239 m  (integrator cancels bias)
MPC-I          : 0.226 m  (constraints + integrator)
MPC + residual : 0.221 m  (hand-tuned PD residual matches integrator at steady state)
```

Reproduce with `cargo run -p sim-hil --example controller_shootout --release`.

### 3. Onboard NN/RL residual — ✅ trait + reference backend + SITL

`nn-runtime` exposes an `InferenceBackend` trait so any runtime (tract, candle, onnxruntime) can slot in once the crate reaches a stable 1.0. Current reference implementation is `AffineBackend` — pure Rust `y = W·x + b` with per-axis clamp. `ResidualPolicy<B>` binds inference to a `SafetyEnvelope` that rejects non-finite / over-accel / velocity-runaway outputs.

SITL evidence: MPC + hand-tuned affine residual under wind beats bare MPC by > 2 m horizontal tracking; matches LQI/MPC-I at steady state.

**Pending:** real trained model (requires external training pipeline — aerial_gym Isaac Gym / Flightmare + PPO, not in scope for this sim-only phase).

### 4. ROS 2 / Zenoh native middleware — ✅ host integration complete

Seven typed messages in `core-bus`: IMU / Attitude / Velocity / Position / Setpoint / ActuatorCmd / Health. Serde + postcard codec, `no_std`, `heapless::Vec<u8, 256>` wire buffer, 5 Kani proofs on the schema invariants.

`sim-hil::zenoh_bus` — Zenoh 1.9 host session wrapper with typed `Publisher<M>` / `Subscriber<M>`. `sim-hil::zenoh_telemetry` — SITL runner that broadcasts the full catalogue at the per-topic rates in `docs/topics.md`:

* IMU 1 kHz • Actuator 1 kHz
* Attitude / Velocity / Position 250 Hz
* Setpoint 50 Hz
* Health 10 Hz

Tested: two peer sessions, 7-topic byte-exact round-trip; 600-tick SITL broadcast with counted subscribers proving ≥ 90 % delivery at each rate.

**Pending:** `zenoh-pico` firmware integration (needs hardware to validate), ROS 2 Jazzy bridge via `zenoh-plugin-ros2dds` (non-trivial router config), cross-language `zenoh-python` subscriber.

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

**Gap remaining:** no production-grade Rust FC exists. `auto_pilot` fills that gap at the SITL tier; real-hardware validation is the next phase.

## Phase plan

The original month-based plan (below) from project kickoff underestimated how quickly SITL would come together. Reality:

### Phase I — SITL-complete (done, April 2026)

Everything in the four-pillars section. Key milestones:

* M0–M2: workspace + EKF + INDI + allocation + PI baseline SITL hover
* M3: realistic SITL (drag, wind, motor lag, noise) + PI cascade bias fix
* M4: wind-observable EKF Jacobian + drag-aware predict
* M5: MAVLink stack complete (ARM / LAND / TAKEOFF / RTL / preflight / COMMAND_ACK)
* M6: CI + firmware ELF link for STM32H753
* M7: embassy runtime + defmt-rtt + probe-rs + ICM-42688 generic driver
* M9: five-way position controller (LQR / MPC / LQI / MPC-I) + shootout regression
* M10: core-bus typed messages + Zenoh host pub/sub
* M11: residual policy + SITL evidence
* M12–M16: Kani expansion (28 proofs) + README upgrade

See [`docs/progress/README.md`](./progress/README.md) for commit-hashed per-step narrative.

### Phase II — Real hardware (blocked on dev board)

Not started. Requires a Pixhawk 6X class FMU + ICM-42688 breakout or similar. Concrete steps:

1. Flash the 91 KiB firmware ELF via probe-rs; confirm defmt-rtt log on SWD.
2. Wire the ICM-42688 over SPI4 / SPI5; validate WHO_AM_I + sample stream.
3. Drive four DShot600 PWM outputs; verify ESC arm beep.
4. USART MAVLink to a radio (or loopback); QGC discovers the real vehicle.
5. Tie-down thrust test before free flight.
6. Small test-frame (250 mm / 5″) first flight.
7. WCET measurement via `DWT::CYCCNT`; confirm 1 kHz inner loop fits the < 500 µs budget.

### Phase III — Real flight benchmarks (after Phase II)

Reproduce Agilicious-paper benchmarks on our stack:

* 8-figure waypoint flight
* Pitch-roll flip
* Single-motor failure recovery (ArduPilot has it, PX4 doesn't)
* Swift-class agile pass through gates (stretch goal; requires NN policy training)

Publish numbers against PX4 / ArduPilot on identical frames.

## Expected benchmarks (targets at v1.0)

* Position RMS (1 m/s wind, hover): < 5 cm (PX4 baseline ~15 cm, Agilicious ~3 cm). **SITL today: 22 cm on MPC-I / MPC+residual. Within striking distance with a trained NN.**
* Max body rate: > 1500 deg/s
* Max linear accel: > 30 m/s²
* Single-motor failure: maintain altitude, return to home
* 1 kHz inner-loop WCET: < 500 µs (measured on hardware in Phase II)
* `cargo geiger` ≈ 0 unsafe outside `core-hal`, Kani proofs green

## Architecture

See [`README.md`](../README.md) for workspace layout and each crate's role.

## Risks

| Risk | Mitigation status |
|---|---|
| Rust embedded linalg maturity | ✅ Validated — `nalgebra` no_std + `libm` covers everything used |
| RTOS real-time characteristics | ✅ Embassy picked; `#[embassy_executor::main]` compiles on thumbv7em. Real timing await Phase II |
| NMPC compute budget on MCU | 🟡 Horizon 10 + projected-gradient with warm-start is cheap enough on SITL; hardware WCET tbd |
| sim-to-real gap for NN policy | 🟡 Residual-learning approach is compatible with any domain-randomisation pipeline; training not yet started |
| Small team bandwidth | ✅ Narrowed to multirotor only; no fixed-wing / VTOL work underway |
| Tract / candle ONNX dep stability | 🟡 Currently use pure-Rust affine backend; swap to real ONNX when tract 1.0 stable lands |
| No access to hardware | ❄️ Phase II blocked until a dev board arrives |

## Status glossary

✅ = delivered with regression test or Kani proof
🟡 = partially delivered / in progress
❄️ = blocked on external resource (hardware, upstream dep)

---

For the per-commit, per-step narrative, read [`docs/progress/README.md`](./progress/README.md). Each row has a commit hash and a short why/what/how-verified entry.
