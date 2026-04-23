# Project plan

## Goal

Production-grade Rust open-source flight controller for industrial multirotor (3–30 kg) inspection / mapping. Match or beat PX4 and ArduPilot on reliability, agility, and ecosystem.

## Status summary (2026-04-24)

Everything in the "four pillars" section below has concrete SITL evidence and regression tests. **Phase III's three core benchmarks are done (figure-8, motor failure, flip)** — the fourth (Swift-class agile gate-pass) is a stretch goal that requires a trained NN. No real hardware has been attached — the entire stack runs in simulation on a developer laptop. See [`docs/progress/README.md`](./progress/README.md) for the per-commit milestone log (39 completed, 82 docs).

| Signal | Value |
|---|---|
| Commits | 170+ |
| Crates | 12 (11 `no_std`-capable) |
| Unit + integration tests | ~257 (default) + 5 (`zenoh-host`) |
| Kani formal proofs | **34** across 5 crates |
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
* **`MotorFaultDetector::apply_decision`** — alive latch monotonicity, level monotonicity, persistence reset on Quiet, BestMatch non-match reset, dead_count ≤ 4 (M20d)

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
* M12–M16: Kani expansion + README upgrade

See [`docs/progress/README.md`](./progress/README.md) for commit-hashed per-step narrative.

### Phase III (first pass) — SITL benchmark parity with Agilicious (done, 2026-04-24)

Three of the four Phase III benchmarks from the original plan have been reproduced **entirely in SITL** — no real-hardware flight yet, but each has a regression test that locks in a numeric target:

| Benchmark | Milestone | SITL evidence |
|---|---|---|
| 8-figure waypoint flight | M17 | MPC-I position RMS < 0.5 m on 2 m × 10 s Gerono lemniscate; beats PI cascade on moving target |
| Single-motor failure | M18–M21 | Detector finds dead rotor from ω̇ residual within 250 ms; 3-motor `FailoverAllocator` drops yaw and preserves lift; `outer_step` yaw handoff prevents position drift; altitude err < 3 m, xy err < 3 m, tilt < 60°; 6 Kani proofs guarantee detector latch monotonicity |
| Pitch-roll flip | M22 | INDI carries the vehicle through a 7.5 rad roll burst (past inverted), `attitude_to_rate` recovery returns upright within 1.5 s; altitude loss < 15 m, final tilt < 30°; surfaced the detector-enable flag as a mode-switch primitive |
| Swift-class agile gate-pass | (stretch) | Blocked on an external training pipeline (aerial_gym Isaac Gym / Flightmare + PPO). The `ResidualPolicy` plumbing is in place — a trained ONNX model just slots into `InferenceBackend` |

### Phase II — Real hardware (blocked on dev board)

Not started. Requires a Pixhawk 6X class FMU + ICM-42688 breakout or similar. Concrete steps:

1. Flash the 91 KiB firmware ELF via probe-rs; confirm defmt-rtt log on SWD.
2. Wire the ICM-42688 over SPI4 / SPI5; validate WHO_AM_I + sample stream.
3. Drive four DShot600 PWM outputs; verify ESC arm beep.
4. USART MAVLink to a radio (or loopback); QGC discovers the real vehicle.
5. Tie-down thrust test before free flight.
6. Small test-frame (250 mm / 5″) first flight.
7. WCET measurement via `DWT::CYCCNT`; confirm 1 kHz inner loop fits the < 500 µs budget.

### Phase III (second pass) — Real flight benchmarks (after Phase II)

Re-run the same three benchmarks — plus the Swift stretch — on real hardware and publish numbers against PX4 / ArduPilot on identical frames. The SITL pass above locks in the tracking / failure / agility contracts; Phase II + III-second-pass is where they meet actual sensors and gravity.

Concrete order of operations (proposed):

1. Small 5″ test frame (same one used for first flight in Phase II).
2. Hover hold benchmark against PX4 1.16 / ArduCopter 4.5 on the same frame, 1 m/s wind gust tunnel.
3. Figure-8 tracking — measure position RMS on GPS RTK outdoors.
4. Single-motor failure — start at 5 m AGL, ESC-disable motor 1 on PWM channel, land safely.
5. Flip — 90° bank first, then full 360° roll.
6. Swift-style gate pass (stretch) — requires a trained NN; the `ResidualPolicy` plumbing is already in place.

## Expected benchmarks (targets at v1.0)

* Position RMS (1 m/s wind, hover): < 5 cm (PX4 baseline ~15 cm, Agilicious ~3 cm). **SITL today: 22 cm on MPC-I / MPC+residual. Within striking distance with a trained NN.**
* Figure-8 tracking RMS (2 m × 10 s period, ideal sim): < 0.5 m. **SITL today: passes the MPC-I assertion; numeric target parity with Agilicious paper awaits real hardware.**
* Max body rate: > 1500 deg/s. **SITL roll-flip benchmark drives ~860 deg/s peak; torque budget on the 250 g test frame is the binding constraint, not the controller.**
* Max linear accel: > 30 m/s²
* Single-motor failure: maintain altitude, return to home. **SITL today: detector-triggered failover maintains altitude within 3 m, xy position within 3 m, tilt under 60°; Mueller-style full 3-motor controller still in the "80 % effect" zone (M21).**
* 1 kHz inner-loop WCET: < 500 µs (measured on hardware in Phase II)
* `cargo geiger` ≈ 0 unsafe outside `core-hal`, Kani proofs green (**34 today across 5 crates**)

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
