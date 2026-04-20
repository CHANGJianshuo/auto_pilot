# auto_pilot — project conventions

Working notes for AI assistants and human contributors. Keep short.

## Ground rules

- Language: **Rust 1.85+, edition 2024**. No C/C++ outside vendor HAL FFI.
- `no_std` in all `crates/core-*`, `crates/algo-*`, `crates/nn-runtime`, `crates/comms-mavlink`. `std` only in `sim-hil`, `app-copter`'s host builds, and tests.
- **No heap** in safety-critical paths (EKF, INDI, allocation, FDIR). Use `heapless::Vec`, `nalgebra` stack matrices.
- `unsafe` is permitted **only** in `core-hal` (MMIO, DMA) and must carry a `// SAFETY:` comment that states the invariant relied upon.
- Every safety-critical function has either a Kani proof in `formal/` or a `// FORMAL: <reason>` comment explaining why one is not applicable.

## Module boundaries

- `core-hal` — owns peripherals, exposes `embedded-hal` traits and project-specific traits (e.g. `ImuSource`, `EscSink`).
- `core-rtos` — task scheduler, priority assignment, WCET instrumentation. No business logic.
- `core-bus` — Zenoh-Pico integration. Topic names match ROS 2 IDL in `docs/topics.md`.
- `algo-*` — pure functions of state + measurement. Deterministic, no side effects, no IO.
- `app-copter` — composes the above into a running firmware.

Rule of thumb: `algo-*` compiles on host with zero cfg tricks, enabling property tests and Kani.

## Determinism & real-time

- No `std::time`, `Instant::now`, or wall-clock IO in algo crates. Pass timestamps as arguments.
- Control loop budgets:
  - rate inner (INDI): **1 kHz, WCET ≤ 500 µs**
  - attitude: 500 Hz
  - position NMPC: 20–50 Hz
  - EKF update: 250 Hz
- Measure WCET with `defmt-trace` counters; CI enforces thresholds.

## Error handling

- `thiserror` for rich errors in host / test code.
- In `no_std` paths, return `Result<T, CoreError>` where `CoreError` is a small `#[repr(u8)]` enum.
- **Never** use `.unwrap()` / `.expect()` / `panic!()` / indexing `[]` in firmware paths. Clippy enforces.
- A `panic!()` on FMU = brownout reboot. Treat it as a crash.

## Numerical

- Use `nalgebra` with `libm` feature (no `std` math).
- Prefer `f32` on MCU for speed. Use `f64` only where EKF covariance needs it (justify in comment).
- Quaternions: Hamilton convention, normalized on store.

## Testing

- Unit tests in `#[cfg(test)] mod tests` alongside code.
- Property tests with `proptest` for invariants (quaternion normalization, covariance PSD).
- Integration tests in `tests/` exercise SITL path.
- HIL tests separate, gated behind `--features hil`.

## Git hygiene

- Conventional commits: `feat(algo-ekf): …`, `fix(core-hal): …`, `docs: …`.
- One logical change per commit. Small is fine; mixed is not.
- PRs require: `cargo fmt --check`, `cargo clippy -D warnings`, `cargo test`, Kani runs green where configured.

## What NOT to do

- Don't add `println!` / `std::fs` / `tokio` to `no_std` crates.
- Don't add dependencies without updating workspace `Cargo.toml`.
- Don't touch `formal/` harnesses without re-running Kani.
- Don't commit `Cargo.lock` edits unrelated to your change.

## Reference stacks

When you need to look at how someone else did a thing:

| Topic | Reference |
|---|---|
| EKF structure & math | `PX4/PX4-Autopilot` `src/modules/ekf2/EKF/` |
| INDI numerics | `tudelft/indiflight` |
| NMPC + allocation | `uzh-rpg/agilicious` `agilib/` |
| MAVLink parsing | `mavlink/rust-mavlink` |
| Zenoh no_std | `eclipse-zenoh/zenoh-pico` |

Copy ideas, not code (license compatibility).
