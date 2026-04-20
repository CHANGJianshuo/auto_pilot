# Contributing

Thanks for considering a contribution. `auto_pilot` aims to be a **production-grade industrial flight controller**, so contributions are held to a high standard — but we try hard to make that bar reachable.

## TL;DR

1. Read [CLAUDE.md](./CLAUDE.md) for coding conventions.
2. Open an issue **before** large changes to align on design.
3. Run the local checks before pushing.
4. Safety-critical changes need a Kani proof or a written justification.

## Local checks (mandatory before PR)

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build -p app-copter --target thumbv7em-none-eabihf --release
```

## Commit / PR style

- **Conventional commits**: `feat(algo-ekf): add magnetometer innovation gate`
- Keep PRs small and focused. One logical change each.
- Include a **test** for every bug fix and every new behavior.
- Update `docs/` when you change public APIs or message schemas.

## Safety-critical code

Any change inside `crates/algo-*` or `crates/core-rtos` is considered safety-critical.
Requirements:

1. A property test (via `proptest`) covering the relevant invariants.
2. For core invariants (e.g. covariance PSD, matrix invertibility, state-machine reachability) add a Kani harness in `formal/`.
3. If a Kani proof is not feasible, add a `// FORMAL: <justification>` comment at the relevant function.

## Scope of this project

We accept contributions that:

- Improve reliability (memory safety, formal verification, better FDIR).
- Advance control performance (INDI extensions, NMPC tuning, new allocation strategies).
- Expand the ROS 2 / Zenoh ecosystem integration.
- Improve tooling (SITL, HITL, logging, benchmarks).

We generally **decline**:

- FPV racing features (Betaflight is the right place).
- Fixed-wing / VTOL work prior to multirotor v1.0 shipping (post-v1.0 is fine).
- Dependencies that pull large `std`-only code into `no_std` crates.

## Licensing of contributions

By submitting a PR, you agree your contribution is licensed under the project's Apache-2.0 license.

## Code of conduct

Be kind. Disagree on the technical merits. No harassment. Maintainers reserve the right to remove content and block users that violate this.
