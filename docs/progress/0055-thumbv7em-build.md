# 0055 — app-copter 编译进 thumbv7em 了

**Commit**: `7a708f8`  **Date**: 2026-04-22  **Milestone**: M7.0

## What / 做了什么

之前 workspace 虽然 CLAUDE.md 规定了 "`no_std` in all `crates/core-*`, `crates/algo-*`, `crates/nn-runtime`, `crates/comms-mavlink`. `std` only in `sim-hil`, `app-copter`'s host builds, and tests"，但 `app-copter::lib` 自己**不是** `no_std` —— 只是因为 host binary 把 std 带进来而已。跑一下 `cargo build -p app-copter --lib --target thumbv7em-none-eabihf` 就 26 个错误：`cannot find Some in scope`、`.sqrt not in scope`……

这一步：
- `app-copter/src/lib.rs` 头部加 `#![cfg_attr(not(test), no_std)]` —— production 构建 no_std，test 构建仍然 std（proptest 需要）；
- 两处 `f32::sqrt()`（TouchdownDetector 横向速度、RTL xy 距离）换成 `libm::sqrtf`；
- 验证**整个 workspace 所有 11 个可嵌入 crate**（core-hal / core-rtos / core-bus / algo-ekf / algo-nmpc / algo-indi / algo-alloc / algo-fdir / nn-runtime / comms-mavlink / **app-copter**）都能编译到 `thumbv7em-none-eabihf`；
- CI 的 `build-firmware` job 从占位 `echo` 换成真实的 `cargo build --lib --target thumbv7em-none-eabihf` 循环，每个 crate 单独 group，失败时 log 立刻定位。

Host 行为完全不变：`cargo test --workspace` 182 pass，`cargo clippy` 绿，host binary 照常构建。

## Why / 为什么这么做

### 为什么 `#![cfg_attr(not(test), no_std)]` 而不是纯 `#![no_std]`

纯 `#![no_std]` 会让单元测试文件也 no_std —— 但 `proptest`、`assert!` 格式化消息里的 `String` 都需要 `std`。

条件 no_std 的 idiom:
```rust
#![cfg_attr(not(test), no_std)]
```

含义：**除了 test 编译单元以外**都 `no_std`。生产构建（包括 thumbv7em）严格无 std；单元测试可以用 std 的便利。

另一个常见写法是 `#![cfg_attr(not(feature = "std"), no_std)]`，把 std 做成可选 feature —— 更灵活但需要定义 feature、维护两套依赖列表。单用 `test` 条件简单得多。

### 为什么必须用 `libm::sqrtf` 而不是 `f32::sqrt`

Rust 标准库 `f32::sqrt` 实际上是一个 `std`-only 方法，因为它依赖 libm（数学库）但绑定定义在 `std::f32`。`core::f32` 没有这个方法。

在 no_std 下标准做法：
- `libm::sqrtf(x)` / `libm::sqrt(x)`（f32 / f64）
- `libm::cosf`, `libm::sinf`, `libm::atan2f` ... 一整套

`libm` 是个 pure Rust 的 C libm 移植，已经在 workspace deps 里（`nalgebra` 依赖它来启用 no_std 数学）。

### 为什么两处 `.sqrt()` 只是改调用，不是大改

两处都是局部几何计算（向量范数、xy 距离），没有任何 std-only 前提。改动是 1-line mechanical replacement。没有算法影响。

### 为什么 CI 用 `cargo build --lib` 而不是 `--lib --tests`

`--tests` 会尝试构建 test binary，这些又回到 std → thumbv7em 上必然失败。`--lib` 只构建库本身，验证"可嵌入"这一件事。

另一种方案：`cargo build --workspace --target thumbv7em-none-eabihf` ——但这会触发 sim-hil / test binary，反而 fail。per-crate 循环最精确。

### 为什么 M7.0 到此为止

原本打算 M7.0 要：
1. no_std lib ✅
2. 写 `#![no_main]` + `cortex-m-rt::entry` firmware binary
3. 加 `memory.x` 链接脚本
4. CI 链接产物出 `.elf`

但 2-4 依赖具体板卡（Pixhawk 6X = STM32H753，iMX RT1176 不同），需要挑一个并定 pin layout。这不是"基础设施"工作，是"驱动启动代码"。保留给 **M7.1 core-hal embassy-stm32**。

M7.0 现在的价值：**不可逆保证** —— 从今往后任何人往这些 crate 里写 `std::` 或 `f32::sqrt()`，CI build-firmware 立刻红。**不让 std 渗透到应用层**是"Rust 飞控工业级" 承诺的基石。

## How it's verified / 怎么验证的

```bash
$ cargo build -p app-copter --lib --target thumbv7em-none-eabihf
Finished

$ for crate in core-hal core-rtos core-bus algo-ekf algo-nmpc algo-indi algo-alloc algo-fdir nn-runtime comms-mavlink app-copter; do
    cargo build -p $crate --lib --target thumbv7em-none-eabihf
  done
全部 Finished

$ cargo test --workspace
182 passed

$ cargo fmt --check && cargo clippy --workspace --all-targets -- -D warnings
全绿

$ cargo build -p app-copter --bin app-copter     # host binary still works
Finished
```

CI push 后 `build (thumbv7em)` job 应该从"instant echo success"变成 ~1 分钟的真实编译。

## Follow-ups / 遗留

- **M7.1 `cortex-m-rt` entry + memory.x**：真正链出 firmware `.elf`；先搞 STM32H753（Pixhawk 6X）。
- **M7.2 embassy-stm32 HAL drivers**：SPI → ICM-42688 IMU 驱动，UART → MAVLink serial 出口，DShot600 PWM → ESC。
- **M7.3 core-rtos on embassy**：当前 `core-rtos` 只是类型定义，真的调度要对接 embassy-executor。
- **app-copter 没有嵌入式 main.rs**：lib 准备好了，bin 还是 host-only。M7.1 加个 `src/bin/firmware.rs`。
- **cargo-binstall probe-rs + flash**：CI 可以编译但不能 flash —— 真机流程未走过。
- **链接时 size 分析**：`cargo size` / `cargo bloat` 测量 firmware 大小，确保 < 1 MB 放进 H7 flash。
- **把 `libm` 依赖集中化**：现在散落在 algo-* 和 app-copter 各自用 libm，可以 workspace.dependencies 统一版本（已经统一了，但要 double-check）。
- **thumbv8m.main-none-eabihf (Cortex-M33)** 也加到 targets：iMX RT1176、STM32H5 用 M33，更新硬件会需要。
