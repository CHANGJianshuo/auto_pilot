# 0001 — M0 工作区脚手架

**Commit**: `7ed3421`  **Date**: 2026-04-20  **Milestone**: M0

## What / 做了什么

从零初始化 `/home/chang/auto_pilot/` 作为 Cargo workspace，12 个 crate 骨架建立：

```
crates/
├── core-hal/      HAL 层，IMU/ESC 的 trait 接口
├── core-rtos/     任务调度抽象（Priority / TaskSpec）
├── core-bus/      Zenoh-Pico 中间件 + 话题名常量
├── algo-ekf/      24 态 EKF 状态结构
├── algo-nmpc/     NMPC 外环骨架
├── algo-indi/     INDI 内环 + compute_torque_increment()
├── algo-alloc/    控制分配
├── algo-fdir/     故障检测/隔离/恢复
├── nn-runtime/    NN/RL 推理 + 安全包络
├── comms-mavlink/ MAVLink 2 适配器
├── sim-hil/       SITL/HITL host-side 桥
└── app-copter/    多旋翼应用 binary（M0：打印任务图）
```

配套文件：
- `Cargo.toml`（workspace + lints）
- `rust-toolchain.toml` 固定 Rust **1.88.0** + `thumbv7em-none-eabihf`
- `.cargo/config.toml` 交叉编译 flags（Cortex-M7 FP）
- `LICENSE`（Apache-2.0）
- `README.md`（对外介绍 4 大设计支柱）
- `CLAUDE.md`（`no_std` / `no-heap` / `no-unwrap` / Kani 不变量的规约）
- `CONTRIBUTING.md`（PR 要求）
- `docs/plan.md`（凝练版路线图，6 个 milestone）
- `docs/topics.md`（ROS 2 话题目录）

## Why / 为什么这么做

- **Rust 全栈重写**是项目第一根支柱（见 `docs/plan.md` §3）。所有 crate 都以 `no_std` 友好的方式骨架化，只有 `sim-hil`、`app-copter`（host 构建时）、测试会用 `std`。
- **Apache-2.0 而不是 GPL**：用户选了"纯开源社区驱动"路线，和 PX4 的 BSD-3 同等开放度，允许商业派生。ArduPilot 的 GPL 会吓跑工业客户。
- **12 crate 细粒度拆分**：算法层（`algo-*`）纯函数，host 可测 + Kani 可证；HAL 层（`core-hal`）是唯一允许 `unsafe` 的地方（MMIO/DMA）。这样未来换 RTOS / 换 MCU 不影响算法代码。
- **lints policy** 写进 workspace `Cargo.toml`：`unsafe_op_in_unsafe_fn=deny`、`panic/unwrap/expect/indexing_slicing=deny`。这是对 CLAUDE.md 里"不 panic、不 unwrap、不越界"的编译期强制。

## How it's verified / 怎么验证的

M0 阶段不跑代码——那时 Rust 还没装。后来 M0.5（0003 号文档）在容器里跑通了 `cargo check --workspace`，12 crate 全绿。

```bash
git log --oneline
# 7ed3421 chore: initial M0 scaffold — Rust industrial flight controller
```

## Follow-ups / 遗留

- `.github/workflows/ci.yml` 写好了但**没推上去**——两个 token 都没 `workflow` scope。需要你用网页 UI 粘或换有 scope 的 token。
- `crates/core-hal/src/traits.rs` 只放了 `ImuSource` / `ActuatorSink`，没有真实 STM32H7 实现——那是 M1 的工作。
