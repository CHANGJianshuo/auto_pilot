# 0075 — Landing + Takeoff pure fn 三连发

**Commits**: `a8be3e6`（M16b LandingState）、`91200fb`（M16c TakeoffState）
**Date**: 2026-04-23  **Milestone**: M16b + M16c

## What / 做了什么

继 M16a 把 `RtlPhase` 重构成 pure function + Kani 证之后，把 **`LandingState`** 和 **`TakeoffState`** 同款处理。三个 in-flight mode 状态机现在**全部**有：

- pure `advance()` 返回 **closed enum of directives**
- 零 `libm::sqrtf` 在 transition path 上
- Kani 证明 "Idle 是吸收态 + 只能通过指定的一条出口退出"

```
RtlPhase::advance      → RtlTransition { Stay, Advance(..), Handoff }
LandingState::advance  → LandingTransition { Stay, Complete }
TakeoffState::advance  → TakeoffTransition { Stay, Reached }
```

`outer_step` 每个状态机对应 3 行 match-apply，副作用明确可见。

**Workspace Kani 证明数**：22 → **28**（新增 6 个：3 Landing + 3 Takeoff）。

**TouchdownDetector.observe** 的 `sqrt(vx² + vy²)` 也换成 `vx² + vy² < threshold²` 配合 M16a 的 RtlPhase，同理由。

**SITL 测试**：app-copter 27 tests + workspace 248 全绿，行为 byte-identical。

## Why / 为什么这么做

### 为什么三个状态机需要统一处理

前 M16a 的动机是"**让 RTL 可形式化**"。但架构对称性要求：如果 RTL 是 pure function，Landing 和 Takeoff 也应该是 —— 否则新人读代码会问"**为什么 RTL 有 advance/transition enum，landing 还是内嵌 if-else？架构不一致**"。

三路统一后：
- 一眼看 `outer_step` 就看清楚每个 mode 怎么转换
- `XxxTransition` enum 的 variants 写明"能发生的事"
- Kani 跨三路套用同一种 harness pattern

### 为什么 Landing 和 Takeoff 各有 3 proof

每个都证三件事：

| 证明 | 意义 |
|---|---|
| **Idle is absorbing** | 不靠外部指令，不会自发启动 |
| **{Landing/TakingOff} only exits via {Complete/Reached}** | 封闭退出形状 |
| **Idle does not touch detector** | 累加器状态不被不该调用的路径污染 |

第三条看起来琐碎但很重要。场景：

1. 第一次 takeoff → altitude reached → detector.settled_s 归零（caller reset on Reached）
2. 巡航中偶尔 takeoff_state 被外部误读（极端 race 场景）
3. 第二次真正 takeoff 开始 → 如果 Idle 曾不知不觉累加 settled_s，**第二次 takeoff 会提前触发 Reached**

Kani 证明"Idle 不动 detector"**预先排除**这种 silent bug。

### 为什么 `TakeoffTransition::Reached` 不是 `Complete`

命名约定：
- **Landing Complete** = "降落完成，飞机触地" — 终态
- **Takeoff Reached** = "到达目标高度" — 不一定 idle，操作员可能立即接新 setpoint

语义细微但真实：Complete 暗示整个 mode 结束，Reached 只说"altitude criterion 满足"。outer_step 里两者的行为（state → Idle + detector.reset）恰巧相同，但如果未来加 "reached 后 auto-goto-next-waypoint" 之类的 behavior，Reached 名字已经留好空间。

### 为什么 TakeoffState::TakingOff 带负载要 Kani 小心处理

```rust
pub enum TakeoffState {
    Idle,
    TakingOff { target_z_ned: f32 },
}
```

Kani 符号化 enum 变体时要枚举**每种 payload**。`f32 target_z_ned` 是连续值，Kani 会把它设为 `any_finite_f32()` 符号值。proof 内容不依赖具体值，所以 Kani 能在 SMT 里一次 cover。

如果 variant 带**多字段 + 相互约束**（比如 `TakingOff { start_z, target_z, deadline }` 且要求 `start_z > target_z`），Kani harness 需要 `kani::assume(start_z > target_z)` — 每多一个约束多一条 assume。当前单字段最简单。

### 为什么 `#[derive(PartialEq)]` on `TakeoffState` 但不 `Eq`

带 `f32` payload 的 enum 不能 derive `Eq`（f32 不是 Eq trait 的实现者，因为 NaN ≠ NaN）。`PartialEq` 够用；tests 比较时 `assert_eq!` 工作正常，除非有人 assert NaN == NaN（永远 false）。

这条在 M6.2 就决定了，M16c 继承。

### 为什么所有 mode 转换都要暴露 Idle 为 `#[default]`

每个 enum 的 `Idle` 都是 `#[default]`，`FlightState::default()` 因此零配置得到"全静默"初态。开机 / 重启 / unit test 初始化都从 Idle 出发，状态机没意外初值。

## How it's verified / 怎么验证的

```bash
$ cargo kani -p app-copter
10 successfully verified harnesses, 0 failures, 10 total.
Verification Time: 34 ms

$ cargo kani -p core-bus core-rtos algo-fdir algo-ekf app-copter
5 crates, 28 proofs total, ~10 s end-to-end in CI

$ cargo test -p app-copter --lib
27 passed (behaviour identical to M16a baseline)

$ cargo test --workspace
248 passed

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿
```

## Follow-ups / 遗留

- **Kani `preflight_check` completeness**：证明每个 `FlightState` 组合（每种 sensor health × 协方差范围）正好映射到预期的 `PreflightReject` 变体，或 Ok。
- **Kani `ArmState` transition**：目前 arm/disarm 是外部 MAVLink 指令写入的枚举，没有 "advance" 函数。但加一层 pure fn `ArmState::can_transition_to(requested, preflight)` 可以证 "disarm 永远允许、arm 需要 preflight Ok"。
- **`PositionController::step` 输出不变量**：证明不管变体 + 输入如何，输出 `AttitudeAndThrust.thrust_n >= 0` 且 quaternion 有限。f32-heavy，可能需要 Kani 绕过。
- **Transition 证明自动化**：三个证明手工写的，形状几乎相同。用 `macro_rules!` 模板化 `absorbing_is_stay!(LandingState::Idle, ...)` 之类。但可读性会下降，评估是否值得。
- **README badge**：CI Kani job 输出 "28 verified harnesses"，README 加 shield badge。
- **State-machine 图**：用 mermaid 画三路 transition 图（纯文本 ASCII 或图片），放 docs/architecture.md。
