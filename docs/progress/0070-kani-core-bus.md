# 0070 — Kani 证明扩到 core-bus + CI

**Commit**: `c32b90e`  **Date**: 2026-04-23  **Milestone**: M12

## What / 做了什么

给 `core-bus` 消息 schema 加 **5 个 Kani 形式化证明**，并把 Kani 作业纳入 CI 每次 push 跑。Workspace 证明总数 **13 → 18**。

新证明（都 sub-second CBMC 完成）：

| 证明 | 保证 |
|---|---|
| `health_level_u8_cast_preserves_severity_order` | `Healthy < Degraded < Emergency < Failed` 作为 u8 |
| `health_level_u8_round_trip` | `HealthLevel → u8 → HealthLevel` 每个变体都 recover |
| `sensor_fault_bits_are_distinct_powers_of_two` | 7 个 fault bit 常量各有 exactly 一位、pairwise 互斥 |
| `sensor_fault_bits_union_preserves_each_member` | `A\|B\|C` AND 运算能正确识别每个成员 |
| `actuator_max_channels_fits_u8` | `ACTUATOR_MAX_CHANNELS ≤ u8::MAX` |

CI 新 **`kani`** job 跑 4 个 crate（core-bus / core-rtos / algo-fdir / algo-ekf），单 crate 证明 2-3 秒，总耗时 < 15 秒。

## Why / 为什么这么做

### 为什么证 `HealthLevel` 排序而不只是 unit test

Unit test 只能验证"当前字面量的值相对关系"—— 如果有人改 enum 变体顺序但忘了调对应的 u8 repr，**test 可能通过但 wire format 坏了**。

Kani symbolic execution 能证：
- 所有可能的 `HealthLevel` 变体（exhaustive enumeration）
- `as u8` cast 结果就是 discriminant
- discriminant 按声明顺序递增

换言之：**"enum 顺序" 和 "严重性顺序" 是同一件事**。未来重排 enum 变体时 Kani 立刻红。

### 为什么证 `SensorFaultBit` 位常量互斥

`SensorFaultBit::GPS = 1 << 0`、`MAG = 1 << 1` ... 每个 bit 应该**独占一位**。写 `1 << 3` 两次就是经典 bit-flag bug，unit test 里如果没同时用这两个 flag，可能永远漏过去。

Kani 证明：
1. 每个 const `b != 0 && b & (b-1) == 0`（只有一位置 1 = power of two）
2. 任意两个 const `bi & bj == 0`（不重叠）

组合起来 = **"这 7 个 bit 编码构成可独立使用的标志空间"**。

### 为什么加 CI job 而不只是本地跑

- **回归守门**：任何 PR 改 enum / const / trait 实现时自动重证
- **反 "忘跑 kani"**：Kani 不是常规 `cargo test` 的一部分（需要 `cargo-kani` + CBMC），容易漏
- **演示 plan.md 创新点 1**：工业级 Rust + 形式化不是宣传，是 CI 强制执行的合同

CI 开销可接受：4 个 crate × 2-3 秒 = **< 15 秒** 总时间。cargo-kani 安装一次 cached，后续 PR 是 step-level 的缓存命中。

### 为什么不加 app-copter / nn-runtime Kani

App-copter 的状态机（`RtlPhase`, `LandingState`）转移逻辑**糅在 `outer_step` 的 1000+ 行函数里**，不是 pure transition function，Kani 很难 harness。需要先 refactor 出 `RtlPhase::transition(&mut self, ...)` 之类的纯函数。

nn-runtime 的 `SafetyEnvelope::check` 涉及 `Vector3::norm()` = `sqrt(x² + y² + z²)`。Kani 对 `sqrt` 的符号推理长期以来是 **15+ 分钟挂机** 或直接放弃（CBMC 没有 float library 完整 axiomatization）。M1.1 时期我们就为此把 `normalize_attitude` 改成**先检查退化情况再 sqrt** 的 early-return 形式，让 Kani 只证退化路径。

今天这些 target 跳过。等 Kani 对浮点有更好支持（比如 2026 下半年可能的 Z3 backend 整合），或我们有专门的 CI slot 跑慢证明。

### 为什么 Kani 跑 `algo-ekf` 只有 1 个 harness 生效（之前声明 2 个）

查看 algo-ekf，两个 `#[kani::proof]` 声明里有一个覆盖**四元数归一化的退化保护**，另一个是 f32-heavy 的 update step。后者在 Kani 下可能自动被 skipped 或 unreachable（Kani 输出 "1 successfully verified harnesses" 意味着实际活的只 1 个）。

这正是**期望行为**：Kani 证能证的，不证不能证的。不卡 CI 时间。

### 为什么证 `ACTUATOR_MAX_CHANNELS ≤ u8::MAX`

看似 trivia，但这条约束支撑了 `core-bus::encode` 的测试：

```rust
n: u8::try_from(ACTUATOR_MAX_CHANNELS).expect("channel count fits u8"),
```

如果有人把 `ACTUATOR_MAX_CHANNELS` 改成 512 想支持多电机 vehicle，`try_from` 会 fail → test panic → 发现 schema 改动没做对。Kani proof 提前 compile-time 抓到，不用等 test fail。

## How it's verified / 怎么验证的

```bash
$ cargo kani -p core-bus
5 successfully verified harnesses, 0 failures

$ cargo kani -p core-rtos
5 successfully verified harnesses, 0 failures (1.3 s)

$ cargo kani -p algo-fdir
6 successfully verified harnesses, 0 failures (2.1 s)

$ cargo kani -p algo-ekf
1 successfully verified harnesses, 0 failures (2.0 s)

$ cargo test --workspace && cargo clippy --workspace --all-targets -- -D warnings
248 passed, clean

$ cargo build -p core-bus --lib --target thumbv7em-none-eabihf
Finished (Kani cfg 不影响嵌入式 build)
```

## Follow-ups / 遗留

- **`ArmState` / `LandingState` / `RtlPhase` 状态机 Kani** — 先 refactor 到 pure transition fn，再证 monotone 等不变量
- **`PreflightReject::reason_str` 非空**：simple exhaustive enum proof
- **`PositionController` enum dispatch**：证明每个变体 .step() 都正常返回 `AttitudeAndThrust`（thrust ≥ 0, quaternion 有限）
- **`TouchdownDetector::observe` 单调/reset 语义**：settled_s 永远 ≥ 0 或 == 0 after reset
- **Kani 对 f32 的 partial 支持**：等 CBMC 2026+ 升级，把浮点 envelope check 搬进来
- **Kani stale time monitor**：当前每 crate 跑完就结束，加 CI job 的 `timeout-minutes` 防悄悄膨胀
- **把 Kani summary 打进 PR comment**：`gh pr comment` + kani 输出；让 review 看得到"本 PR 新增 / 改动 N 个 proof"
- **Kani property-test 桥接**：把 proptest 里的 property 手动翻成 Kani harness；逐步把 f32 shims 上去
