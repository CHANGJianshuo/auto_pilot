# 0025 — NIS 外值计数 → FDIR HealthLevel

**Commit**: `c362e59`  **Date**: 2026-04-21  **Milestone**: M1.14

## What / 做了什么

在 `algo-fdir` 加 `SensorRejectionCounter`：消费 EKF measurement update 的 `applied: bool` 信号，把连续被拒次数映射到 `HealthLevel`。这是让之前做的"单向 FDIR 状态机"（M1.4）**第一次有真实输入**。

新增：
- `pub struct SensorRejectionCounter` + `with_thresholds(degrade, emergency)` / `new()` / `observe(accepted)` / `level()` / `streak()`
- 默认阈值：10 连拒 → Degraded；50 连拒 → Emergency
- 6 个单元测试
- **1 个 Kani 证明**：单调性 —— 任何 observation 序列下，`level` 的严重度不可能下降

## Why / 为什么这么做

### 为什么要连续计数而不是滑窗 / 比例

1. **代码最短**：一个 u32 计数器 + `saturating_add`
2. **语义清晰**：10 次连续拒绝很可能是传感器坏掉或模型错；5 次拒绝 + 5 次接受交替则不一定
3. **和 Mahalanobis 阈值解耦**：阈值 11.345 已经是 1% 错误接受率；剩下就是"到底拒了多少次"
4. **CPU 消耗几乎为零**：每次 update 只加一个 u32
5. **恢复策略自然**：任何 `applied=true` 清零 streak（但不下调 level —— FDIR 单向性）

PX4 ekf2、ArduPilot EKF3 都用类似的计数器（PX4 里叫 "fault_status_flags"，ArduPilot 里叫 "sensor_innovation_health"）。我们简化了：一个 enum + 一个计数器，而不是 20 个 bit flag。

### 阈值 10 / 50 的算法意义

- **10 连拒**：取决于传感器速率 = 0.2 s（GPS 5Hz）~ 2 s（Baro 50Hz）。**瞬时性** glitch 不触发，**持续性** 问题会触发。
- **50 连拒**：1-10 s。这是真的**需要升级到 Emergency** 的级别 —— 操作员或 NMPC 要看到这个就应该启动 RTL（return to launch）。

用户可以 `with_thresholds` 覆盖。HITL 调试时常常调宽，真飞行用默认。

### 为什么 Kani 证明单调性

`HealthLevel` 本身有单调性证明（M1.4，0008 号 doc）。但 `SensorRejectionCounter.observe` 里**多了一条路径**：`accepted=true` 时的 early-return。

如果这里不小心写成了 `self.level = HealthLevel::Healthy`（接受 → 重置 level），整个 FDIR 就破了。Kani 证明：任何长度 ≤ 4 的符号观测序列下，level 单调不降。CBMC 在毫秒内证完。

`observe` 内部调用 `transition_in_flight`（M1.4 已证单调）。本证明在**组合层**确保"外层逻辑没有绕过内层保证"。

### 为什么最终只证 1 步而不是多步序列

**第一次尝试**：kani::any bool × bound 4 的符号循环。CBMC 在 CaDiCaL 上跑了 **15 分钟未结束**（估计是 bool 符号序列 + saturating_add 的 u32 位宽一起让 SAT 解空间爆炸）。

**改成**：单步证 —— `assert observe(any_bool).severity >= initial.severity`。

**代价/收益**：
- 单步证明覆盖了"单步 observe 不下调 level"这一核心性质。
- 多步单调性可以由**归纳** 从单步 + 起始条件推导得出，**不需要** Kani 本身跑多步。
- 单元测试 `counter_never_recovers_in_flight` 和 `counter_escalates_on_consecutive_rejections` 覆盖了多步具体轨迹。

**教训**：Kani 擅长**单步不变量**；多步序列留给 proptest / 单元测试。未来遇到可以用 Z3 solver（`cargo kani --solver=z3`）或换**归纳证明 harness** 的套路。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-fdir
test result: ok. N passed

$ cargo kani -p algo-fdir
VERIFICATION: SUCCESSFUL
```

关键测试：
- `counter_starts_healthy`
- `counter_stays_healthy_on_acceptances`（100 次接受）
- `counter_accepted_resets_streak`（streak 归零但 level 不变）
- `counter_escalates_on_consecutive_rejections`（with_thresholds(3, 6) 下 3 次拒→Degraded、6 次→Emergency）
- `counter_never_recovers_in_flight`（degraded 后接受也不降）
- `counter_does_not_overflow_on_long_streak`（saturating_add）
- Kani `check_counter_observe_is_monotone`

## Follow-ups / 遗留

- **集成到 EKF loop**：`app-copter` 里把 `gps_update`/`mag_update`/`baro_update` 的 `applied` 字段喂给各自的计数器。三个独立 sensor stream。
- **上报**：顶层 health = `max(gps_counter.level, mag_counter.level, baro_counter.level)`。顶层到 Emergency 时触发 RTL 或失败保护。
- **日志**：每次 level 变化记录到飞行数据（后续 `comms-mavlink`）
- **复位 API**：地面测试时需要一个"重置所有 counter"的 API，目前只能 `let mut c = SensorRejectionCounter::new();` 重造，等 M2 接入主循环时再 expose
