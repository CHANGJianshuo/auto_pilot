# 0071 — 全遥测 Zenoh 通路

**Commit**: `cc9965c`  **Date**: 2026-04-23  **Milestone**: M13

## What / 做了什么

M10b 只证了"一条 SetpointPositionMsg 能通过 Zenoh"。M13 把 core-bus **全部 7 条 topic** 端到端拉通：

```
TelemetryPublisher
├── publish_imu(msg)       → auto_pilot/imu/raw
├── publish_attitude(msg)  → auto_pilot/estimator/attitude
├── publish_velocity(msg)  → auto_pilot/estimator/velocity_ned
├── publish_position(msg)  → auto_pilot/estimator/position_ned
├── publish_setpoint(msg)  → auto_pilot/control/setpoint_position
├── publish_actuator(msg)  → auto_pilot/control/actuator_cmd
└── publish_health(msg)    → auto_pilot/system/health
```

单个 `TelemetryPublisher` 预声明全部 7 个 publisher（付一次性 round-trip 到 router 的代价），之后每 tick `publish_*(&msg).await?` 是 async-local 的。

Helper：
- `healthy_msg(ts)` — 默认"全好" HealthMsg
- `with_fault(msg, SensorFaultBit::GPS)` — 叠加位标志 + 升级 severity
- `sim_state_to_telemetry(&sim, ts)` — SimState → (Imu, Attitude, Velocity, Position) 4 元组

**测试**：`full_catalog_round_trips_through_zenoh` 在同一进程里用 2 个 peer session，**7 个 subscriber 同时订阅，1 个 publisher 依次 put，3 秒 timeout 接收 + byte-exact 比对**。多线程 tokio（zenoh 要求）。

+ `healthy_msg_has_no_faults` / `with_fault_bumps_overall_to_degraded` 两个纯逻辑 unit test。

**3 新测试**（workspace default 248 unchanged — 在 zenoh-host feature 下 +3）。

## Why / 为什么这么做

### 为什么需要全 catalog 测试，单一消息 round-trip 不够

M10b 只证了**一条**消息的 Zenoh 路径。但各个消息类型的 serde layout 不同、字段顺序不同、fixed-array vs vec 处理不同。postcard 对每种 shape 的编码都可能有 edge case：

- `ImuMsg` 有 `[f32; 3]` 数组 → postcard 按 tuple 编码
- `ActuatorCmdMsg` 有 `[f32; 8] + u8` → 固定 8 个 f32 + tag
- `HealthMsg` 有 `enum HealthLevel + [HealthLevel; 4] + u32 flags` → enum discriminant + fixed array + u32

每种至少有一个 "postcard bug 可能卡住" 的点。全部跑一遍 round-trip 是**唯一**保证 schema 和 wire format 完全对齐的方法。

### 为什么 7 个 subscriber 并行订阅

Zenoh 的 subscriber 是**被动**的 —— `.recv()` 等下一条消息。如果我按"先 put 再 declare subscriber"顺序，subscriber miss 了第一条。

正确顺序：
1. 先 declare 所有 subscriber（打开订阅队列）
2. sleep 150 ms 等 peer discovery 稳定
3. 依次 put 每条消息
4. 每个 subscriber `.recv()` 带 3 秒 timeout 抓自己的那条

这模拟**真实操作**：地面站订阅所有 topic 一次，飞机广播遥测一次。双方连上就通。

### 为什么 byte-exact `PartialEq` 比对

不用 "大致正确" 检查（比如 `abs(a - b) < 1e-3`），用 `assert_eq!`。原因：

1. **postcard 是 binary-exact** 的（没有 JSON 那种浮点打印精度问题）
2. 如果 round-trip 丢了 bit，我**想知道**是哪一个 bit —— 不是"大致差不多"
3. serde `PartialEq` derive 每个字段比对，任何退化立刻可定位

浮点 NaN 的特殊性（NaN != NaN）不是问题，因为我们 put 的消息里没有 NaN。

### 为什么 multi_thread tokio 是强制的

Zenoh 1.9 在运行时会 spawn 自己的 task 调度内部消息传递：

```
error: Zenoh runtime doesn't support Tokio's current thread scheduler.
```

单线程 tokio → Zenoh task 和我们的 task 争夺唯一 worker → 死锁。`#[tokio::test(flavor = "multi_thread", worker_threads = 4)]` 开 4 worker threads，足够 Zenoh 内部调度 + 我们 7 个 subscriber 并行 recv。

CLAUDE.md 的 "determinism" 章节没有被违反：multi-thread 是**host-only** 的测试设施；firmware embassy runtime 是单核 + 非抢占式 —— 完全不同世界。

### 为什么 `TelemetryPublisher` 持 `ZenohSession` clone

早期尝试：Publisher 直接持 `ZPublisher<'static>`，但 `'static` 生命周期要求 session Arc 永远活着。如果调用方 `drop(session)` 后还用 TelemetryPublisher，是 use-after-free（Zenoh crash 或 Sessionality 错）。

修复：TelemetryPublisher 里存一个 `_session: ZenohSession`（Arc 内部克隆）。这样：
- Publisher drop 前，TelemetryPublisher drop 前，session **必然活着**
- `ZenohSession::clone()` 便宜（Arc clone）

副作用：session close 有点晚（等 TelemetryPublisher drop），不是问题。

### 为什么 `with_fault` 自动升级到 Degraded

操作员视角：**加一个 fault flag = 至少 Degraded**。如果 HealthMsg 写着 `overall = Healthy` 但 `fault_flags != 0`，**矛盾**，操作员会困惑。

`with_fault` 自动：
```rust
if msg.overall == HealthLevel::Healthy {
    msg.overall = HealthLevel::Degraded;
}
```

已经是 Degraded/Emergency/Failed 的 msg 保持原级别（不会把 Failed 降级到 Degraded）。调用方想精确控制的话直接 `msg.overall = X` 赋值。

这个 helper **在 Kani 保证的位标志互斥合约** 上运行 —— M12 证明了 `fault_flags` 位不会 alias，所以 OR 合并不会丢信息。

### 为什么 sim_state_to_telemetry 标记 `#[must_use]`

返回的 4 元组如果 caller 忽略，就白算了（CPU 周期 + SimState 快照复制）。`#[must_use]` 让 compiler warn：
```
warning: unused `(ImuMsg, AttitudeMsg, VelocityNedMsg, PositionNedMsg)` ...
```

对一个 **pure conversion helper** 这是最小成本的正确性保护。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil --features zenoh-host zenoh_telemetry
3 passed:
  full_catalog_round_trips_through_zenoh  （7 topic × 1 message → byte-exact）
  healthy_msg_has_no_faults
  with_fault_bumps_overall_to_degraded

$ cargo test --workspace
248 passed（default feature; zenoh-host 路径另算）

$ cargo clippy -p sim-hil --features zenoh-host --all-targets -- -D warnings
全绿

$ cargo fmt --check
全绿
```

## Follow-ups / 遗留

- **SITL + Zenoh 集成**：把 TelemetryPublisher 插进 `run_closed_loop_*` 系列函数，每 tick 按 `docs/topics.md` 的 rate 自动广播。类似 `sitl_mavlink.rs` 的 Zenoh 版本。
- **Rate limiting helper**：`RatedPublisher<T>` 用 embassy-time-like Ticker 控制频率（50 Hz attitude、5 Hz position 等）。
- **Python subscriber smoke test**：`zenoh-python` 写一个订阅脚本，docker-compose 起来跑 5 秒，验证 Rust publisher 和 Python subscriber 的 postcard 互通。需要定义 Python 侧 schema（手写 postcard decoder 或 msgpack fallback）。
- **真实 ROS 2 Jazzy bridge**：启 `zenoh-plugin-ros2dds` router，subscribe via `ros2 topic echo`。Docker Jazzy 已经装了，但这个 plugin 配置非平凡，留作独立 milestone。
- **Zenoh queryable**：除了 pub/sub 加 RPC-like `get/set` API（GCS 拉 `vehicle/config`）。
- **QoS / reliability 分级**：ActuatorCmdMsg 需要 reliable，ImuMsg 可以 best-effort。`Publisher::put_reliable()` / `_best_effort()`。
- **Throughput benchmark**：1 kHz IMU publish + 50 Hz attitude + ... N 秒内 subscriber 丢帧统计。
- **Kani 扩展**：`with_fault(h, flag).fault_flags & flag != 0` 作为 harness 证明 —— 但 HealthLevel 是 enum，Kani 要 `any_health_level` helper，能做。
