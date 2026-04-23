# 0072 — SITL runner 真的通过 Zenoh 广播遥测了

**Commit**: `648b68a`  **Date**: 2026-04-23  **Milestone**: M14

## What / 做了什么

M13 的 `TelemetryPublisher` 之前只在单元测试里被手动 put 过 7 条消息。M14 把它**塞进 closed-loop SITL**：sim 跑多久，telemetry 就流多久，7 条 topic 各按自己的速率自动广播。

```rust
let publisher = TelemetryPublisher::new(&session).await?;
let final_state = run_closed_loop_with_zenoh_telemetry(
    &sim_cfg,
    seed,
    ticks,
    &publisher,
).await?;
// ticks ms 的仿真运行 + 每 tick 按 docs/topics.md 的速率广播
```

速率表（严格按 `docs/topics.md`）：

| Topic | 频率 | 每 N tick |
|---|---|---|
| `imu/raw` | 1 kHz | 1 |
| `control/actuator_cmd` | 1 kHz | 1 |
| `estimator/attitude` | 250 Hz | 4 |
| `estimator/velocity_ned` | 250 Hz | 4 |
| `estimator/position_ned` | 250 Hz | 4 |
| `control/setpoint_position` | 50 Hz | 20 |
| `system/health` | 10 Hz | 100 |

**测试**：`sitl_broadcasts_every_topic_at_documented_rate` 起 2 个 peer session、6 个 counting subscriber；600 tick SITL → subscribers 数自己收到多少条、10% 下界容差断言速率；最后 assert 飞机还在 30 cm 内悬停（publisher 没饿死控制器）。

+1 测试 → workspace default 248 不变；zenoh-host feature 下 3→4 tests。

## Why / 为什么这么做

### 为什么把 app-copter 设成 zenoh-host feature dep

之前 app-copter 是 sim-hil 的 **dev-dependency** —— 非 test build 不拉。现在 `run_closed_loop_with_zenoh_telemetry` 需要调 `rate_loop_step` / `FlightState` 这些 app-copter API。两个选择：

1. **Feature gate**：`zenoh-host = ["dep:app-copter", ...]` → 只在 zenoh-host 用户才拉 app-copter
2. **全 main dep**：所有用户都拉 → 无条件增加 sim-hil 依赖图

选 (1)：
- 不用 zenoh 的人（比如 sitl_mavlink demo）**不付 app-copter 编译代价**
- 和 zenoh dep 一起 opt-in，意图清楚
- 默认 test matrix 快

代价：同一个 crate (`app-copter`) 在 `[dependencies]` 里 feature-gated + 在 `[dev-dependencies]` 里无条件。Cargo 合并两份：dev build 总会拉，feature build 也会拉，其他 build 不拉。正确行为。

### 为什么 publisher 工作 inline 在 tokio 主任务上

设计选择：
- **A**：SITL 在 `spawn_blocking` 线程，publish 请求经 mpsc 送回 tokio 线程
- **B**：整个 SITL 在 async 函数里跑，inline `publish_*().await`

选 (B)：
- **简单**：无 mpsc，无 join/select 复杂度，无 drop semantics 坑
- **测试场景**：每 tick 一次 await，loopback Zenoh < 100 µs，600 tick 全部 < 60 ms
- **这不是生产 firmware loop**：embassy 的 rate_loop_task 不会这样写；这个函数定位是 host-only 测试工具

注释里明写了这条警告。如果有人想把这函数上板，会立刻看到 "don't ship it as a firmware loop"。

### 为什么 count_until_idle 用 500 ms idle-timeout

Subscriber 怎么知道"publisher done 了"？三种方式：

1. Publisher 发个 "end" 消息 → 额外 topic，schema 污染
2. 用 `tokio::select!` 带一个 cancel channel → 测试复杂度
3. **Idle timeout**：recv 500 ms 里没收到就认为 publisher 停了 → 简单、鲁棒

600 tick = 0.6 s sim，每 tick publish ~7 条消息，间隔 < 1 ms。500 ms 空窗 = publisher 至少停了 0.5 s，肯定结束。

这个 pattern 也不依赖 publisher 精确时序 —— 网络抖动、tokio 调度延迟都不会误触发 "idle"（因为 500 ms 远超任何合理抖动）。

### 为什么 tolerance 是 10% 下界

- **上界不需要**：publisher 发多少，subscriber 最多收多少（Zenoh 不变魔术变多）
- **10% 下界**：
  - Peer discovery 启动后前 ~100 ms subscriber 可能还没 ready → 丢早期消息
  - 10% × 600 tick = 60 tick = 60 ms 容差，匹配 Zenoh 内部 discovery 时间

如果实际测试里丢超过 10%，那不是"网络抖动"而是**真有 bug**（rate 算错、publisher 死锁、序列化丢帧）。

所有 6 个 topic 实测都是 **刚好 expected 附近**，远没触及 10% 下界 —— 好信号。

### 为什么同时断言飞行性能

仅测 "telemetry 速率对"不完整。如果 publisher 的 `.await` 饿死了 MPC 线程，sim 步进会漏 tick，**telemetry 速率看起来还对**（基于 tick 计数），但**真机就会撞地**。

加 `altitude_err < 0.3 m` assertion 保证了：
- SITL 控制器在 publishing 时仍然正常工作
- Publisher 开销没把 closed loop 拖爆

反之如果飞机在 30 cm 外，要么 controller 退化要么 sim 异常 —— 不管是哪个，都是真问题。

### 为什么 ZenohBusError::Zenoh(String) 吃"MPC 不收敛"错误

`run_closed_loop_with_zenoh_telemetry` 返回 `Result<SimState, ZenohBusError>`。MPC DARE 不收敛是 `Option::None` 而不是 Zenoh 错误 —— 强行装进 `ZenohBusError::Zenoh` 有点 overload。

替代方案是 **定义专门的 RunnerError enum**：
```rust
enum RunnerError { MpcFailed, Zenoh(ZenohBusError) }
```

但 runner 错误类型只会被此函数的调用方见到，弄一个新 enum + `impl From` chain 成本比收益高。选了最简路径。如果将来 runner 失败模式多样化（比如 "setpoint 无效"），再拆 enum。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil --features zenoh-host zenoh_telemetry
4 passed (1.7 s)：
  sitl_broadcasts_every_topic_at_documented_rate  ← 新增 M14
  full_catalog_round_trips_through_zenoh
  healthy_msg_has_no_faults
  with_fault_bumps_overall_to_degraded

$ cargo test --workspace
248 passed (default matrix unchanged)

$ cargo clippy -p sim-hil --features zenoh-host --all-targets -- -D warnings
全绿（修了 7 处 lint：expect、loop→while let、整数除法、iter-indexing 等）

$ cargo fmt --check
全绿
```

## Follow-ups / 遗留

- **MAVLink + Zenoh 并行 demo**：扩展 `sitl_mavlink.rs` 让它同时向 QGC（MAVLink UDP）**和** Zenoh 发遥测，操作员 choice of stack
- **SITL + NMPC+residual**：目前 runner 用裸 MPC；换成 `MpcResidualController` 让 NN residual 也上 Zenoh
- **Rate-limited publisher helper**：embassy-time-style `Ticker`-驱动 `publish_at(rate_hz, || msg)` 抽象，让 rate 逻辑不用散在调用方
- **Python subscriber smoke test**：`zenoh-python` 跑一个订阅脚本，docker-compose 起 5 秒订阅，验证 Rust → Python 跨语言通
- **Rate assertion 精确化**：当前 ± 10%；加一个 "频率方差 < 5%" 断言确认发布间隔抖动在可控范围
- **Rate scheduler**：用 embassy Ticker 模拟的纯-Rust rate limiter，给真机 firmware 准备好 onboard 等效物
- **Zenoh QoS 选择**：actuator_cmd reliable、IMU best-effort 分级 `put_reliable()` / `put_best_effort()`
- **吞吐量 benchmark**：criterion 测 1 kHz IMU 在 10 s SITL 下的 CPU/网络开销基线
