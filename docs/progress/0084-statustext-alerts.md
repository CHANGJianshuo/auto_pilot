# 0084 — STATUSTEXT alert surface（M23b + M23c）

**Commits**: `b309190`（M23b motor）、`cad1eb1`（M23c preflight + health）
**Date**: 2026-04-24  **Milestone**: M23b + M23c

## What / 做了什么

把 M23a 的 `encode_statustext` encoder **接入**到 SITL → UDP 14550 这条实际广播链路。以及把"飞控内部所有 pilot-actionable 事件"都转成 STATUSTEXT 告警。

### M23b — motor failure STATUSTEXT（commit `b309190`）

- `MavlinkUdpSink::send_statustext(severity, text)` 新 async method
- `sitl_mavlink` example 里 Snapshot 加 `motor_alive: [bool; 4]` 字段
- Main task 保留 `prev_motor_alive`，每 snapshot 做 **edge 检测**：`was_alive && !is_alive` → emit `STATUSTEXT(CRITICAL, "MOTOR N FAILED")`
- 1 个新 test：`statustext_broadcast_round_trips_over_udp`（ loopback UDP + parse verify body）

### M23c — preflight + health escalation STATUSTEXT（commit `cad1eb1`）

Snapshot 扩展：

```rust
struct Snapshot {
    motor_alive: [bool; 4],                     // M23b
    preflight: Option<PreflightReject>,         // M23c
    overall_health: HealthLevel,                // M23c
    ...
}
```

Edge 触发规则：

| 事件 | 条件 | STATUSTEXT |
|---|---|---|
| Preflight 失败开始 / 原因变化 | `None→Some` 或 `Some(a)→Some(b)` | `ERROR`，`"PREFLIGHT: <reason_str>"` |
| 健康等级升级 | `new.severity > old.severity` | `WARNING/CRITICAL/EMERGENCY`，`"HEALTH: <label>"` |
| 电机声明失效 | `was_alive && !is_alive` | `CRITICAL`，`"MOTOR N FAILED"` |

每类事件**一次 edge 一条 STATUSTEXT**，不 tick-by-tick flood。

## Why / 为什么这么做

### 为什么所有 pilot alerts 都走 STATUSTEXT（而不是混用 message 类型）

MAVLink 为不同种类的警报提供了好几种 message：
- `STATUSTEXT` — 通用文本 + severity
- `SYS_STATUS` — 传感器位图 + health byte
- `EXTENDED_SYS_STATE` — 飞行模式细节
- `HIGH_LATENCY2` — 简化 telemetry for 卫星链路
- `AUTOPILOT_VERSION_REQUEST` → `AUTOPILOT_VERSION` — 版本查询

问题：每个 GCS 对 message X 的展示规则不一样。QGC 会 SYS_STATUS 转为 HUD 颜色，但告警弹窗**只认 STATUSTEXT**。Mission Planner 反过来。MAVProxy 又一套。

**STATUSTEXT 是唯一的共同最低水平线** —— 任何 GCS 都会 toast 一个 50-byte 字符串。其他 message 也该 emit（SYS_STATUS 能让图表自动化），但把**所有 pilot-actionable 通知**重复走 STATUSTEXT 是 "always works" 的保守选择。

未来 M30+ 硬件阶段加 SYS_STATUS 等更结构化 telemetry；STATUSTEXT 保留做 primary alert channel。

### 为什么 edge-triggered 而不是 level-triggered

Tick 速率 100 Hz (Snapshot 频率)。Level-triggered 意味着 "motor 死了每秒 100 条 'MOTOR 0 FAILED'" —— 几分钟就占满 MAVLink 链路 bandwidth。

Edge 方案：**状态变化时一次**。典型飞行 5 分钟收 5-10 条告警（开机、解锁、起飞、正常、降落；有故障另加几条），reasonable 量级。

Trade-off：edge 方案对**丢包**敏感。MAVLink 走 UDP，万一那条 STATUSTEXT 丢了，GCS 永远看不到 —— 直到下次 edge 才有机会。缓解方案：关键告警（Emergency / Failed）**重播 3 次间隔 1s**。M25 加这个。

### 为什么 preflight edge 同时 catch `Some(a)→Some(b)`（不只是 None→Some）

试想场景：
- `t=0`: GPS unhealthy → STATUSTEXT "PREFLIGHT: GPS unhealthy"
- `t=3`: GPS 恢复 + baro 开始挂 → 如果只检 None→Some，pilot 永远不知道现在问题变了
- 假如 GPS 恢复后 preflight 干净几秒再 baro 挂，那会触发 None→Some → OK
- 但实际 race 常常是"GPS 和 baro 几乎同时 fix / 挂"，中间 None 状态可能只持续 1 tick

**Any transition**（包括 Some(a)→Some(b)）确保 pilot 看到**最新失败原因**。同原因持续不重播（equality check on Option<PreflightReject>）。

### 为什么 HealthLevel 的 drop 不 emit

HealthLevel 的契约：**飞行中不回滚**，只有 `reset_on_ground()` 在地面显式 reset。所以 severity 下降只可能因为：
1. 飞控代码 bug（违反契约）
2. 地面手动 reset 后重新解锁飞

情况 1 应该由 Kani / test 抓，不是 pilot 的事。情况 2 是意图行为，不需要告警。所以 drop 不 emit 是 feature。

### 为什么 "HEALTH: HEALTHY" 变体也存在但用 INFO severity

看上面表格 `HealthLevel::Healthy → MAV_SEVERITY_INFO`。但根据我们的 drop-not-emit 规则，这 variant 永远**不会 fire**（HealthLevel 不会升级回 Healthy，也不会从 Healthy 升级到 Healthy）。

留着只是 **exhaustiveness** —— Rust match 要求 covers 所有 variants。写明其语义（如果它真的发生，INFO），免得未来读代码的人 "Healthy 怎么 emit 了？" 困惑。

Dead code not reachable，但 annotated intentionally.

### 为什么不加 "RTL activated" / "LANDING started" / "TAKEOFF target reached" 告警

考虑了这些，没加。理由：
- 这些是**pilot-commanded**事件（LAND / RTL 是 MAVLink command 的直接响应）。Pilot 发命令之后期待看到 vehicle 反应，但这个反应在 ATTITUDE / GLOBAL_POSITION_INT 里已经可见（高度在下降 = LANDING）
- 加了会 spam 告警 channel
- 非故障事件不应该 level-up pilot's attention

**Alert channel 应该保持 signal 高 / noise 低**：pilot 看到 STATUSTEXT 就知道有真问题，不要让它变成 "normal status updates"

### 为什么 format! 不是 heapless 的 `write!`

Example 是 host-only target。`std::format!` 干净 / 熟悉 / O(1) 代码。embedded firmware 入口（`crates/app-copter/src/bin/firmware.rs`）未来做同样 edge-detect 时要用 `heapless::String<50>` + `core::fmt::write!`，那里会是 30+ lines of boilerplate。example 不该代替 firmware 示范 no_std 风格，它是**用户怎么用这个 API** 的文档，保持易读。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil --features mavlink-sim
# Includes statustext_broadcast_round_trips_over_udp (M23b)

$ cargo build -p sim-hil --example sitl_mavlink
# 检查 example 编译，CI 不跑但编译错会被 catch

$ cargo run -p sim-hil --example sitl_mavlink
# 手动试：QGC 打开 → 应该看见 HEARTBEAT + ATTITUDE + GLOBAL_POSITION_INT
# 注入故障（改 sim_cfg.motor_fault_mask[0] = 0.0）→ QGC toast "MOTOR 0 FAILED"
```

CI 靠 encoder/udp 那个 loopback round-trip 测试保证 byte-level 正确。Example 的 edge-detect 是简单逻辑，type-check 即验证。

## Follow-ups / 遗留

- **Retry / rate limit on critical alerts**：EMERGENCY / FAILED 告警应该重播 3× 间隔 1s 以防 UDP 丢包
- **Hardware firmware edge-detect**：M30+ 真机上把这些 edge-detect 逻辑移到 `app-copter::bin::firmware`，用 `heapless::String` + MAVLink UART 发
- **`docs/alerts.md`**：列出所有 STATUSTEXT 告警 + severity + text + 意义，让飞行员 / 测试员 / 修 bug 的人一页纸查
- **GCS-side parser test**：用 `mavproxy` 或 `pymavlink` 自己起个 GCS，验证 STATUSTEXT 能看到（端到端非 loopback 测）
- **SYS_STATUS 补充**：除了文本告警，加结构化 health byte → GCS 图表自动化

## Session cumulative（今天 2026-04-24）

今天的 commits 串：

| Milestone | Commit(s) | 类别 |
|---|---|---|
| M20d Kani on MotorFaultDetector | `3eed5b4` + `4d6e800` | 形式化 |
| M21 3-motor yaw handoff | `a50d0d5` + `747a46d` | 控制律 |
| M22 Roll flip SITL | `c48dae8` + `0367c84` | Phase III benchmark |
| Docs refresh | `17448c2` | 项目文档 |
| M23a STATUSTEXT encoder | `b4a4b8f` + `a28a75c` | MAVLink |
| M23b SITL motor-fault STATUSTEXT | `b309190` | MAVLink |
| M23c preflight+health STATUSTEXT | `cad1eb1` + this | MAVLink |

Branch `sitl-phase-iii-m10-m18` 在 GitHub 上保持同步。workspace 从 248 → ~259 tests / 28 → 34 Kani / 16 → 24 new milestones in 2 days.
