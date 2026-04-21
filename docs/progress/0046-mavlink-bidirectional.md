# 0046 — MAVLink 双向：QGC 能指挥 SITL 飞了

**Commit**: `7570dcb`  **Date**: 2026-04-21  **Milestone**: M5.5

## What / 做了什么

SITL demo 现在不仅**发**遥测，还能**收**地面站的命令。具体：当 QGroundControl 用户点 "Go To Location" 时发出的 `SET_POSITION_TARGET_LOCAL_NED`，demo 会**在几毫秒内更新 setpoint**，控制器带着飞机去那里。

改动：
- `sim_hil::mavlink_udp::MavlinkUdpSink::try_recv() -> Result<Option<(MavHeader, MavMessage, SocketAddr)>>`
  - 非阻塞 `try_recv_from` + M5.4 的 `parse_frame`
  - 损坏帧默默丢弃（WouldBlock / parse error 都返回 None）
- `sim_hil::mavlink_udp::setpoint_from_mav_message(&MavMessage) -> Option<Setpoint>`
  - 识别 `SET_POSITION_TARGET_LOCAL_NED` 并映射到 `algo_nmpc::Setpoint`
- demo example 用 `Arc<Mutex<Setpoint>>` 在 tokio 主任务和 spawn_blocking sim 线程之间共享 setpoint

4 新测试（sim-hil 共 17 测试）。

## Why / 为什么这么做

### 线程间数据流

```
┌────────────────────────────┐        ┌──────────────────────────┐
│ tokio main task (async)    │        │ spawn_blocking sim task │
│                            │        │                          │
│ while sim alive:           │        │ loop {                   │
│   drain mpsc (latest snap) │ <───── │   publish Snapshot      │
│   drain UDP (set setpoint) │ ─────> │   read Setpoint (Arc)   │
│   send MAVLink telemetry   │        │   rate_loop_step()       │
│   sleep 5ms                │        │   physics step           │
└────────────────────────────┘        └──────────────────────────┘
                ↕ UDP socket
           QGroundControl
```

### 为什么 `Arc<Mutex<Setpoint>>`

Setpoint 是**写少读多**：
- UDP 主任务偶尔（每次 GCS 发命令时）写一次
- sim 线程每 ms 读一次

用 Mutex 的原因：
- 跨线程共享（不能 Cell）
- 数据不大（Setpoint = 10 f32），拷贝便宜；Mutex < 100 ns lock/unlock
- Arc 让 sim 线程和 main 任务各持一份 handle

若更激进可以换 `atomic_float` 存 12 个分量 + seqlock 模式，但对 1 kHz 控制循环 Mutex 足够。

### 帧错误为什么默默丢

```rust
Err(ParseError::Incomplete | ParseError::Corrupt | ParseError::Unknown) => Ok(None),
```

UDP 下"收到错帧"是常态（radio glitch、rate mismatch、GCS 发了别的 dialect 的消息）。**log 会刷屏**，**退出 recv 会丢后续消息**。工业级策略：
- 记一个 counter（`error_frames_total`）
- 大于阈值才进入健康告警

当前 demo 没上 metrics —— 默默丢是最安全的默认。实际部署加 counter 就行。

### SET_POSITION_TARGET_LOCAL_NED 字段映射

```rust
MavMessage::SET_POSITION_TARGET_LOCAL_NED(data) => Setpoint {
    position_ned: (data.x, data.y, data.z),
    velocity_ned: (data.vx, data.vy, data.vz),
    accel_ned: (data.afx, data.afy, data.afz),   // feed-forward
    yaw_rad: data.yaw,
}
```

忽略 `type_mask`（QGC 默认 0 = 所有字段有效）。忽略 `yaw_rate`（我们 Setpoint 只支持 absolute yaw）。

生产级还要：
- 按 `type_mask` 选择性更新（比如只改位置不动 yaw）
- `coordinate_frame` 支持 `MAV_FRAME_LOCAL_OFFSET_NED`（相对当前位置）
- 超出飞行包线的 setpoint 拒绝 + 上报警告

### `try_recv_parses_heartbeat` 的 polling

tokio 的 `try_recv_from` 是**立即返回**。但 UDP 包从 sender 到 receiver 的传输有几十到几百微秒延迟。测试里用 polling loop：

```rust
for _ in 0..20 {
    if let Some(...) = sink.try_recv().unwrap() { return; }
    tokio::time::sleep(Duration::from_millis(5)).await;
}
panic!("did not receive within 100 ms");
```

这比纯 `try_recv` 一次检查更健壮。生产代码用 `recv_from().await`（阻塞）或 epoll，但测试里 polling + 100 ms 超时最简单。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil mavlink_udp
test result: ok. 6 passed; 0 failed

$ cargo test --workspace
  所有 crate 绿

$ cargo build -p sim-hil --example sitl_mavlink
  Finished

$ cargo clippy --workspace --all-targets -- -D warnings
  Finished
```

**真机验证**：
1. 容器里 `cargo run -p sim-hil --example sitl_mavlink -- 172.17.0.1:14550`
2. 宿主开 QGC → 自动连上 Zurich vehicle
3. 地图上点 "Go To Location"，选个点 → vehicle 飞过去
4. console 看到 `tracing::info! target_ned = ...`

## Follow-ups / 遗留

- **M5.6**：ARM/DISARM command (COMMAND_LONG + MAV_CMD_COMPONENT_ARM_DISARM)、LAND/RTL 命令
- **Type mask honoring**：只写某些字段
- **多 coordinate_frame**：`MAV_FRAME_LOCAL_OFFSET_NED` 等
- **MANUAL_CONTROL**：从 QGC joystick 发 RC-style 输入
- **遥控器桥接**：SBUS/ELRS 接收 → 转 MAVLink `MANUAL_CONTROL` 给 SITL 和实机
- **PARAM_VALUE/PARAM_SET**：运行时参数调整
