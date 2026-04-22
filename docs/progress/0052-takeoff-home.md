# 0052 — TAKEOFF + 起飞位置记忆

**Commit**: `d2d1469`  **Date**: 2026-04-22  **Milestone**: M6.2

## What / 做了什么

飞控终于有了 **"home"** 概念——第一次 arm 时自动记下 EKF 位置估计，作为后续自动模式（TAKEOFF / 将来的 RTL）的参考锚点。

对称于 M5.7 的 LAND，新增：
- `TakeoffState { Idle, TakingOff { target_z_ned: f32 } }` —— 带负载的枚举，目标高度直接存。
- `AltitudeReachedDetector`：|z - target| < 0.3 m AND |v_z| < 0.3 m/s 持续 1 秒 → 完成。
- `FlightState::home_position_ned: Option<Vector3<f32>>`：首次 arm 时 latch。
- `outer_step` 新行为：TakingOff 态**原地爬升**（home xy 固定，z 推向 target），完成自动退出。
- `RateLoopConfig::takeoff_climb_mps = 1.0 m/s`：默认爬升速率。

MAVLink 层：
- `sim_hil::mavlink_udp::takeoff_request_from_mav_message -> Option<f32>` 解 `MAV_CMD_NAV_TAKEOFF` (id 22)，返回 `param7` 作为目标高度（米）。
- Demo 收到 TAKEOFF **自动 arm**（单点发指令即起飞），分享 `Arc<Mutex<TakeoffState>>`。
- `COMMAND_ACK` 现在识别 TAKEOFF → ACCEPTED。

**测试**：+3 app-copter（home latch 语义 / takeoff 完成 / detector 边界）+1 sim-hil（TAKEOFF 解析 vs LAND 不匹配），23 app-copter + 24 sim-hil 测试全绿。

## Why / 为什么这么做

### 为什么 Landing 和 Takeoff 不共用一个 mode 枚举

考虑过定义 `FlightMode { Manual, TakingOff, Landing, Mission, ... }`。但：
- Landing 和 TakeOff **不互斥**于"手动控制"——它们都是**临时覆盖** setpoint 的 mode；
- 如果共用 enum，切到 `Landing` 会隐含"不是 Takeoffing"，反之亦然——但我们希望两者都可以**独立存在**并独立有完成回调；
- 带负载的枚举 `TakingOff { target_z_ned }` 更清晰——target altitude 属于 mode 的一部分，放到 FlightMode 里会污染所有变体。

现在的方案：两个独立的小 enum（LandingState、TakeoffState），outer_step 里**优先级硬编码**（Landing > TakeOff > caller setpoint）。扩展到 RTL 时再加第三个，或者真的做 FlightMode。

### 为什么 Landing 优先 TakeOff

```rust
if landing { landing_override() }
else if takeoff { takeoff_override() }
else { caller_setpoint }
```

语义："操作员说 land 时就要落，不管当前在不在 takeoff"。安全第一。

### 为什么 home 用 `Option` 而不是 `Vector3::zeros()` 占位

- `Option::is_none()` 准确区分 **"没 arm 过"** 和 **"home 就在原点"**；
- 如果 RTL 时发现 `home = None` 可以报错 / 用当前位置作为 fallback；
- 显式 None 比"魔数零位置"清晰。

### 为什么 first-arm latch，不是 "每次 arm 都 latch"

场景：操作员在 A 点 arm 起飞、降落、disarm。然后**不挪动飞机**再 arm 起飞——home 应该仍是 A 点。如果每次 arm 都 latch，第二次 arm 时 EKF 可能已经漂了几厘米，home 会漂移。

First-arm 一次到位符合"home = 起飞场地"的直觉。

更通用的方案：`MAV_CMD_DO_SET_HOME` 允许 GCS 显式指定 home，或者 `MAV_CMD_DO_SET_HOME` 带 param1=1 的"use current" 模式。M6.4 候选。

### 为什么 TAKEOFF 自动 arm

PX4/ArduPilot 的习惯：TAKEOFF 必须先 arm。如果 GCS 发了 TAKEOFF 但 vehicle 还 disarmed，会直接 REJECTED。

为什么不这样？
- **Demo 场景**：操作员只想**一键起飞**，不关心两步握手；
- **MAVLink spec 没禁止 auto-arm**：只是推荐先检查 preflight。
- **真机固件**：应加 preflight 检查（GPS lock、baro 正常、battery >20%），失败返 `TEMPORARILY_REJECTED`。M6.4 候选。

### 为什么爬升速率 1 m/s

- 250 g 机 hover thrust ≈ 2.45 N，每电机 0.6 N；motor_max = 6.0 N（10× hover）——爬升加速度上限 ~45 m/s²；
- 1 m/s 匀速爬升需要 **额外** ~0.25 N / motor（很小）；
- 爬太快（> 3 m/s）EKF 的 baro/GPS 更新跟不上，估计误差大；
- 保守选 1 m/s，typical indoor / small outdoor 场景。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib
20 passed, 包含:
 * home_position_latches_on_first_arm
 * takeoff_mode_drives_z_toward_target_and_completes
 * altitude_reached_detector_fires_and_resets

$ cargo test -p sim-hil
23 passed, 包含 takeoff_request_parses_target_altitude

$ cargo test --workspace
所有 crate 绿

$ cargo fmt --check && cargo clippy --workspace --all-targets -- -D warnings
全绿

$ cargo build -p sim-hil --example sitl_mavlink
Finished
```

**QGC 预期交互**（还没真机测 QGC 有 takeoff 按钮的界面）：
1. demo 启 + QGC 连；
2. QGC 发 `MAV_CMD_NAV_TAKEOFF param7=5.0` → console 看到 `TAKEOFF command received — climbing to target altitude_m=5`；
3. vehicle 自动 arm → 爬升到 -5 m（5 m 高）；
4. 1 s 内停稳 → TakeOff state 回 Idle，demo 维持 hover setpoint (0,0,-1) —— 注意：**此处 demo 会开始试图回到 -1 m**，因为 sitl_mavlink 的默认 setpoint 是 -1 m。真正任务应该让 takeoff 成功后让 GCS 设 waypoints。

## Follow-ups / 遗留

- **Takeoff 完成后 setpoint 应该停在 target_z_ned**：现在 TakeOff 完成后 demo 又回到 (0,0,-1) 会立刻下降 4 米，挺怪的。应该让 takeoff 完成时顺便把 shared setpoint 更新为 target。
- **`MAV_CMD_DO_SET_HOME`**：让 GCS 显式设 home，不只是 first-arm 的副作用。
- **Preflight checks**：arm / takeoff 前检查 EKF 收敛、GPS lock ≥ 3D、baro/mag 健康、electric 电压；失败返 `TEMPORARILY_REJECTED`。
- **TAKEOFF at current location, yaw**：当前忽略 param4 (yaw)、param5/6 (lat/lon)；生产固件应honour。
- **RTL (M6.3)**：有了 home 就可以做。
- **自动下降 rate 适应电量**：电量低时放慢爬升，腾出余量。
- **M6.2 takeoff 测试覆盖闭环 SITL**：目前 `takeoff_mode_drives_z_toward_target_and_completes` 是"飞机已在 target 再 step"的设定，没真的从地面飞上来。完整 SITL 闭环测试更有说服力。
