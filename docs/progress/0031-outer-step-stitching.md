# 0031 — outer_step：位置 setpoint 一路到电机推力

**Commit**: `43f3226`  **Date**: 2026-04-21  **Milestone**: M2.6

## What / 做了什么

把 M2.5 的 position 环 + M2.4 的 attitude 环 + M2.3 的 rate loop 在 `app-copter` 里**缝起来**。调用方只给 `Setpoint`，函数返回 4 个电机推力。

新增：
- `RateLoopConfig` 加字段 `k_attitude`, `mass_kg`, `position_gains`
- `pub fn outer_step(cfg, flight, imu, dt, setpoint) -> RateLoopOutput`
- `default_config_250g()` 更新含所有新增默认值
- 2 集成测试

## Why / 为什么这么做

### 完整控制栈的链条

```
Setpoint (position/velocity/accel/yaw)
    │
    ▼
position_to_attitude_thrust   ← M2.5: P-P cascade
    │
    │  (q_desired, thrust_n)
    │
    ▼
attitude_to_rate              ← M2.4: quaternion error × k_att
    │
    │  rate_cmd (body ω)
    │
    ▼
rate_loop_step                ← M2.3: INDI → allocate → clamp
    │
    │  motor_thrusts (4)
    │
    ▼
→ ESC / motor driver
```

每层都在独立 crate 里，每层都有 proptest 和单元测试。`outer_step` 是把它们**按正确的顺序和数据流**串起来。

### 为什么临时覆盖 `hover_thrust_n`

`rate_loop_step` 的签名里 `VirtualCommand::thrust_n` 来自 `cfg.hover_thrust_n`（单独调用 rate_loop_step 时用的 fallback）。外层调用时我们有**实时的** thrust demand 来自 `position_to_attitude_thrust.thrust_n`，要用这个而不是硬编码的 hover。

最干净的写法是给 `rate_loop_step` 加个参数 `override_thrust: Option<f32>`，但这破坏已有 API。我选择**临时覆盖+恢复**模式（save/restore），单线程代码里这是安全的：

```rust
let saved = cfg.hover_thrust_n;
cfg.hover_thrust_n = att.thrust_n;
let out = rate_loop_step(cfg, flight, imu, dt_s, rate_cmd);
cfg.hover_thrust_n = saved;
```

后续重构可以把 `thrust_n` 作为 `RateLoopOverrides` struct 的一部分传入。

### 为什么 outer_step_hover 容差是 5 cm / 10 cm/s

单元测试跑 1000 次带 mock IMU（纯重力读数），EKF 没有外部测量更新，状态完全靠 predict。1 kHz × 1 s 积累：
- 位置漂移 ~5 cm（浮点累积 + accel_ned 残差）
- 速度漂移 ~10 cm/s（同上）

工业目标（根据 docs/plan.md）：悬停精度 < 5 cm。但那是**有 GPS/mag/baro 测量更新**的情况。本测试只验证"**不发散**" —— 实际精度需要 SITL 测。

### 为什么 outer_step_altitude 只检查"thrust 增加"

期望 `z_setpoint = -1 m`（1 m 高空）。单步 outer_step 的推力应该 > 2.45 N hover。但具体数值取决于：
- P 位置误差 1 m × k_pos.z = 2 m/s 速度命令
- 速度误差 2 m/s × k_vel.z = 10 m/s² → 被 max_accel=8 m/s² 饱和
- 有效 accel_cmd_z = -8 m/s²（NED 向上）
- thrust_vec = m × (accel - g) = 0.25 × (-8 - 9.8) = -4.45 N
- 总推力 |thrust_vec| = 4.45 N

具体 4.45 N 确实 > 2.45 N hover，断言通过。具体值留给 SITL 回归（那里能测真实动力学）。

### outer_step 的 WCET 估算

每次调用做：
- position_to_attitude_thrust：几十 flop + sqrt 三次 + cross product 一次 + 3×3 matrix 几 flop + from_matrix 大约 30 flop
- attitude_to_rate：Hamilton 积 28 flop + 3 乘
- rate_loop_step：EKF predict 大约 2000 flop（24×24 矩阵乘是最大头）+ 3 次 Vector3 乘加

总共 ~3000 flop × 1 kHz = 3 Mflop/s。Cortex-M7 FPU 典型 200 MFlop/s，负载 1.5%。**余量极大**，未来加 NMPC 外环（几百-几千 flops × 50 Hz）完全容得下。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib
test result: ok. 5 passed; 0 failed
  含 3 新 tests: outer_step_hover_at_origin + outer_step_altitude_setpoint
                + 之前的 3 个 rate_loop 测试

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **SITL 桥**（`sim-hil`）：把 `outer_step` 接到 Gazebo 物理引擎，**第一次真正飞起来**
- **MAVLink setpoint receiver**：从 QGroundControl 收 `SET_POSITION_TARGET_LOCAL_NED`
- **STM32H7 固件**：在 `app-copter` 里加 `bin/firmware.rs` with `embassy-stm32`，真 FMU 跑 outer_step
- **外层频率**：现在 outer_step 1 kHz 跑（每个 IMU sample 都过 position 环）。工业级做法：outer 50 Hz, inner 1 kHz。按 sample index 取模即可
- **yaw rate 响应测试**：给 `yaw_rad` 变化，观察 yaw 收敛速度
