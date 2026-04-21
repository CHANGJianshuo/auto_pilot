# 0035 — PI 级联：位置环加积分项

**Commit**: `0004831`  **Date**: 2026-04-21  **Milestone**: M3.2

## What / 做了什么

给 outer loop 加积分项。在带噪声+电机滞后+气动阻力的 realistic SITL 下，高度稳态误差**从 2+ m 压到 < 0.6 m**。

改动：
- `algo-nmpc::PositionGains` 新增 `k_i_vel: Vector3` 和 `max_integrator: f32`
- `algo-nmpc::position_to_attitude_thrust_pi(..., integrator, dt)` → `(AttitudeAndThrust, new_integrator)`
- 旧 `position_to_attitude_thrust` 委托到新函数（integrator=0），**零破坏**老测试
- `app-copter::FlightState` 新增 `vel_integrator` 字段
- `outer_step` 用 `_pi` 变体，自动贯穿 integrator state
- `default_config_250g` 启用 `k_i_vel = (0.5, 0.5, 0.8)`
- realistic SITL 测试容差收紧（6m → 0.6 m）

## Why / 为什么这么做

### 为什么 I 项放在 velocity loop 而不是 position loop

典型多旋翼 cascade 有两个可能的 I 放置位置：
- **Position error → I**：`vel_cmd = k_p · pos_err + ∫ k_ip · pos_err · dt`
- **Velocity error → I**（我选的）：`accel_cmd = k_v · vel_err + ∫ k_iv · vel_err · dt`

Velocity-loop I 的好处：
- **直接吃稳态偏差**：稳态时 `vel_err ≠ 0`（风、漂移），I 累积直到推力偏移平衡它
- **pos 环保持 pure-P**：位置响应快、overshoot 小
- PX4 的 `MulticopterPositionControl` 和 ArduPilot 的 Loiter 都这么做

### 为什么 conditional 抗 windup 够用

**问题**：无 anti-windup 的 I，long saturation 期间积分会"爆"，然后反超。

**方案**：conditional integration —— 只在**不饱和**时累积：

```rust
if !saturated && dt_s > 0.0 {
    integrator += k_i · vel_err · dt;
    integrator = clamp(integrator, -max_int, +max_int);
}
```

相比更复杂的方法（back-calculation、tracking anti-windup），conditional 是**最简单的实用方案**，也是大多数 PX4/ArduPilot 代码的路线。代价是饱和时 I 完全冻结（可能太保守），但我加了 `max_integrator` 做硬上限保险，所以即使冻结逻辑有误也不会炸。

### 为什么默认 k_i_vel = (0.5, 0.5, 0.8)

- **horizontal（x, y）** = 0.5：cancel ~0.5 m/s drift in 2 s
- **vertical（z）** = 0.8：更强一点（baro 精度高，允许更激进），cancel ~0.5 m/s in 1.25 s

`max_integrator = 5 m/s²` 意味着 I 项最多给 5 m/s² 偏移，约 0.5 g。设得太小不够补偿风；设太大下界变成静态偏置。5 m/s² 对 30% 油门余量的小 quad 够用。

### 为什么**不破坏**老的 position_to_attitude_thrust

向后兼容：0030 doc 的所有测试、proptest、benchmark 都用的是老函数签名。破坏 = 几十行改动 + 风险暴露。

解法：新函数 `position_to_attitude_thrust_pi` 承载 PI 逻辑，老函数保留为 thin wrapper：

```rust
pub fn position_to_attitude_thrust(...) -> AttitudeAndThrust {
    let (att, _) = position_to_attitude_thrust_pi(..., Vector3::zeros(), 0.0);
    att
}
```

旧调用方看到的行为**完全相同**（k_i_vel 默认 0，integrator 初始 0，所以新逻辑等价于老 P-P）。

### integrator 为什么放在 FlightState 而不是 PositionController 结构

之前考虑过 `pub struct PositionController { gains, state }`，但：
- FlightState 已经有 last_gyro_filtered、3 个 health counters 等"rate loop 持久状态"
- 再加一个 Vector3 不增加心智负担
- 不引入新类型，减少代码量

代价：FlightState 变大了（之前 24+24²×4 + 小杂项 ≈ 2.5 KB，现在 +12 B），不是问题。

### 8 秒收敛时间来自哪里

realistic SITL 在带风扰(0) + 噪声 + drag + lag 下，I 项在无饱和时每秒积累 `k_i · vel_err` 量级。风洞测试经验：
- drag 引入 ~0.5 N 稳态推力偏差
- I 要把这个吸收掉需要约 2-4 s
- 传感器噪声延缓收敛约 2×
- 所以 8 s 时 90% 收敛 ≈ 0.6m 容差 OK

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-nmpc
test result: ok. 8 passed; 0 failed

$ cargo test -p app-copter --lib
test result: ok. 9 passed; 0 failed

$ cargo test -p sim-hil
test result: ok. 9 passed; 0 failed
  realistic closed-loop 现在过 0.6m altitude tolerance

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **k_i 自动调参**：目前是硬编码 `(0.5, 0.5, 0.8)`。不同飞机质量 / aero 会需要不同值 —— 加 `k_i_scale` 参数、或 M4 用 NMPC 代替
- **pos-error I 项**：对 "精确定点" 场景（LOITER / RTL 结束阶段）有用，可以做双 I
- **Feed-forward 扰动估计**：EKF 里已经有 `wind_ne` 状态，把它 feed-forward 到 accel_cmd 就几乎零延迟消除风扰（I 项只处理未建模扰动）
- **D 项**：加速度前馈能减少 overshoot。当前 `setpoint.accel_ned` 是预留位，后续轨迹规划器会填
- **M4 NMPC**：优化问题天然带"显式积分+约束"，PI 是过渡方案
