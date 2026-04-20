# 0030 — 位置环（P-P 级联基线）

**Commit**: `851a79c`  **Date**: 2026-04-21  **Milestone**: M2.5

## What / 做了什么

控制栈**最外层**：输入**期望位置**，输出 `(q_desired, thrust_n)` 给姿态环消费。

新增于 `algo-nmpc` crate（命名暗示将来 M4 会被 NMPC 替换）：
- `pub const GRAVITY_M_S2 = 9.80665`
- `pub struct Setpoint { position_ned, velocity_ned, accel_ned, yaw_rad }`
- `pub struct PositionGains { k_pos, k_vel, max_accel } + Default`
- `pub struct AttitudeAndThrust { q_desired, thrust_n }`
- `pub fn position_to_attitude_thrust(setpoint, pos, vel, mass, &gains) -> AttitudeAndThrust`
- 5 单元 + 1 proptest（共 6）

## Why / 为什么这么做

### 数学流水

```
1. 位置误差 → 速度命令
   vel_cmd = k_pos ⊙ (p_sp − p) + v_sp

2. 速度误差 → 加速度命令
   accel_cmd = k_vel ⊙ (vel_cmd − v) + a_sp

3. 加速度饱和（防姿态过大）
   if ‖accel_cmd‖ > max_accel: scale down

4. NED 力平衡
   thrust_vec_ned = m · (accel_cmd − g_vec)    # g_vec = (0, 0, +9.80665)
   thrust_mag     = ‖thrust_vec_ned‖

5. 期望 body z 方向 = 期望推力方向的反向（推力沿 −body_z）
   zb_desired = −thrust_vec_ned / thrust_mag

6. 期望 body x 方向 = 期望航向投影到 zb_desired 的正交平面
   heading = (cos yaw, sin yaw, 0)
   xb_desired = heading − (heading · zb) · zb   # Gram-Schmidt
   normalize

7. body y = zb × xb

8. R_desired 是 [xb, yb, zb] 列拼的旋转矩阵 → q_desired
```

### 为什么"推力向量法"而不是直接调姿态 + 油门

**推力向量法**（thrust vector parameterisation）是 PX4 ekf2、Agilicious、Lee-Leok 2010 论文的标准做法：

1. **角度奇异性天然消除** —— roll/pitch 通过向量给出，不会在 ±90° 边界翻转
2. **Yaw 独立** —— 航向和倾斜完全解耦（通过 Gram-Schmidt）
3. **直接对应物理** —— "推哪个方向、推多少" 是物理量，姿态只是实现手段

这是为什么所有现代多旋翼控制器都用这个范式。传统的 "roll_cmd/pitch_cmd/yaw_rate/thrust" 接口（mavlink `ATTITUDE_TARGET` 的默认）已经是过时的。

### NED 符号约定

我踩了一个坑：**+pitch = nose UP**（标准航空），所以要前飞，需要**nose DOWN** = 负 pitch = `q.j < 0`。第一次测试我写成 `q.j > 0` 失败了。这个约定全栈必须一致：PX4/ArduPilot/NED wiki 都是这么约定的。

### 饱和的哲学

`max_accel = 8 m/s²` 的意义：最大水平加速 ≈ 0.8 g，对应最大倾斜角 ≈ `atan(0.8) ≈ 38°`。超过这个就拒绝激进指令。

这是**软约束**而不是硬约束：没达到就直接用；达到就等比缩放（保方向、降幅度）。更硬的方法（QP）留给 M4 NMPC。

### 为什么 `algo-nmpc` 而不是 `algo-position`

我一开始纠结了。最后放在 `algo-nmpc`：

- 这个 crate 的语义是 "outer loop"，不是 "NMPC specifically"
- M4 会加**真 NMPC**，保留接口 `Setpoint → AttitudeAndThrust` 不变
- 不额外拆 crate，减少依赖图复杂度

代价是 crate 名字可能混淆，用 doc 注释澄清。

### 256-sample proptest

```
output_is_always_valid:
  随机 (p_sp, v, yaw) 在 ±20 m / ±10 m/s / ±π
  对任意输入：‖q_desired‖ ∈ [1−1e-3, 1+1e-3]
            thrust_n 有限 ≥ 0
```

一个 256 样本的总安全检查。任何极端输入（远距离、NaN 边缘、yaw 绕圈）都不会让控制器产生**非法** attitude / thrust。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-nmpc
test result: ok. 6 passed; 0 failed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **接入 app-copter**：在 rate_loop_step 前面加一层 outer_step：`position_setpoint → (q_desired, thrust) → rate_cmd → rate_loop_step`
- **setpoint generator**：从 MAVLink setpoint / waypoint 生成 Setpoint 的模块
- **饱和重分配**：当饱和时优先保姿态角（M2.1b QP 允许这样）
- **M4 NMPC**：10-20 步 horizon QP，替换本 baseline
- **Integral term**：当前是 P-P 无 I，有风扰时会有稳态误差。M3 加 feed-forward 扰动估计（wind_ne 状态在 EKF 里已经有了！）
- **Differential flatness**：轨迹跟踪更准确的方法；Agilicious 同时支持 DFBC 和 MPC，后续可加作为第三种 outer controller
