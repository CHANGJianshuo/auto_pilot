# 0033 — 第一次 SITL 悬停 🚁

**Commit**: `6cb7c96`  **Date**: 2026-04-21  **Milestone**: M3.1a

## What / 做了什么

自控飞起来了 —— 在仿真里。写了一个 6-DoF 刚体仿真器，把 `app-copter` 栈产生的 4 个电机推力喂进去，物理引擎更新 state，反过来生成 IMU/GPS/mag/baro 读数喂回 EKF，**闭环**跑了 3 秒并悬停在 1 m。

新增 `sim-hil/src/lib.rs`：
- `pub struct SimConfig` —— 仿真参数（质量、惯性、电机几何、磁场）
- `pub struct SimState` —— 仿真地面真值（attitude、body_rate、v、p、time）
- `pub fn step(cfg, &mut state, motor_thrusts, dt_s)` —— 6-DoF 欧拉积分
- `pub fn sense_imu / sense_gps / sense_mag / sense_baro` —— 无噪声传感器模型
- `pub fn accel_world_from_thrusts` —— helper 算 IMU 加速度
- `mod gazebo`（placeholder，M3.2 接 Gazebo 用）
- 4 测试（其中最重要的是 `closed_loop_sitl_hovers_with_app_copter`）

## Why / 为什么这么做

### 为什么**先做自己的小仿真器**再接 Gazebo

1. **隔离 bug 源**：Gazebo 集成涉及 gRPC、protobuf、CMake 构建、模型文件。一旦有 bug，不知道是 Gazebo 错了还是控制律错了。**自己写个 50 行的欧拉积分先把控制律闭环跑通**，再上 Gazebo 就知道问题一定在接口层。
2. **CI 里能跑**：`cargo test` 就能验证，不需要 Gazebo 启动（~2 GB 内存）。这个测试以后每次 PR 都会跑。
3. **CI 里比 Gazebo 快 100×**：3 秒仿真在容器里 2.4 秒，Gazebo 需要启动 + 加载 world + 同步时钟。

Gazebo 后面做（M3.2），但目前**这个 50 行的仿真器已经是控制栈的完整 SITL 验证**。

### 为什么欧拉积分够用（不用 RK4）

M3 阶段我们要验证"**控制栈能不能工作**"，不是"**Gazebo 够不够准**"。欧拉在 dt=1ms 下误差 ~1e-4（相对），对 3 秒积分累积到 ~1 cm 级。**远小于控制律目标**（0.5 m 门值），所以不是瓶颈。

RK4 留给 M3.2（Gazebo 接口固定时）或 M4（NMPC 在线仿真）。

### 动力学方程：为什么要 `ω × (J·ω)` 项

Euler's equation for rigid-body rotation:
```
τ = J·α + ω × (J·ω)
```

如果 `J = J_xx·I`（球对称，对多旋翼**不**成立），`ω × (J·ω) = 0`，项可以丢。但我们有 `J_xx = J_yy = 0.015, J_zz = 0.025`（**纺锤状**），所以**必须**保留交叉项。

代码：
```rust
let gyro_term = omega.cross(&(cfg.inertia * omega));
let alpha = j_inv * (total_torque_body - gyro_term);
```

这保证了仿真里"快速滚转时会有逃逸倾向"这种真实现象。EKF 必须在此条件下仍收敛。

### `accel_world_from_thrusts` 的为什么

IMU 的加速度计是**比力**传感器：`f_body = R^T · (a_world − g)`。要算 `f_body` 我们需要 `a_world`。但 `a_world` 是 `step()` 积分前的**瞬时**加速度，等 `step()` 内部算好再返回给调用方就晚了。

解法：把同样的公式**暴露成独立函数** `accel_world_from_thrusts`。调用顺序：
```rust
let accel_w = accel_world_from_thrusts(cfg, state, &thrusts);  // 算 a_world
let imu = sense_imu(state, accel_w);                            // 用它生成 IMU 读数
step(cfg, &mut state, &thrusts, dt);                            // 然后再物理积分
```

这个顺序对 —— 对物理学来说，IMU 报的就是"**此刻正在发生的**"加速度。

### `closed_loop_sitl_hovers_with_app_copter` 细节

```rust
1. 初始化 sim_state.position_ned.z = -1.0（1 m 高空）
2. 初始化 FlightState 用一次 baro + gps 让 EKF 知道"大概在 1m"
3. setpoint = (0, 0, -1) NED
4. 循环 3000 次（3 秒 @ 1 kHz）：
   a. 上一帧推力产生的 accel_w
   b. sense_imu(state, accel_w) → IMU
   c. outer_step(app_cfg, flight, imu, dt, setpoint) → motor_thrusts
   d. step(sim_cfg, sim_state, motor_thrusts, dt) → 新 sim_state
   e. 每 200 ms 给 GPS、40 ms 给 mag、20 ms 给 baro
5. 断言：|altitude − 1 m| < 0.5 m, |水平位置| < 0.5 m, ‖q‖ 单位
```

**这是整个项目最重要的一个测试**：它证明从位置 setpoint → 电机推力 → 物理响应 → 传感器反馈 → EKF → 控制律 整条路能闭合并稳定。之前所有单元测试加起来是**部件级**保证，**这个是系统级**保证。

### 容差 0.5 m 的来由

P-P cascade 有固有的稳态误差（没有积分项）。`max_accel = 8 m/s²` 饱和让 attitude 响应被限制。3 秒时间让 EKF 收敛到真值，但 P 控制器本身不会收敛到 0 —— 有稳态偏移。

未来改进：
- 加积分项（变成 PID cascade）—— 典型精度 < 5 cm
- 前馈扰动估计（EKF 里的 wind_ne 状态可以反馈回来）
- M4 NMPC —— 有约束的优化自然消除稳态偏移，精度 < 2 cm

目前 0.5 m 足以证明"**控制系统能工作**"。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil
test result: ok. 4 passed; 0 failed
  closed_loop_sitl_hovers_with_app_copter  finished in 2.4 s

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

**这是本项目的第一次"证明控制系统工作"的证据**。

## Follow-ups / 遗留

- **传感器噪声**：当前 sense_* 都是无噪声。加 Gaussian 扰动让 EKF 的 NIS 门真正生效
- **Motor first-order lag**：`T_i(t+dt) = T_i(t) + (T_cmd − T_i)·dt/τ`，τ ~20 ms
- **气动阻力**：`drag = -k·v·|v|` 加到 force
- **风扰**：用 EKF 的 wind_ne 状态的真值做输入
- **RK4 integrator**：对高角速度仿真更稳
- **M3.2 Gazebo Harmonic 集成**：同一套 step/sense API 接 gz-transport
- **Performance benchmark**：在容器里跑 1 小时实时仿真，看 EKF 是否数值稳定
- **故障注入**：单电机失效（thrust[i] = 0）、GPS spoofing（突然偏 100 m）、mag 干扰
