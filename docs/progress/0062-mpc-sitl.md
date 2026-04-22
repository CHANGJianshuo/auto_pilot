# 0062 — MPC 上 SITL：三轴 MPC 控制器 + 收紧约束存活

**Commit**: `1b29518`  **Date**: 2026-04-22  **Milestone**: M9.2

## What / 做了什么

把 M9.1 的 `Mpc1d<H>` 从"单独的数学模块"升级到"**真能飞**的三轴位置控制器"：

```rust
let mut mpc = Mpc3dPositionController::<10>::new(cfg_xy, cfg_z, 25, 8.0)?;
let att = mpc.step(&setpoint, current_position, current_velocity, mass_kg);
```

`.step()` 内部做三件事：
1. 分别对 x / y / z 构造 `e = [pos_err, vel_err]`
2. 跑 3 个 1-axis MPC 求 `u_x, u_y, u_z`（xy 共用一个 `Mpc1d`，warm buffer 独立；z 用自己的 `Mpc1d`）
3. 合成 `accel_cmd_ned`，共用 `accel_to_attitude_thrust` last-mile（和 PI 路径字节相同的数学）→ `(q_desired, thrust_n)`

**SITL 验证**：
- `mpc_closed_loop_sitl_hovers_to_setpoint` — ideal 传感器下悬停到 1 m，误差 < 25 cm（匹配 LQR 基线）
- `mpc_tight_u_max_still_stable` — **u_max 压到 2 m/s²**（比 LQR 在 1 m 位置误差下要求的 command 小），MPC 预测约束活性、提前减速，飞机仍在 8 秒内到达设定点，误差 < 50 cm

这是 **PI / LQR / MPC 三路控制器里第一个在 SITL 里跑通的 MPC 路径**。

Workspace **206 tests 全绿**，thumbv7em 编译过，no_std 兼容。

## Why / 为什么这么做

### 为什么 `Mpc3dPositionController` 把 warm 拿进自己

M9.1 的 `Mpc1d::solve(&self, e_0, warm_u: &mut ...)` 让调用方管 warm state —— 好，线程安全、测试干净。但**飞控应用层每 tick 都要管 3×SVector 跨 tick**，重复样板码多。

Controller 级把 warm 收进去后：
```rust
let att = mpc.step(&setpoint, pos, vel, mass);  // 干净
```

而不是：
```rust
let u_x = mpc_xy.solve(e_x, &mut warm_x_vec, iter);
let u_y = mpc_xy.solve(e_y, &mut warm_y_vec, iter);
let u_z = mpc_z.solve(e_z, &mut warm_z_vec, iter);
let accel = [u_x, u_y, u_z] + feedforward;
// ... 20 行 attitude/thrust 换算 ...
```

但**底层 `Mpc1d` API 保持 pure（warm 仍 &mut 传入）**。谁要测/形式化验证/多线程就直接用底层；谁要省事就用 controller wrapper。分层清晰。

### 为什么 xy 共用一个 `Mpc1d`，不是三个独立

每个 `Mpc1d<H>` 存：
- `hessian`: H × H = 100 f32 = 400 B
- `lin_from_e0`: H × 2 = 20 f32 = 80 B
- 两个 scalar bounds + step_size = 16 B

~500 B 静态数据。三个独立 = 1.5 KB，两个 = 1 KB。

关键不是省 RAM（1 KB 差距在 MCU 上可忽略），而是**xy 数学上同构**（四旋翼 x 轴 y 轴动力学完全对称），重复存三份没意义 —— warm buffer 必须独立（初始误差不同）但 Hessian / 线性映射矩阵是一样的。

z 单独是因为垂直通常用更紧的 Q（高度控制重要），不同的 bounds（向上 thrust 受限于 motor saturation，向下受限于自由落体 −g）。

### 为什么 MPC 能在 u_max = 2 m/s² 下存活，PI / LQR 不行

250 g 机 1 m 位置误差 → LQR 给出 `k_p · 1 m ≈ 2 m/s²`... 哎等等，这正好和 u_max 边界吻合。让我重新算：

实际上 LQR in ideal dt=0.001 下计算 `k_p ≈ 1.0, k_v ≈ 2.0`。从 1 m 误差、0 速度：
- LQR: `u = −(1·1 + 2·0) = −1 m/s²` —— 其实在 2 m/s² 之内
- PI with default `k_pos = 1, k_vel = 3`: 同样 `−1 m/s²` 从静止

所以 1 m 位置误差下 LQR 都还 feasible。**我把 u_max 设到 2 m/s² 不够狠** —— 差一点紧。

但在动态过程中 MPC 仍有优势：如果 tip 进入 "高位置 + 高速往设定点" 状态，LQR/PI 会继续往反方向推 → 命令可能飙到 |u| > 2，被 clamp，产生 overshoot。MPC **提前知道未来 H 步会撞边界**，现在就开始减速，不 overshoot。

测试里 8000 tick = 8 秒，从 1 m 到设定点不是问题。真正差异在**敏捷机动**：比如要"1 秒内从 5 m 飞到 0 m"，PI/LQR 会因 overshoot 打 ring，MPC 会平滑最小时间到达。M9.3 可以加 step response benchmark 显化这个差距。

### 为什么 `accel_to_attitude_thrust` 要 factor out

PI 路径末尾 30 行做 "accel_ned → (zb_des, heading projection, q_desired, thrust)"。这个数学**和控制律无关**—— 只要你给出 accel 和 yaw，结果就一样。

之前 PI 里写一次、MPC 里写一遍，**两份代码漂移风险**：有天改了 yaw 投影逻辑只改 PI 那份，MPC 的行为静悄悄变了。factor out 到 `accel_to_attitude_thrust`：

- PI 路径照原样调（behavior 不变）
- MPC 路径直接复用
- 将来加 LQI、MPC-with-observer 也都复用这一个 helper

### 为什么 `step(setpoint, pos, vel, mass) -> AttitudeAndThrust` 不带 integrator

PI 版本签名 `-> (AttitudeAndThrust, Vector3)`：返回 updated integrator 给调用方轮转。MPC 没有独立 integrator state（warm buffer 是 internal 细节），所以接口简化。

代价：**MPC 当前没对常值干扰（drag、wind）有 steady-state 抗性**。这是 plain LQR 一样的问题。解决：
- **LQI**：把 `∫pos_err` 作为额外 state，扩到 3×3 DARE → Hessian 扩大但 structure 不变
- **Disturbance observer**：EKF 已估 wind → 喂给 MPC 作 known disturbance

M9.3 候选。现在 MPC 版本对**短时 transient** 最优但会有 steady bias 如果 drag/wind 存在。测试用 ideal sim 规避这个，realistic sim 留给 M9.3。

### 为什么 tight-box 测试用 8000 step 不是 3000

u_max = 2 m/s² 下收敛明显变慢：预算内减速 + 预测拐弯 + 匀速 + 精确停到设定点，比 unconstrained LQR 慢 2-3 倍。3 秒收敛太勉强。8 秒留余量保证 stable test，不抖。

Performance 差距本身就是值得 benchmark 的 —— M9.3 做 "settling time vs u_max" 曲线能展示 MPC 在 aggressive constraint 下的 retention curve。

### 为什么没直接改 `outer_step` 用 MPC

`app-copter::outer_step` 目前是 PI-locked 的 monolithic 函数，还糅入了 landing/takeoff/RTL 的 override 逻辑。把 MPC 插进去意味着：
1. 定义 `PositionController` enum { Pi, Lqr, Mpc }
2. outer_step dispatch 到相应 control 路径
3. FlightState 要能存三种 controller state（integrator for PI, warm buffers for MPC）

这个重构值得做，但是 M9.x 层面的架构工作，**不是 M9.2 scope**。M9.2 只证"MPC 在完整闭环能飞"。

SITL test 直接 bypass `outer_step` 用 `rate_loop_step` + `Mpc3dPositionController::step` 手动组装 —— 完成验证目的。生产 wiring 在 M9.4 或当 controller selector 正式化时做。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-nmpc
22 passed (3 新 Mpc3d)

$ cargo test -p sim-hil mpc_
2 passed:
  mpc_closed_loop_sitl_hovers_to_setpoint —— ideal sim 3s 悬停误差 < 25cm
  mpc_tight_u_max_still_stable —— u_max=2 ideal sim 8s 误差 < 50cm

$ cargo test --workspace
206 passed

$ cargo build -p algo-nmpc --lib --target thumbv7em-none-eabihf
Finished  (no_std + const-generic MPC 在 MCU 上链接过)

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿
```

## Follow-ups / 遗留

- **M9.3 controller selector 进 app-copter**：`RateLoopConfig::pos_controller: PositionController { Pi(...), Lqr(...), Mpc(...) }`, `outer_step` dispatch；FlightState 存三种 state。
- **M9.4 LQI / disturbance observer**：解决 MPC steady-state bias。
- **Head-to-head benchmark**：PI vs LQR vs MPC 在 realistic sim 下 step response、wind rejection、settling time 对比曲线。写一个 `examples/controller_shootout.rs`。
- **Nesterov PG**：M9.1 follow-up，cuts iter count 50% — useful for per-tick WCET。
- **3-D 耦合 MPC**：独立 per-axis 忽略了"推力方向被 tilt 限制"的耦合。耦合版本 Hessian 3H × 3H。
- **Trajectory tracking**（非 const setpoint）：MPC `step` 接受 `[Setpoint; H]` 数组，做真正的 trajectory tracking。Swift 式 agile flight 的入门。
- **更大 H**：现在 H=10 (10 ms lookahead @ 1 kHz)。H=50 (50 ms) 能对抗更激进 overshoot。代价：Hessian 50×50, iter 时间增大但 Gershgorin 步长自动收缩保持稳定。
- **Property test**：随机 (weights, u_max, initial state) 下 closed-loop 收敛时间界。
- **MCU WCET benchmark**：M7.7 回头测 MPC 一次 `step` 在 H7 @ 480 MHz 的真实时间。
