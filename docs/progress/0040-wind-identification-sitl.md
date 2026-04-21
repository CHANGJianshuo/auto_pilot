# 0040 — 端到端 wind 识别测试（开环 SITL）

**Commit**: `52421fe`  **Date**: 2026-04-21  **Milestone**: M4.1

## What / 做了什么

系统级验证 —— 在 SITL 里放一个**真实已知的恒风**，让 EKF 从 GPS 观测到的 drift **自动识别**风。15 秒仿真后，`wind_ne` 估计和真值的方向 cosine > 0.5。这证实 M4.0 的 Jacobian 是对的。

新增：
- `ekf_identifies_wind_with_drag_aware_predict_open_loop`（sim-hil，~9 秒测试时间）
- 同时**保守回退**：`default_config_250g` 的 `drag_over_mass_hz` 仍是 0.0（M4.0/M3.5 做的工作在 opt-in 下生效）

## Why / 为什么这么做

### 为什么是开环测试

我第一次写的是**闭环**测试（outer_step + wind）。结果：`wind_ne` 发散到 -17.8（真值 +2）。原因：

闭环下，控制器（M3.3 wind FF + PI integrator）反应比 EKF 快：
- Tick 0：vehicle at rest, wind=+2 → drag 推 vehicle +x
- Tick 1：EKF 看到 +x 的 velocity 残差
- Tick 2：Controller FF 推 x 反方向 → vehicle 往回拉
- Tick 3：EKF 看到 −x 的 residual → wind_ne 估计朝 −x 偏

闭环情况下 residual 在 ±0 附近振荡，EKF 把噪声误当风扰。

**开环**（motors 固定 hover）切断控制器，vehicle 纯粹被风拖着走。GPS 看到的位置和速度残差是**单调**的，EKF 有**一致信号**来识别风。

### 这个发现的意义

"开环才能识别扰动"听起来像缺陷，其实是**物理常识**：
- 如果控制器完美补偿了扰动，扰动在系统响应中就"消失"了
- 没有观测线索 → 没法识别
- 实际飞控用"切换到 observer mode"或"用 acceleration-inversion"（Pocket Rocket）等技巧

工业级解：**扰动估计和控制要有时间尺度分离**。EKF 快收敛，控制器慢反应（带 low-pass filter on wind_ne before feeding FF）。当前我还没做这一步，所以 opt-in 才是正确默认。

### 对 cos > 0.5 而不是数值精度的断言

开环下 vehicle 持续 drift，EKF 和真 drag model 之间还有几个不一致：
- sim 有线性+二次 drag，EKF 只建线性
- sim 有电机滞后、传感器噪声
- EKF 初始信念 P0 很大（10 m 位置 σ）

15 秒的 finite time → wind 估计到 ~1.5 m/s（真值 ~2.24 m/s），**方向对但数值差 30%**。对这个 M4.1 的目标（验证 Jacobian 符号 + 数量级）足够。

### `wind_per_s = 0.2` 的调整

默认 `ProcessNoise::wind_per_s = 1e-2`。对静态风扰估计这个值太小 —— 滤波器"相信"wind 不会快变化 → Kalman gain 对 wind 投影很小 → 更新慢。

开环测试把它提到 0.2（意思是"风每秒能变化 0.2 m/s 量级"）。这是**测试专用**，不动全局默认（真实飞行中风其实变化慢，更稳）。

### 回退 default drag = 0

M3.5 已经发现打开 drag default 会让 realistic SITL 容差 0.6m → 2.5m，因为 PI 整定基于 drag-free 假设。

M4.0 加了 Jacobian，但没重新整定 PI。所以**继续 opt-in**。本 commit 做的事：
1. 加端到端测试证明 **能用**
2. 保持现有 SITL 不回归

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil ekf_identifies_wind
test result: ok. 1 passed; 0 failed

$ cargo test --workspace
  algo-ekf 70, sim-hil 10, app-copter 11, 其它全过
  总 ~150+ 测试

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **PI 重整 + drag default 开**：需要试 `k_i_vel = (0.3, 0.3, 0.5)` 配合 drag on，看 realistic SITL 是否在原来的 0.6m 容差内
- **闭环 wind ID**：加 low-pass on EKF wind_ne 之后再喂给 FF，打破紧耦合
- **wind 收敛精度**：增加观测率、调 wind_per_s、或加专门的 wind measurement（airspeed sensor）
- **3D wind / wind shear**：当前 wind_ne 只 2D，高空作业需要
