# 0038 — app-copter 接 drag-aware predict（opt-in）

**Commit**: `51708f9`  **Date**: 2026-04-21  **Milestone**: M3.5

## What / 做了什么

把 M3.4 写的 `predict_step_with_drag` 接进 `app-copter::rate_loop_step`，通过 `RateLoopConfig::drag_over_mass_hz` 开关。**默认关闭**（= 0），避免回归 PI 整定。

新增：
- `RateLoopConfig::drag_over_mass_hz: f32`（默认 0.0）
- `rate_loop_step` 改调 `predict_step_with_drag(..., cfg.drag_over_mass_hz)`
- 1 测试（共 11）：当 wind_ne 注入且 drag=0.4 时，预测速度出现差异

## Why / 为什么这么做

### 为什么 default = 0

试图默认 0.2 → **realistic SITL 回归**（altitude err 0.6 m → 3 m）。

原因分析：
- M3.4 加了 drag 到**预测**中，但 F 矩阵没改
- PI 整定（M3.2）基于**老预测**（无 drag）做的
- 新预测让 EKF 估计的 v 更"真实"，但 PI 对真实响应过冲
- 一致性问题：打开 predict drag 必须**同步**更新 Jacobian，否则 filter 内部不自洽

正确顺序：
1. ✅ M3.4：predict 加 drag（代码）
2. ❌ M4（未做）：F 矩阵加 ∂v/∂wind_ne，drag 作为 velocity auto-coupling
3. ✅ M3.5（本步）：接口就位、默认关
4. 未来：启用 default drag，一并重调 PI

### 用户怎么用

```rust
let mut cfg = default_config_250g();  // drag off
cfg.drag_over_mass_hz = 0.2;          // 手动开
```

或者 app 对接外部风速计时，写 wind_ne 到 `flight.state.wind_ne`，并开 drag —— 此时 predict 会用风修正速度。M3.3 的 FF 仍然工作在控制器侧。

### 为什么不滚退 M3.4

M3.4 本身是对的——**新增**一个"drag-aware"路径。问题在 M3.5 默认把它打开。把默认 flip 成 off 即可。

### 新测试的构造

```rust
两个 FlightState 带同样 wind_ne=(2,0)
跑 100 步 stationary IMU
drag_on 的 cfg.drag_over_mass_hz = 0.4
drag_off 的 cfg.drag_over_mass_hz = 0.0

比较 velocity_ned.x 差异 > 0.01 m/s
```

如果 `predict_step_with_drag` 没真正被调用或 wind 没流到 predict，这个 test 会失败。

### 整个 wind 系统的现状

| 组件 | 状态 |
|---|---|
| `State.wind_ne` 字段 | ✅ 存在 |
| `build_process_noise` 给 wind_ne 加噪声 | ✅（`wind_per_s`） |
| 控制器 `outer_step` 用 wind_ne 做 FF | ✅ M3.3 |
| EKF predict 用 wind_ne 计算 drag | ✅ M3.4（opt-in M3.5） |
| EKF 从测量残差**学习** wind_ne | ❌ M4 |

基础设施已经构建完毕，**缺最后一块 M4 Jacobian** 就能让 wind 自动识别。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib
test result: ok. 11 passed; 0 failed
  + rate_loop_drag_influences_predicted_velocity_when_wind_set

$ cargo test -p sim-hil
test result: ok. 9 passed; 0 failed
  realistic SITL 和 wind SITL 回到 M3.2 的容差（drag 被默认关闭）

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M4 核心任务**：`build_transition_jacobian` 加 ∂v/∂v += -drag·dt·I₃ 和 ∂v/∂wind_ne = +drag·dt（3×2 非零块）
- **调 PI 增益**：在 Jacobian 完成后，重调 `k_i_vel` 和 `wind_per_s` 让 wind 可观测 + drag 下 SITL 仍收敛
- **End-to-end test**：SITL 给恒定风 → EKF 自己识别 wind_ne ≈ 真值 → 控制器 FF + predict drag 联动，稳态误差 < 10 cm
- **3D wind**：垂直风扰（升降气流、下洗流）——当前 wind_ne 只 2D
