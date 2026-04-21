# 0037 — EKF predict 支持气动阻力（wind → predict）

**Commit**: `a39e2ad`  **Date**: 2026-04-21  **Milestone**: M3.4

## What / 做了什么

给 EKF 的 predict step 加上气动阻力项。当 `state.wind_ne` 被写入时，predict 出的速度会正确反映"风拉着飞机"的物理。

新增：
- `State::predict_with_drag(imu, dt_s, drag_over_mass) -> State`
- `State::predict` 现在委托到 `predict_with_drag(..., 0.0)`
- `predict_step_with_drag(...)` 顶层函数
- 2 proptest（`algo-ekf` 共 68 测试）

## Why / 为什么这么做

### 跟 M3.3 的联系

**M3.3** 做的：控制器把 `state.wind_ne` 用作 **feed-forward**，让油门提前对抗风。
**M3.4** 做的：滤波器把 `state.wind_ne` 用作 **predict 依据**，让估计的速度也反映风对车的推力。

两者在**飞行系统角度是一对**：控制器算补偿、估计器算预测。之前估计器对 wind_ne 视而不见，现在不再如此。

### 数学

```
drag_accel = −drag_over_mass · (v − wind_world)
wind_world = (wind_ne.x, wind_ne.y, 0)   # horizontal only
accel_ned = R(q)·f + g + drag_accel
```

Drag 对速度的影响是**负反馈**（离开稳态越远，回撤越猛），对 wind 的影响是**正向拖动**（vehicle 跟着风走）。

### 为什么**不**做 F Jacobian 的更新

要让 EKF 通过测量残差**学习** wind_ne，需要：
- `∂v/∂v` 里添加 `−drag·dt` 项（速度自衰减）
- `∂v/∂wind_ne` 里添加 `+drag·dt` 块（风影响速度）

这是 **24 维 F 矩阵的 2 个新子块**，加起来 ~20 行代码 + 需要做**有限差分 proptest** 验证，同时要改 Q 噪声让 wind_ne 有非零过程噪声（否则 Kalman gain 永远是零）。

工作量约 0.5-1 天，**不在今天这小步的范围内**。我明确 queue 到 M4+ 作为 follow-up。

**本 commit 的价值**：
1. 如果外部源（pitot-static airspeed、外部 windsock、手动覆盖）告诉 EKF 风是什么，predict 能用了
2. 下次加 Jacobian 块时，predict 代码已经对齐，不用动主流程
3. SITL 里可以**手动注入** wind 到 `flight.state.wind_ne` 验证整个系统：EKF 自恰 + 控制器 FF

### 两个 proptest 的结构

```
预测方程:   v_new = v_old + drag · (wind − v_old) · dt

1. zero wind / moving vehicle:
   v_new = v_old − drag · v_old · dt = v_old · (1 − drag · dt)
   
2. non-zero wind / stationary vehicle:
   v_new = 0 + drag · wind · dt = drag · wind · dt
```

256 随机 (drag, v₀ / wind, dt) 样本验证两个极限。

### 约束 `drag_over_mass > 0`

如果 `drag_over_mass = 0` 直接返回 zero drag_accel（走老路径，0 开销）。负数或 NaN 也走老路径（**不处理错误输入**，与其他 predict 异常路径一致）。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf predict_with_drag
running 2 tests
test tests::predict_with_drag_decelerates_when_wind_zero ... ok
test tests::predict_with_drag_and_wind_pushes_stationary_vehicle ... ok

$ cargo test -p algo-ekf
test result: ok. 68 passed; 0 failed   ← 所有原有 66 + 2 新

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M4**：wind 可观测性 — 在 `build_transition_jacobian` 里加：
  - `F[v_i, v_i] += -drag · dt`（速度自衰减）
  - `F[v_x, wind_ne.x] = drag · dt`
  - `F[v_y, wind_ne.y] = drag · dt`
  - 以及对应的 process noise 调整 (`wind_per_s` 从 1e-2 可能要提高)
- **SITL 测试**：Gazebo 风扰 + EKF 用 predict_step_with_drag，观察 wind_ne 能否收敛到真值（需 Jacobian 先上）
- **app-copter**：`RateLoopConfig` 加 `drag_over_mass` 字段，outer_step 用 `predict_step_with_drag` 而非 `predict_step`（下一步做）
- **3D wind**：当前 wind_ne 只 2D。垂直风扰（下洗流、热气流）作为未来工作
