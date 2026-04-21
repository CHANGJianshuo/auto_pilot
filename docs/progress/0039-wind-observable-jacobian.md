# 0039 — 风可观测的 F Jacobian 块

**Commit**: `b875322`  **Date**: 2026-04-21  **Milestone**: M4.0

## What / 做了什么

给 F 矩阵加 **drag 相关的 4 个子块修改**，让 EKF 可以**从测量残差学 wind_ne**。这是 M3.4/M3.5 之后的自然收尾。

新增：
- `build_transition_jacobian_with_drag(state, imu, dt_s, drag_over_mass) -> Covariance`
- `predict_step_with_drag` 现在内部用**drag-aware Jacobian**（之前用的是 drag-free 版本 —— 那时 F 和 state propagation 不自洽）
- 2 proptest（共 70 测试）

## Why / 为什么这么做

### 自洽的"predict + Jacobian"

**关键原则**：Kalman 滤波器里，F 必须是 state propagation 的**一阶偏导**。任何失配都会让 P 增长有偏，Kalman gain 偏，过程越走越糟。

M3.4 加了 drag 到 state propagation 的 **v_new = v + (thrust - drag)·dt**。但 F 仍然是 drag-free 的 —— 其 `∂v/∂v = (1 - drag·dt)·I` 应该是 `(1 - drag·dt)·I`，但在 M3.4 之后的 `build_transition_jacobian` 里仍然是 `I`。

本 commit 补齐。之后 F 和 predict 一致，wind 从测量残差驱动。

### 4 个子块的数学

```
a_drag = -drag · (v - wind_world)        wind_world = (wind.x, wind.y, 0)

∂a_drag / ∂v        = -drag · I₃
∂a_drag / ∂wind_ne  = +drag · E   (E = [[1,0],[0,1],[0,0]] 是 3×2)

∂v_new/∂v      = I + (-drag·I)·dt  = 原有 I + (-drag·dt·I) 修正
∂v_new/∂wind_ne = (+drag·E)·dt  = +drag·dt·E
∂p_new/∂v      = I·dt + (-drag·I)·½·dt² = 原有 I·dt + (-drag·½·dt²·I) 修正
∂p_new/∂wind_ne = (+drag·E)·½·dt² = +drag·½·dt²·E
```

E 是 3×2 矩阵，把 wind_ne 的 2 维水平分量注入到 v 的 3 维 NED（z 行全零，因为风没垂直分量）。

### 怎么让 wind 可观测

Kalman 更新：`x̂_new = x̂ + K · (y − h(x̂))`，其中 `K = P·Hᵀ·S⁻¹`。

- `∂v/∂wind_ne ≠ 0` 意味着 **P 里 wind ↔ v 的协方差增长**
- GPS 给 v 的测量 → `y_residual` 非零
- K 把这个 residual 投影回 wind_ne（通过 P 的 wind-v cross-covariance）
- wind_ne 估计**跟着** velocity residual 往正确方向移动

没这一步的 Jacobian，wind_ne 就永远不动（K 的 wind 行是零）。

### 有限差分 proptest 检验

```
perturb state.wind_ne by δw:
  nominal v_new = predict(state, ...)
  perturbed v_new = predict(state + δw, ...)
  measured dv = perturbed − nominal
  predicted dv = F_block_v_wind · δw
  
  |measured - predicted| < 1e-5  for 256 random samples
```

同样对 v 扰动验证 `∂v/∂v` 修正：256 个 (drag, v₀, δv, dt) 组合全过，容差 1e-5。

这是**捕捉符号错误**的黄金标准 —— 如果我写成 `+drag·dt` 而非 `-drag·dt`，proptest 立刻会爆。

### 为什么 `fixed_view_mut` 和 `fill_with_identity + scale_mut`

nalgebra 的 view mutation 比起 `for i in 0..3 { v[(i,i)] = ... }` 更快更干净，还避开 `indexing_slicing = deny` clippy 规则。

```rust
let mut top2 = v_w.fixed_view_mut::<2, 2>(0, 0);
top2.fill_with_identity();  // 置为 I₂
top2.scale_mut(dd);          // 乘标量
```

### 没动 default config 开关

M3.5 把 `drag_over_mass_hz` 默认设为 0.0。本 commit**不**改默认值 —— 继续让 app-copter 悬停在"drag 关闭 + PI 整定"那套已验证的组合上。

让 user 明确 opt-in `drag_over_mass_hz > 0`，这时 **Jacobian 和 predict 同时激活**，filter 内部自洽。

未来步骤：
1. 整定 PI（可能需要放宽 `k_i_vel`，因为现在 EKF 能更准估 v）
2. 端到端 SITL：真风 + drag-aware EKF + wind FF → wind_ne 应该收敛到真值
3. 翻 default 开关

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 70 passed; 0 failed
  + dv_dwind_block_matches_finite_difference
  + dv_dv_drag_correction_matches_finite_difference

$ cargo test --workspace
  所有 crate 的 test result 都是 ok
  sim-hil 9 测试仍过（default drag_over_mass_hz=0 → 新路径不激活）

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **PI 重整**：`drag_over_mass_hz > 0` 时的 `k_i_vel` 调优
- **SITL 风识别测试**：sim 里注入已知风、EKF 初始 wind_ne=0，跑 30 秒看 wind_ne 是否收敛到真值
- **3D wind**：wind_ne 是 2D；加 wind_up/down 状态（drag 3×3）可选
- **wind 过程噪声**：当前 `wind_per_s = 1e-2`；若观测到 wind 收敛慢，需调大
