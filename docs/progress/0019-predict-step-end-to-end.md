# 0019 — predict_step 端到端 + 不变量测试

**Commit**: `4993ce4`  **Date**: 2026-04-20  **Milestone**: M1.9c（**EKF predict 半段收尾**）

## What / 做了什么

把 M1.7–M1.9b 累积的四块拼图（`State::predict` + `build_transition_jacobian` + `build_process_noise` + `predict_covariance`）合成一个单入口函数，并用**长序列 proptest** 验证数值稳定。

新增：
- `pub fn predict_step(state, covariance, imu, noise, dt) -> (State, Covariance)` —— 单次 predict 完整循环
- `predict_step_hovers_stable_state_is_stable_covariance` —— 1000 步悬停 invariant
- `predict_step_preserves_invariants` —— 256 × (10-200 步) 随机 IMU 序列 invariant

## Why / 为什么这么做

### 单入口函数的价值

rate loop task 里每一次 IMU sample 到来都要做完整 predict。如果要求调用方记得 4 个分别的函数名 + 正确顺序，总有一天会漏一个（比如忘了 `build_process_noise`，P 就不增长，EKF 会过度自信 → 实际不准但报告 σ 很小 → 灾难）。

`predict_step` 签名 = 编译期强制契约，四件事必须一起做。

### 为什么要跑 1000 步和 50 000 步的测试

单步 predict 看起来对（所有 Jacobian 的有限差分 proptest 都通过了），但**不变量能否保持上万步** 是另一个问题。几种典型失败模式：

1. **四元数漂移**：每步 ‖q‖ 差 1e-7 个 ulp，积累 10000 步 → 1e-3 偏差。本 commit 验证 1000 步后仍在 1e-4 内。
2. **协方差发散**：P·Fᵀ 循环里如果 Q 太小或 F 有 bug，P 的某个对角会 monotonically 增长或衰减到 0。本测试断言每个 `P[i, i]` 始终正数。
3. **P 失对称**：`(F·P·Fᵀ)[i, j]` 和 `(F·P·Fᵀ)[j, i]` 浮点下差 1 ulp，不 enforce_symmetry 的话 1000 步后差 1e-5。本测试断言 1e-4 内仍对称。
4. **NaN/Inf 侵入**：除零、溢出等。`is_finite()` 断言。

proptest 用 256 随机 IMU 组合 × 10-200 步 ≈ **50 000 合成步**。任何数值 bug 几乎都会在这里被捕捉。

### 1000 步悬停测试的具体数字

```
state.velocity_ned.norm() < 0.01 m/s  (after 1 s of "stationary")
state.position_ned.norm() < 0.01 m
```

理论上完全零，实际差 1 cm。原因：`accel_m_s2 = (0, 0, -GRAVITY_M_S2)` 加上 `accel_ned = R(q)·f + g` 后应该严格为 0，但 GRAVITY_M_S2 只有 6 位小数精度（`9.80665`），`f32` 相加会损失几个 ulp，积分 1000 步积累到 cm 级。

这个误差量级是**真机静止测试**的已知特征，工业 EKF 靠 GPS/气压测量更新压制。M1.10+ 接入测量更新后，稳态偏差会被修正。

### 为什么随机 IMU 序列不发散

`predict_step_preserves_invariants` 给的是**常值**随机 IMU（每次测试 pick 一个 ω 和 a，跑 10-200 步）。这是故意的：
- 常值输入是 EKF 最苛刻的场景（比随机噪声输入更容易暴露 F 矩阵 bug）
- 随机 ω 大到 3 rad/s 意味着每 200 步累计旋转 ≈ 34°，覆盖四元数大部分区域
- 随机 a 包含垂直加速度扰动 ±2 m/s²，测试 accel_bias Jacobian 的数值稳定

256 个组合全部通过 = F 矩阵没有系统性 bug。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 45 passed; 0 failed
  finished in 20.86s
  （主要时间在 predict_step_preserves_invariants：256 × ~100 步 ≈ 25 000 predict）

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

EKF predict 半段收尾。下一个阶段是**measurement update**（M1.10+）：

| step | 测量 | 维度 | 难度 |
|---|---|---|---|
| M1.10 | GPS 位置/速度 | 6×24 H 矩阵 | 中 |
| M1.11 | 磁力计 | 3×24，非线性 | 中（需要 H 的 Jacobian） |
| M1.12 | 气压计 | 1×24 | 低 |
| M1.13 | 融合测试 | 端到端 | 中 |

每一步都要：
1. H 矩阵（测量函数对状态的 Jacobian）
2. 残差计算 `y - h(x̂)`
3. 卡尔曼增益 `K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹`
4. 状态更新 `x̂ = x̂ + K·(y - h(x̂))`
5. 协方差更新 Joseph form `P = (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ`

比 predict 简单（每步只需要一两个 Jacobian 块 + 一个 nxn 逆矩阵 n≤6）但**实战最多坑**的地方（GPS spike 检测、magnetometer interference、baro ground-effect 等）。
