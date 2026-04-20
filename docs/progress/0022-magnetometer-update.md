# 0022 — 磁力计 update（非线性 H）

**Commit**: `5c1231f`  **Date**: 2026-04-20  **Milestone**: M1.11

## What / 做了什么

第二个 EKF 测量源。和 GPS 不同，磁力计的观测模型对 q **非线性**（旋转地磁场到 body frame），必须显式构造 3×24 的 `H` 矩阵。

新增：
- `pub fn rotation_transpose_jacobian_wrt_q(q, v) -> Matrix3x4<f32>` —— `∂(R(q)ᵀ·v)/∂q`
- `pub fn predict_magnetometer(state) -> Vector3<f32>`
- `pub const MAG_CHI2_GATE = 11.345`
- `pub struct MagMeasurement { body_field, sigma }`
- `pub struct MagUpdateResult { state, covariance, applied, nis }`
- `pub fn mag_update(&state, &covariance, &measurement) -> MagUpdateResult`
- 4 单元 + 1 proptest（共 60）

## Why / 为什么这么做

### 观测模型

```
y_mag_body = R(q)ᵀ · mag_earth + mag_bias + noise
```

物理上：地磁场 `mag_earth` 在 NED 世界坐标系里（我们状态里的 `mag_ned`）。传感器装在 body 上测的是 body 坐标系下的分量，所以用 world→body 的旋转 `R(q)ᵀ`。加上硬铁偏差 `mag_bias`。

### 为什么必须显式构造 H

GPS 的 `H` 只在 `position_ned` 的 3 列非零，所以我们能用 block 抽取替代 `P·Hᵀ`。磁力计的 `H` 有**三块非零**：
- `q` 列（4 列，因为 R(q)ᵀ 对 q 是非线性）
- `mag_ned` 列（3 列，因为 `R(q)ᵀ·mag_ned` 对 `mag_ned` 线性）
- `mag_bias` 列（3 列，恒等）

这不是连续块，要 blocky 拼起来。所以我们直接构造 `SMatrix<f32, 3, 24>`，然后标准套路 `S = H·P·Hᵀ + R`、`K = P·Hᵀ·S⁻¹`、Joseph form。

开销比 GPS 大一倍但可接受：24×24 矩阵乘法在 Cortex-M7 上几微秒，磁力计更新率 25 Hz（40 ms 周期），预算充足。

### 转置旋转 Jacobian 的技巧

`R(q)ᵀ = R(q*)`，其中 `q* = (w, −x, −y, −z)` 是共轭。所以：

```
∂(R(q)ᵀ·v)/∂q  =  ∂(R(q*)·v)/∂q*  ·  ∂q*/∂q
                =  J_R(q*, v)  ·  diag(1, −1, −1, −1)
```

实现就是调 `rotation_jacobian_wrt_q(conjugate(q), v)`，把后 3 列乘 −1。复用前面写好的 Jacobian，不用重新推导 9 个元素 × 4 列 × 3 项。

### 为什么磁力计值"小"（σ ≈ 0.01）时效果好

地磁场 `‖mag_earth‖ ≈ 0.5 gauss`。σ=0.01 意味着 **2% 精度** → 每次 update 能把 yaw 的不确定度压到约 1°。一秒钟 25 个 update 后，yaw 通常收敛到 < 0.1°。

这就是磁力计的用途：**yaw 可观测性**。单靠 gyro + accel，roll/pitch 能收敛（重力方向可见），yaw 则只能 drift。磁力计给 yaw 提供绝对参考。

### 有限差分 Jacobian 测试的重要性

非线性 `H` 最容易出错。`mag_h_jacobian_matches_finite_difference`：
```
h(q + δq) − h(q)  ≈  H_q · δq
```
在 256 随机 δq 下匹配到 1e-4。这基本上是 `rotation_transpose_jacobian_wrt_q` 的最严格单元测试。

如果 `rotation_transpose_jacobian_wrt_q` 有符号错（比如漏了乘 -1 的那几列），这个 proptest 会立刻爆炸。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 60 passed; 0 failed
  含 24 个 proptest × 256 样本 = 6144 随机实例

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M1.12**：气压计 update（1 维 `h(x) = -state.position_ned.z`，最简单的）
- **M1.13**：综合测试 —— predict + GPS + magnetometer 长序列，模拟真实飞行
- **M1.14**：接入 FDIR 健康状态（NIS 连续被拒 → 报 `HealthLevel::Degraded`）
- 把 `mag_ned` 作为状态是 EKF3 的做法；PX4 ekf2 有时选择固定它（已知地磁场），代价是少一个对齐参数，节省一些 state 但依赖 WMM 查表
