# 0013 — 过程噪声 Q + 占位 F 的协方差 predict

**Commit**: `81f1a25`  **Date**: 2026-04-20  **Milestone**: M1.9a

## What / 做了什么

把 `P ← F·P·Fᵀ + Q + 对称化` 这条公式的**数据流**跑通，但 `F` 先用单位矩阵占位。这样 M1.9b 替换真实 Jacobian 时，所有调用点、测试契约、内存布局都不用动。

新增：
- `pub struct ProcessNoise { attitude_per_s, velocity_per_s, position_per_s, gyro_bias_per_s, accel_bias_per_s, mag_ned_per_s, mag_bias_per_s, wind_per_s }` —— 每组状态的**每秒**1-σ 噪声强度
- `pub fn build_process_noise(noise, dt_s) -> Covariance` —— 按 `Q = diag((σ·√dt)²)` 生成单步噪声
- `pub fn identity_transition() -> Covariance` —— F=I 占位
- `pub fn predict_covariance(&P, &F, &Q) -> Covariance` —— `F·P·Fᵀ + Q` + enforce_symmetry
- 3 个新测试（共 20）

## Why / 为什么这么做

### 为什么分 M1.9a / M1.9b

完整的 `F` Jacobian 涉及四元数/速度/位置相对 IMU 测量的偏导，几百行推导（PX4 ekf2 的 derivation.py 跑几小时才生成）。一次性上会让单个 commit 审查不动。

分步的好处：
- **M1.9a（本 commit）**：所有调用点、类型、测试框架先固定。下游消费者（比如 app-copter 里的 EKF 循环）现在就能用 `predict_covariance`，只是 `F=I` 没有把姿态/速度的相关性带入 P 而已。
- **M1.9b**：只改 `F` 的生成代码，其它文件零改动。测试契约（对称、PSD、维度）继续生效。

### 为什么 `Q = (σ·√dt)²` 而不是 `σ²·dt`

离散化 Wiener 过程：连续时间的白噪声强度 `σ²` 积分 `dt` 后方差是 `σ²·dt`。也就是**1-σ 在时间间隔 `dt` 内累积为 `σ·√dt`**。

我的字段命名 `*_per_s` 明确是**每秒**1-σ，乘 `√dt` 得到该 dt 步内的 1-σ，再平方得到方差进对角。

```rust
sigma_dt = sigma_per_s * sqrt(dt)
Q[i, i] = sigma_dt²
```

这和 `σ² · dt` 数值上相同（方差线性 with dt），但**语义**是直接的："我设 1° 每秒的姿态噪声" → 1 ms 后姿态 σ ≈ 1°·√0.001 ≈ 0.032°。

### 为什么 `position_per_s = 1e-6`（几乎零）

位置的"真实"过程噪声来自**速度积分**。如果我给位置分量额外的过程噪声，就等于承认"位置可能凭空漂移"，EKF 在 GPS 更新前会被这项 dominated，违背物理。

但如果 Q 某个对角块是**精确的零**，在 PSD 检查 / Cholesky 时可能数值不稳定（零协方差 = 无穷大置信度 = 更新时除零）。所以给个 `1e-6` 的"极小但非零"值，既保证 PSD 又不影响 EKF 收敛。

这是 EKF 实现里的经典 trick（PX4 ekf2、ArduPilot AP_NavEKF3 都这么做）。

### 为什么 `predict_covariance` 接受 `&F` 而不是自己生成

解耦：本函数只管数据流（矩阵乘 + 对称化），**不管 Jacobian 怎么来**。M1.9b 会有一个独立函数 `build_transition_jacobian(state, imu, dt) -> Covariance` 生成真 F，然后调用方串起来：

```rust
let f = build_transition_jacobian(&state, &imu, dt);
let q = build_process_noise(noise_params, dt);
let p_next = predict_covariance(&p, &f, &q);
```

这样每个函数都能独立 proptest / Kani，组合起来就是完整 predict。

### F=I 的语义含义

"状态的扰动以等幅 1:1 传递到下一步"。物理上这**不正确**（比如姿态扰动应该让速度估计的不确定度通过旋转矩阵传播），但它**正确地不引入新的错误** —— 对角项按 Q 增长，非对角项保持为零。整个 predict_covariance 不会让 P 变坏，只是**不让它变更好**。

在有 GPS/mag/baro 测量更新的情况下，F=I 的 EKF 依然能悬停（收敛速度慢一点，方差被 GPS/mag 直接修正）。这就是"能飞，但还没到工业级精度"的中间状态。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 20 passed; 0 failed
  含 8 个 proptest

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

关键不变量测试：
- `predict_covariance_with_identity_f_adds_q`：手算每个对角 `p_next[i,i] == p[i,i] + q[i,i]`，全部 24 个位置 OK。
- `predict_covariance_keeps_symmetric_input_symmetric`：对称输入 → 对称输出（1 ulp 内）。
- `build_process_noise_rejects_bad_dt`：负 / NaN / 0 的 dt 返回零 Q，调用方的 predict 会自己跳过。

## Follow-ups / 遗留

- **M1.9b**：写真实 F = ∂f/∂x 的 24×24 Jacobian。
  - 四元数对 ω·dt 的偏导（最复杂，4×3 块 + 4×4 块）
  - 速度对姿态的偏导（旋转 body → world 的偏导）
  - 速度对 accel_bias 的偏导
  - 位置对速度的偏导（= I·dt，平凡）
  - ...
- ProcessNoise 的**值**以后要从参数表读，而不是 `default()` 硬编码。
- Q 现在是纯对角，真实 F 生效后 P 会有非对角项，enforce_symmetry 才开始发挥作用。
