# 0020 — GPS measurement 类型 + innovation

**Commit**: `edcc0e8`  **Date**: 2026-04-20  **Milestone**: M1.10a

## What / 做了什么

开始做 EKF 的 **measurement update 半段**。这个 commit 只做"构造"部分 —— innovation（残差）和 S 矩阵，先不做 Kalman 增益 / 状态修正（那是 M1.10b）。

新增：
- `pub struct GpsMeasurement { position_ned, sigma }` —— GPS 观测
- `pub struct GpsInnovation { residual, innovation_covariance }` —— innovation 数据
- `impl GpsInnovation::normalized_squared() -> f32` —— NIS（Normalized Innovation Squared），χ² 外值拒绝用
- `pub fn gps_innovation(&state, &covariance, &measurement) -> GpsInnovation`
- 4 单元 + 1 proptest（共 50）

## Why / 为什么这么做

### 为什么拆成"innovation"和"update"两步

EKF 测量更新的经典 5 步：

```
1. residual y = z - h(x̂)
2. S = H·P·Hᵀ + R
3. K = P·Hᵀ·S⁻¹
4. x̂ ← x̂ + K·y
5. P ← Joseph_form(P, K, H, R)
```

步骤 1-2 是**数据诊断**（can we see that the measurement is reasonable?），步骤 3-5 是**状态修正**（apply it to the filter）。

**1-2 拆出来**好处明确：
- **外值拒绝**：GPS jump / spoofing / multipath 可能给出 50 m 的假位置。先算 NIS，如果 > χ²_99%（3 自由度 ≈ 11.34），跳过 3-5，**don't corrupt the filter**。PX4 ekf2 和 ArduPilot EKF3 都这样做。
- **测试性**：1-2 只看输入/输出 3 维向量 + 3×3 矩阵，容易单元测试。3-5 要验证 P 的对称性、PSD、数值稳定性，复杂得多。
- **日志**：把 innovation 本身 log 下来，飞行后分析是 GPS 有多干净、EKF 是否过度自信。

### 为什么不把 H 矩阵实体化

H 是 3×24 的。对位置观测来说：

```
H = [ 0_{3×7}  |  I_3  |  0_{3×14} ]
```

只有位置 block 是非零。如果写成 `Matrix3x24<f32> = [H]`，然后 `H · P · Hᵀ = H * (24x24) * Hᵀ`，那是 3 × 24 × 24 = 1728 次 FMA。

直接抽取 `P[P_NED, P_NED]` 的 3×3 block `== H·P·Hᵀ`，1 次 block copy = 9 flops。**200× 速度**。还少了中间矩阵分配。

本 commit 的 `gps_innovation` 用 block 抽取：
```rust
let p_block: Matrix3<f32> = covariance
    .fixed_view::<3, 3>(idx::P_NED_START, idx::P_NED_START)
    .into_owned();
let r = Matrix3::from_diagonal(&sigma_sq);
let s = p_block + r;
```

这对所有**线性选子状态的测量模型**（GPS pos/vel、baro z、magnetometer 如果存简化模型）都适用。磁力计的非线性 h(x) = R(q)·m_earth 才需要雅可比 H（M1.11 的工作）。

### 为什么 NIS（normalized innovation squared）

```
NIS = residualᵀ · S⁻¹ · residual
```

这是个**标量**，符合 χ² 分布（当 EKF 模型正确时）。三维观测的 99% 分位数是 11.34。如果 NIS > 11.34，观测严重与模型不一致 → reject。

这是**工业飞控的标准外值门**。具体阈值可调（比如飞行中可以放宽到 25 以避开 GPS 间歇性跳变 + 重新获取），但总是用 NIS 做决策。

本 commit 提供 `normalized_squared()` 方法，后续 `gps_update_state_covariance` 会调用它做自动 gating。

### `Matrix3::try_inverse` 的异常保护

```rust
match self.innovation_covariance.try_inverse() {
    Some(s_inv) => ...
    None => f32::INFINITY,
}
```

如果 S 奇异（协方差分量全零 → σ=0），返回 `∞` 让外值门自然拒绝。避免 unwrap 或 panic。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 50 passed; 0 failed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

关键测试：
- `gps_innovation_residual_tracks_difference`：残差 = 测量 − 状态
- `gps_innovation_covariance_is_pblock_plus_r`：S = P_pos_block + diag(σ²)
- `gps_innovation_nis_monotone_in_offset`：256 proptest —— offset 越大 NIS 越大

## Follow-ups / 遗留

- **M1.10b**：Kalman gain + state/covariance update（Joseph form）
- **M1.10c**：NIS-based outlier gating 接入 EKF 主循环
- **M1.11**：Magnetometer update（非线性 H = R(q)·m_earth，需要之前做好的 `rotation_jacobian_wrt_q`）
- **M1.12**：Barometer update（一维测量，最简单）
- **M1.10d**：端到端 predict + GPS update 的长序列 proptest
