# 0023 — 气压计 update（1 维高度）

**Commit**: `ab3e2b3`  **Date**: 2026-04-20  **Milestone**: M1.12

## What / 做了什么

EKF 第三个测量源，也是最简单的一个 —— 1 维标量观测。

新增：
- `pub const BARO_CHI2_GATE = 6.635` —— χ² 99% 1-DoF
- `pub struct BaroMeasurement { altitude_m, sigma_m }`
- `pub struct BaroUpdateResult { state, covariance, applied, nis }`
- `pub fn baro_update(...)`
- 4 单元 + 1 proptest（共 65）

## Why / 为什么这么做

### 为什么最简单

1-D 观测 → 标量 S、标量 NIS、标量求逆（就是除法）。所有数学从矩阵代数退化成算数：

```
residual = z − h(x)                        (标量)
s = P[down, down] + σ²                     (标量)
nis = residual² / s                         (标量)
K = -P[:, down] / s                         (24-vector, 因为 H = -e_down)
```

没有 `try_inverse`，没有 3×3 矩阵。只要 `s > 0` 除法就合法。

### NED 高度符号约定

NED：N/E/D 三轴，z 朝下（Down）。高度是"朝上的距离"，所以：
```
altitude = -position_ned.z
h(x) = -position_ned.z
```

H 的形式：
```
H = [0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, ...]   (1×24, 在 P_NED_START+2 处为 -1)
```

所以 `K = P·Hᵀ/s = -P[:, P_NED_START+2] / s`。负号是符号本身，不是错误。

### Joseph form 1-D 退化

一般 Joseph form：`P ← (I − KH)·P·(I − KH)ᵀ + K·R·Kᵀ`

1-D 下：
- `R` 是 1×1 标量 `σ²`
- `K·R·Kᵀ = k·σ²·kᵀ = σ² · k·kᵀ`
- `(I − KH)` 的 H 只在 down_idx 列有 `-1`，所以 `K·H` 只在 down_idx 列有 K。
- `(I − K·H)[:, j] = I[:, j]`（j ≠ down_idx）
- `(I − K·H)[:, down_idx] = I[:, down_idx] − K · (−1) = I[:, down_idx] + K`

实现里我就用 `i_minus_kh.fixed_columns_mut::<1>(down_idx) += k` 直接加 K 到那一列。

### χ² 1-DoF 门值 6.635

1-D χ² 的 99% 分位数。比 3-DoF 的 11.345 小得多 —— 单维度信息少，同样的概率需要更严格的 cut。

实际应用中气压计在**地面效应**（起降时地面空气压缩导致读数偏高 1-2 m）场景会临时关掉或调宽门值。这些策略等 M2 飞行测试时补。

### 收敛 proptest

```
target_altitude ∈ [-20, 20] m
for 30 iterations: baro_update(target, σ=0.1m)
assert |final_altitude - target| < 0.3 m
```

256 random targets 全通过。气压计收敛比 GPS 快是因为 `sigma=0.1m` 比 GPS 的 `sigma=0.2m` 紧，又是 1 维直接观测。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 65 passed; 0 failed
  含 25 proptest × 256 ≈ 6400 随机实例

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M1.13**：端到端综合测试 —— predict + GPS + mag + baro 全部启用，模拟 "起飞 → 悬停 → 降落" 的多分钟序列
- **M1.14**：NIS 连续被拒接入 `HealthLevel` 状态机（`algo-fdir`）
- 温度补偿：BMP388 数据手册有精度随温度变化的模型，M2 做
- 地面效应门控：起飞/降落阶段临时放宽 BARO_CHI2_GATE 或暂停 baro update
