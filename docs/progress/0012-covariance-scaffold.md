# 0012 — 协方差矩阵骨架 + 对称性修复

**Commit**: `cf8196a`  **Date**: 2026-04-20  **Milestone**: M1.8

## What / 做了什么

把 `P` 矩阵（EKF 的协方差）相关的类型别名、初始化、维护工具统一落地。为 M1.9（完整 predict 协方差更新 `P ← F·P·Fᵀ + Q`）铺路。

新增：
- `pub type Covariance = SMatrix<f32, 24, 24>`
- `pub mod initial_sigma { ATTITUDE, VELOCITY_NED, POSITION_NED, GYRO_BIAS, ACCEL_BIAS, MAG_NED, MAG_BIAS, WIND_NE }` —— 每组状态的启动 1-σ 值及单位注释
- `pub fn initial_covariance() -> Covariance` —— P₀ = diag(σ²)
- `pub fn enforce_symmetry(&mut Covariance)` —— `P ← (P + Pᵀ)/2`
- 1 个新 proptest + 3 个单元测试（共 17 测试）

## Why / 为什么这么做

### σ 值是怎么定的

我按 ICM-42688-P 数据手册、u-blox F9P 开机性能、以及飞控社区的保守默认值拍的：

| 状态组 | 初始 σ | 单位 | 来源 |
|--------|--------|------|------|
| attitude | 0.1 | rad (小角度近似) | 第一次 accel/mag 对齐前的不确定度 |
| velocity_ned | 1.0 | m/s | 开机时不知道是否动 |
| position_ned | 10.0 | m | GPS fix 前的位置误差 |
| gyro_bias | 0.01 | rad/s | ICM-42688 datasheet |
| accel_bias | 0.1 | m/s² | ICM-42688 datasheet |
| mag_ned | 0.05 | gauss | 地磁场强度地理差异 |
| mag_bias | 0.2 | gauss | 硬铁校准前的误差 |
| wind_ne | 2.0 | m/s | 保守估计 |

这些是**起飞阶段**的上界。飞行中 EKF 会收敛到远小于这些的值。

### 为什么 P₀ 是对角矩阵

在没有任何飞行数据前，我们无法声明任何两个状态分量之间有相关性（不然就是无中生有）。对角 P₀ 等价于"各分量独立不确定"。EKF 跑起来后，测量会让非对角项增长，那是物理相关性（例如姿态误差和水平速度的相关性）的正确来源。

### 为什么 `enforce_symmetry` 是一条独立函数

数学上 `P` 永远对称。但浮点下 `P ← F·P·Fᵀ + Q` 的矩阵乘法**每一步都引入 ~1 ulp 的非对称**。1 kHz 运行 1 小时累积 3.6M 步，非对称性能积累到让 Cholesky 分解或 PSD 检查误判。工业级 EKF（PX4、ArduPilot）都在每次更新后强制 `(P+Pᵀ)/2`，本质上是**免费的数值保险**。

把它做成独立函数而不是 "`predict_covariance` 自己调用" 有两个考量：
1. Measurement update 也要调，独立函数避免重复
2. 测试可以直接对 `enforce_symmetry` 跑 proptest —— 不变量"输出对称"比端到端验证好写多了

### 为什么没用 `p[(i,j)]` 索引

workspace `Cargo.toml` 的 `indexing_slicing = deny` 把这种写法编译拒掉。用 `p.fixed_view::<1,1>(i,j).to_scalar()` / `fixed_view_mut` 获得 1×1 的子块视图，语义相同且通过 clippy。这是写 EKF 数学代码必须掌握的 nalgebra 惯用法。

### 提前把 F / Q / K / H 想清楚

M1.8 只是下了"数据结构"。M1.9 要填算法：
- **F = ∂f/∂x** (24×24 状态转移 Jacobian)
- **Q** (24×24 过程噪声，只在几个对角块非零：gyro_bias 随机游走、accel_bias 随机游走、wind 扩散等)
- **P ← F·P·Fᵀ + Q** 然后 `enforce_symmetry(P)`

F 的每一列对应一个状态分量被扰动时，`predict` 输出的偏导。最复杂的是四元数部分（涉及 Hamilton 乘积对 ω·dt 的 Jacobian）。几百行但**结构清晰**，本 doc 不展开。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 17 passed; 0 failed
  含 8 个 proptest × 256 样本

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M1.9** EKF predict 的协方差更新（F 矩阵 + Q + `P ← F·P·Fᵀ + Q` + 对称化）—— 本项目最复杂的几百行之一
- PSD 性质在 M1.9 之后才值得建测试（现在 P₀ 对角是琐碎的 PSD）
- initial_sigma 的值要在首次 SITL 飞行后重调
- 后续要加 `Q_per_second: Covariance`（过程噪声强度，按 dt 线性放大为 Q）
