# 0021 — GPS Kalman gain + Joseph-form update

**Commit**: `ed3114f`  **Date**: 2026-04-20  **Milestone**: M1.10b

## What / 做了什么

把 M1.10a 的 innovation 拓展成完整的测量更新：卡尔曼增益 `K` + 状态修正 + **Joseph 式协方差更新** + χ² 外值拒绝。

新增：
- `pub const GPS_CHI2_GATE = 11.345` —— χ² 99% 3-DoF
- `pub struct GpsUpdateResult { state, covariance, applied, nis }`
- `pub fn gps_update(&state, &covariance, &measurement) -> GpsUpdateResult`
- 4 单元 + 1 proptest（共 55）

## Why / 为什么这么做

### Joseph form 的价值

最短的 EKF covariance 更新是"简化式"：
```
P_new = (I − K·H) · P
```

数学上对，但**浮点下很脆弱**：舍入让 (I-KH) 的某些元素偏离理论 10⁻⁷，几步后就让 P 失对称/失 PSD。

Joseph form：
```
P_new = (I − K·H) · P · (I − K·H)ᵀ + K·R·Kᵀ
```

比简化式**多两次矩阵乘法**，但：
- **对称性自动**：两项都是 `A·X·Aᵀ` 形状，对称输入 → 对称输出
- **对 K 的微小误差鲁棒**：K 的舍入误差进入 (I-KH)·(I-KH)ᵀ 后只影响二阶项
- **PSD 自动**（当 P 原本 PSD 且 R PSD 时）

所有工业级 EKF（PX4 ekf2、ArduPilot、Swift、Agilicious）都用 Joseph form。性能代价可以接受（24×24 矩阵乘法 ~14K flops，Cortex-M7 M7 可以在几微秒做完）。

### χ² 门值 11.345 的来源

NIS 是 3-DoF χ² 分布，99% 分位数 = 11.345（查表或 `scipy.stats.chi2.ppf(0.99, df=3)`）。

这意味着**如果 EKF 模型完全正确**，平均只有 1% 的真实观测会超过这个阈值被拒。实际中：
- 正常 GPS fix → NIS < 3
- 多径反射 → NIS 10-50
- GPS jump/re-fix → NIS >> 100
- 欺骗/失效 → NIS = ∞

超过 11.345 几乎总是异常，**拒绝不损失信息** 。真漏报几个好测量比吸入一个坏测量致灾好得多。

阈值调整：
- 高动态飞行可以放宽到 25 以容忍模型误差
- 对关键 GPS 数据（如着陆）可以收紧到 5

### `from_vector(state.to_vector() + dx)` 作为状态修正

EKF 状态修正是向量加法 `x̂ += K·y`。但我们的 `State` 是 struct 形式。解法：
```rust
let dx = k * innov.residual;                     // SVector<f32, 24>
let mut new_state = State::from_vector(&(state.to_vector() + dx));
new_state.normalize_attitude();                  // q 不在 SO(3) 切空间上
```

两步：
1. 直接往 24D 向量加修正量
2. 重新归一化四元数（EKF 不管它，所以我们管）

这是标准的"加性 EKF + 四元数 renormalization"的组合。严格说四元数应该用乘性修正 `q ← q ⊗ exp(½·δθ)`，但只要 δθ < 0.01 rad 就等价。M2 以后如果迁到 error-state，这一步会改。

### 为什么不把 K 存成 struct 字段

有些 EKF 把最近的 K 存下来做事后诊断。我们这里故意让它是临时变量 —— GC 很小（24×3 = 96 个 f32 = 384 B），每步丢掉。生产时 K 本身不用跨函数流动，简单就行。

### 收敛性 proptest

```
for _ in 0..20 { state = gps_update(target, sigma=0.2m) }
assert |state.position_ned - target| < 0.5m
```

这是**端到端**验证：GPS 以 σ=0.2m 的精度报测量，EKF 经过 20 次更新后，估计值应该在 0.5m 内。**256 个随机目标**全通过。

这验证了：
- K 计算正确（符号、大小）
- 状态修正方向对（向测量，不背离）
- 协方差缩减合适（后续 update 的 K 逐渐减小）
- NIS 门在合理测量下都通过

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 55 passed; 0 failed
  finished in 20s  (主要是 256 × 20 次 gps_update 的 proptest)

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M1.10c**：EKF 端到端（predict + GPS update）长序列 proptest，模拟飞行
- **M1.11**：磁力计 update —— 非线性 h(x) = R(q)·m_earth，需要 `rotation_jacobian_wrt_q`
- **M1.12**：气压计 update（1 维最简）
- GPS **速度**测量（u-blox F9P 报 pos + vel，两者可以并发更新）留到驱动层做
- 外值被拒后的 recovery 策略（3 连续被拒 → HealthLevel::Degraded）—— 接 `algo-fdir`
