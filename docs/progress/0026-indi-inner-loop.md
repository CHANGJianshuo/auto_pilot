# 0026 — INDI 内环 + 低通滤波器

**Commit**: `25a8e3f`  **Date**: 2026-04-21  **Milestone**: M2.0

## What / 做了什么

把 `algo-indi` 从 M0 的骨架升级为生产级。INDI 是本项目的**核心差异化点 #2**（NMPC + INDI 混合控制栈）的一半。

新增：
- `pub struct RateCommand / TorqueIncrement`
- `pub type RateGain / Inertia`（Matrix3 别名，语义清晰）
- `pub struct IndiInput<'a>` —— 单步 INDI 的所有输入
- `pub fn compute_torque_increment(&IndiInput) -> TorqueIncrement` —— 纯函数
- `pub struct LowPassFilterVec3` —— 首阶 IIR 滤波器，含配置校验
- **10 个测试**（5 单元 INDI + 5 单元/proptest LPF）

## Why / 为什么这么做

### INDI 的数学直觉

PID 控制律：`τ = K_p·error + K_d·d_error + ...`，假设我们知道精确的 `J` 和 `ω̇` 模型。

INDI：**不假设模型精确**。把动力学线性化到**当前测量点**：
```
Δω̇ ≈ J⁻¹·Δτ      (所有建模不确定性都藏在 "上一步残余" 里)
⇒ Δτ = J·(ω̇_desired − ω̇_measured)
```

`ω̇_desired = k_rate·(ω_cmd − ω)` 就是 P-控制律产生的"期望角加速度"，`ω̇_measured` 是**实际角加速度**（传感器报的）。`J` 是飞机的惯性张量。

好处：
- **模型失配鲁棒** —— 真实 J 偏离 10% 不影响
- **外扰补偿** —— 风吹偏了？`ω̇_measured` 直接看到，反馈回来
- **对电机故障容忍** —— 单电机失效后，分配矩阵重构即可（不用重调 PID）

这是 TU Delft indiflight、Agilicious、PX4 实验分支公认的下一代控制律。

### 为什么 INDI 是**纯函数** + 外部状态

```rust
pub fn compute_torque_increment(input: &IndiInput) -> TorqueIncrement
```

caller 传进来**已经滤过的** ω 和 ω̇。`LowPassFilterVec3` 由 caller 管状态。好处：
- INDI 本身无副作用，容易 proptest（同输入 → 同输出）
- 滤波器复位 / 重新初始化不会污染 INDI 状态
- 可以任意换滤波器（将来换 dynamic notch、换 biquad）

### LowPassFilterVec3 的实用细节

**First-order IIR**：`y[n] = α·x[n] + (1−α)·y[n−1]`，其中 `α = dt/(τ+dt)`。简单、稳、CPU 廉价（1 次乘 + 1 次减 + 1 次加 × 3 轴 = 12 flops）。

**Invalid config → pass-through**（`α=1`）：如果用户传了 NaN 或 cutoff > Nyquist，不 panic，直接透传输入。这比返回 Option 更实用 —— 不想让每个调用点都 unwrap。

**First-sample initialization**：第一个 sample 直接赋值（不做递归），避免启动瞬态。

### 为什么 `torque_magnitude_is_bounded` 是 proptest 而不是 unit

6 个独立输入（cmd x/y/z、omega x/y/z、alpha x/y/z），手写几个 case 覆盖不全。proptest 256 组随机采样，**每次都重新验证闭式公式**：
```
expected = J · (k·(cmd − omega) − alpha)
got = compute_torque_increment(...)
|got − expected| < 1e-4
```

这是防止未来重构（比如切换到向量/矩阵 SIMD、或 fold 成 fused multiply-add）引入数值偏差的保险。

### `default_inertia()` = diag(0.015, 0.015, 0.025)

大约是 250 g 小四轴的值（`I_roll=I_pitch ≈ 1.5e-2 kg·m²`, `I_yaw ≈ 2.5e-2`）。测试里用一个**合理的量级**，避免 "J=1.0" 这种数值假人设让 proptest 通不出错。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-indi
test result: ok. 10 passed; 0 failed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M2.1**：`algo-alloc` 控制分配 —— 用伪逆把 virtual torque → 电机推力
- **M2.2**：attitude loop —— 上层 `(R_cmd, R_current)` → rate command（小角度近似下是 `q_error·2` 乘增益）
- **M2.3**：rate limiter + saturation handler —— 内环 + 分配共同约束
- **动态陷波**（dynamic notch）滤波 —— Betaflight 的杀手锏，跟踪电机基频压制结构共振
- **Kani** `compute_torque_increment` 的线性性：`f(x + y) = f(x) + f(y)`（纯线性方程可证）
