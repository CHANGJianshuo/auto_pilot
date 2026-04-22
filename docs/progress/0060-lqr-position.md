# 0060 — LQR 位置环：迈向 MPC 的第一步

**Commit**: `1738c0a`  **Date**: 2026-04-22  **Milestone**: M9.0

## What / 做了什么

把位置外环从**手调 P-P 级联**升级到**代价最优线性反馈 (LQR)**：

```rust
let g = compute_lqr_gains(LqrWeights { q_pos, q_vel, r }, dt_s)?; // 求 2×2 DARE
// g.k_p / g.k_v 是 "无约束、无限 horizon" MPC 的闭式最优增益
```

数学：
- 每轴独立的**双积分器模型**：`x_{k+1} = A x_k + B u_k`，其中 `x = [pos, vel]`, `u = accel`
- 代价：`J = Σ (x_k^T Q x_k + u_k^T R u_k)`，`Q = diag(q_pos, q_vel)`, `R = r`
- **离散时间 DARE**：`P = A^T P A − A^T P B (R + B^T P B)^{-1} B^T P A + Q`，收敛到定点
- 最优反馈：`K = (R + B^T P B)^{-1} B^T P A`，控制律 `u = −K(x − x_ref)`

没有 QP 求解器、没有 constraints —— 但这是 **MPC 的骨架**。plan.md 的"M4 NMPC"完整形态是这一步 + 约束 + 有限 horizon；先把无约束无 horizon 的 analytical 版本做成、测过、开关可切。

API 扩展（向后兼容）：
- `LqrWeights / LqrAxisGains` 新类型
- `compute_lqr_gains(weights, dt)` —— 迭代 DARE 求解 2×2
- `lqr_position_gains(xy, z, dt, max_accel)` —— 把三轴 LQR 增益塞进既有的 `PositionGains`

现有 `position_to_attitude_thrust{,_pi}` 零改动。想用 LQR 就填一个不同的 `PositionGains`。

**测试**：6 新 algo-nmpc + 1 新 sim-hil SITL（LQR 控制 vehicle 在 ideal sim 下 3 秒内悬停到 1 m setpoint，位置误差 < 25 cm），workspace **196 tests 全绿**。

## Why / 为什么这么做

### 为什么从 LQR 开始而不是直接上 MPC

MPC = LQR + constraints + finite horizon。**LQR 是 MPC 的三块里最 stable 的那一块**：

1. **数学明确**：DARE 有唯一稳定解，不像 QP 求解器可能不收敛；
2. **无溶剂依赖**：纯 nalgebra 2×2 运算，no_std 原地跑，不需要 OSQP / Clarabel；
3. **可验证**：性能和稳定性有解析界 (spectral radius of A − BK)；
4. **是 MPC 的"warm start baseline"**：即使将来用 QP 求 MPC，第一迭代从 LQR 解出发收敛最快。

跳过 LQR 直接做 MPC，相当于同时调试 QP 求解器 + 约束处理 + horizon 截断 + 实时 convergence —— 太多 moving pieces。先把 LQR 做对，再往上加层次。

### 为什么"per-axis decoupled"而不是整机 6×6

四旋翼位置动力学**本质上是耦合的**：推力方向由姿态决定，横向加速度通过 tilt 实现。但：

- 我们这一层是 **setpoint → accel_ned**；accel → tilt 在下一层（`position_to_attitude_thrust` 里的力平衡）做；
- 对每轴独立设计 LQR 得到三个独立增益 (k_p, k_v)，然后组装 `PositionGains`；
- 6×6 MPC 会多考虑姿态动力学（"倾 30° 要耗时"），但这对位置环收益不大 —— 姿态已经在 500 Hz 环上快得多。

三个独立 2×2 DARE 计算 = 6 × (2×2 矩阵乘) × 200 迭代 = ~5 000 flop，µs 级。MPC 那种 "一次解完全部" 的成本通常是 ms 级。设计时间 vs 运行时间 trade 站得住。

### 为什么迭代 DARE 到收敛，不用 Schur / Hamiltonian 闭式

闭式解：
- Hamiltonian matrix + Schur decomposition（nalgebra 不直接支持）
- 或者 Kleinman 迭代（类似我们做的）
- Laub 算法（数值稳定但复杂）

迭代 DARE 代码 20 行：
```rust
for _ in 0..200 {
    p = a'·p·a − a'·p·b·(r+b'·p·b)^{-1}·b'·p·a + q;
    if trace_change < ε: break;
}
```

对 2×2 问题收敛 < 50 迭代。设计时间计算（不在 1 kHz 环里反复），200 迭代的 worst case 有余裕。

### 为什么 DARE 迭代有 trace-relative 停机判据

绝对 `‖P_{k+1} − P_k‖ < ε` 的阈值选择与问题 scale 紧耦合：如果 `q_pos = 1000`，P 的 norm 也会大，绝对 ε 变成不稳定。相对判据：

```rust
if (trace - prev_trace).abs() < 1.0e-6 * (1.0 + trace.abs()) { break; }
```

对所有 well-scaled 问题稳定。1e-6 对 f32 相对精度已经到极限（f32 epsilon ≈ 1.19e-7）。

### 为什么 `compute_lqr_gains` 返回 `Option`

失败模式：
- 输入 NaN / negative weights / dt ≤ 0 —— 调用方程式 bug；
- DARE 迭代发散（非常规 Q/R 组合理论上可能）；
- `R + B^T P B ≤ 0` —— 数值病态。

`Result<_, Error>` 层次化更严格但 case 少，`Option` 足够。调用方 fallback 到 PI gains 即可，不是 unrecoverable。

生产固件可以用 `defmt::warn!` 记录具体失败原因（当前 no_std panic-free 上下文里只返 None）。

### 为什么 LQR SITL 测试用 ideal sim，不 realistic

ideal sim（无噪声 / 无阻力 / 无风）下 LQR 应当**严格收敛**到设定点（不动点 stability + optimality）。放宽到 realistic，残差受 drag / wind 扰动主导，LQR 的**特性**被噪声淹没 —— 测不清该 commit 做对了什么。

PI cascade 的 realistic SITL 测试（M3.2）依然在；PI 在有扰动时更健壮（因为有 I 项）。LQR 加 I 项 / 观测器在 M9.2。

### 为什么 integrator 在 LQR 版本里 disable

LQR 的**最优性前提**是"模型完全已知"。I 项是对 "真系统 ≠ 模型" 的补偿。纯 LQR + I 会破坏 LQR 的 optimality guarantee，同时手动调整 k_i 无理论基础。

正确做法：
1. **LQI** (LQR + Integrator)：把 integrator state `∫pos_err dt` 加进 state vector，重新解 DARE。增益是 "三维" 的：(k_p, k_v, k_i) 一体优化。
2. **Disturbance observer**：估 "drag + wind" 作为 state，让 LQR 自动补偿。

M9.2 候选。现阶段保持 "纯 LQR" 的清洁语义，让读者看清 "LQR 做的事" vs "PI 在做的事"。

### 为什么 default weights `(4, 1, 0.5)`

纯凭手调匹配既有 PI cascade 的响应速度。关系：
- Q/R ratio 大 → stiff 控制 → 快收敛、用力多
- Q/R ratio 小 → soft 控制 → 慢收敛、省力

`q_pos = 4` 压过 `r = 0.5` 一个量级 → 中等 stiff。实测 250 g 机在 dt=0.05 下：`k_p ≈ 2.0, k_v ≈ 2.5`，同量级于 PI 的 `k_pos = 1.0, k_vel = 3.0`。

但 LQR 的神奇是**这些是最优的**，而不是"某人试出来的"。新机型不要 try-and-error，只需 tune `q_pos/q_vel/r` 三个有物理意义的参数。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-nmpc
14 passed (+6 新 LQR)

$ cargo test -p sim-hil lqr
1 passed — ideal SITL + LQR 收敛到 1 m 悬停，误差 < 25 cm

$ cargo test --workspace
196 passed

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿

$ cargo build -p algo-nmpc --lib --target thumbv7em-none-eabihf
Finished (no_std compatible)
```

## Follow-ups / 遗留

- **M9.1 box-constrained QP**：真 MPC 的第一层约束。u_min ≤ u ≤ u_max（加速度 saturation），active-set 或 projected-gradient 求小规模 QP。
- **M9.2 finite receding horizon**：当前 infinite-horizon。H = 10 (200 ms lookahead) 配合 QP 能处理 "未来短期内接近边界" 的情况。
- **M9.3 LQI（LQR + integrator）**：把积分态放进 state，重解 DARE。代替 M3.2 PI 里的手调 k_i。
- **M9.4 disturbance observer**：EKF 已估 wind；把它作为 known disturbance 喂进 LQR，得 feed-forward 项。
- **M9.5 gain scheduling**：不同 mass / speed 范围重算 LQR，存表插值。
- **对比 benchmark SITL**：PI vs LQR 在 realistic sim 下谁跟踪误差小。目前只 ideal sim 测了 LQR。
- **Kani 证 DARE 单调 / P 半正定**：数值形式化验证。
- **3×3 耦合 LQR（跨轴项）**：四旋翼不是严格解耦；加上 yaw-axis 和姿态 → 位置的耦合。不紧急。
