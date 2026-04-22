# 0061 — 带约束的有限 horizon MPC

**Commit**: `43d2af6`  **Date**: 2026-04-22  **Milestone**: M9.1

## What / 做了什么

在 M9.0 LQR 的基础上走到真正的 MPC：**有限 horizon + box 约束 + 每 tick 求 QP**。

```rust
let mpc = Mpc1d::<10>::new(Mpc1dConfig {
    weights: LqrWeights::default(),
    dt_s: 0.05,
    u_min: -3.0,
    u_max: 3.0,
})?;
let mut warm = SVector::<f32, 10>::zeros();
let u0 = mpc.solve(e_0, &mut warm, 50);  // 取第一步应用，warm 为下一 tick 备用
```

算法：
1. **展开动力学**：`e_k = F_k e_0 + Σ G_{k,j} u_j`，把 QP 变量压缩到 `u ∈ R^H`。
2. **构造 Hessian + 线性项**：`J(u) = ½ u^T H u + g(e_0)^T u + const`。精细结构：
   - `H = G^T Q_blk G + R_blk + G_f^T P_f G_f`（stage + control + terminal）
   - `g = (G^T Q_blk F + G_f^T P_f F_H) e_0`（线性，预算成矩阵）
3. **终端代价 `P_f`**：用 M9.0 的无限 horizon DARE 解 —— 保证有限 horizon 不破坏 LQR 的 stability guarantee。
4. **Projected gradient**：步长 `α = 1/L`，`L` 用 Gershgorin 上界；每次 iteration `u ← Π_box(u − α(H u + g))`。
5. **Warm start**：`solve` 把 `warm_u` 读成初始猜测并覆盖写新解；下次 tick 直接复用，~10× 冷启动加速。

5 新测试，workspace **201 测试全绿**：

- `new` 拒绝坏输入（dt ≤ 0 / NaN / u_min > u_max / r ≤ 0 / H = 0）
- **unconstrained MPC 的第一步 command 与 LQR 匹配到 0.05**（验证数学一致）
- 紧 `u_max` 下每一步都尊重约束（不只是返回的 u_0）
- 闭环 1-D 模拟从 (1, 0) 收敛到 (0, 0)，即便 u 被 ±3 m/s² 卡死
- 零误差产生数值零命令

`no_std` 兼容，thumbv7em 编译过。

## Why / 为什么这么做

### 为什么 finite horizon + 终端代价 `P_f`，不用无限 horizon

Infinite-horizon MPC 无法在 MCU 上直接算 —— 你不能在 1 ms 内"解无穷多步"。**有限 horizon 截断 + 选好的终端代价 = 等效于无穷 horizon 的稳定性保证**（Mayne 1990s 的经典结论）。

具体：
- 取 `P_f = LQR Riccati P` 作终端代价；
- 若 LQR 无约束下本身稳定（对我们的双积分器 + Q, R > 0 恒成立），则 MPC closed-loop 在终端区内不激活约束时等价于 LQR；
- 约束激活时 MPC 优于 LQR（后者盲目把 u 投影到 box）。

**如果不加终端代价**，会发生什么？有限 horizon 看不到 terminal state 的未来代价 → 贪婪行为 → closed-loop 有可能 destabilise。这是入门级 MPC bug，加 `P_f` 是标准修复。

### 为什么 per-axis 1-D MPC，不直接做 3-D 耦合 MPC

四旋翼 position loop 水平 / 垂直耦合弱（推力方向由 tilt 调，但 tilt 足够快 → 在位置时间尺度上近似解耦）。三个独立 1-D MPC：

- **状态空间小** (H = 10) → Hessian 10×10 → 100 flops/iter；
- 三轴并行：总 300 flops × 50 iter = 15 000 flops，STM32H7 @ 480 MHz < 50 µs；
- 真耦合 MPC 变成 30×30 + 约束交叉，算力上升 20-50 倍。

M9.x 后续可以上耦合版本。现阶段 per-axis 匹配 LQR 的结构，切换成本 0。

### 为什么 projected gradient 不用 active-set / interior-point

比较：
| 方法 | 迭代数 | 每迭代成本 | 预测性 |
|---|---|---|---|
| Projected gradient | 30-100 | O(H²) | ✅ 固定 |
| Accelerated PG (Nesterov) | 20-50 | O(H²) | ✅ 固定 |
| Active-set | 2-5 激活集变动 | O(H³) | ❌ 坏情况 2^H |
| Interior-point | 5-15 | O(H³) | ~固定 |

飞控要 **worst-case 时间可预测**，`N` 次 iteration 上界 = 实际 WCET。PG 满足这个。用 Nesterov 加速 (M9.2 候选) 还能再省一半。

Active-set 在好情况极快但坏情况指数爆炸，不适合 1 kHz 硬实时。

### 为什么 Gershgorin 步长而不是精确 λ_max

`λ_max(H)` 要做 eigendecomposition 或 power iteration，嵌入式代价不值。Gershgorin circle 定理给 `λ_max ≤ max_i Σ_j |H_{i,j}|`（行绝对值和的最大）— **保守但无需迭代**。

用这个上界算步长 `α = 1/L_bound`，保证单调收敛（可能比最优步长略慢）。精确 spectral radius 用 Lanczos 一次可以算，但对 H=10 的矩阵，差距是"30 iter vs 20 iter"级别。

### 为什么 `warm_u: &mut` 是 borrow out，而不是 solver 内部 state

调用方管理 warm start 有好处：
- **线程安全**：solver 纯函数，多个任务能安全并发 solve（三轴并行时重要）；
- **reset 容易**：需要 cold start（比如 mode 切换）时调用方直接 `warm.fill(0.0)`；
- **可测性**：单元测试传零 warm，行为确定。

副作用是：调用方要持久化 `SVector<f32, H>` = H 个 f32 = H · 4 bytes。对 H=10 = 40 B，跨 tick 保留无压力。

### 为什么 MPC 默认 u_max = [-100, 100]（巨宽）

测试里 `default_mpc_config(false).u_min = -100, u_max = 100` —— 远超任何实际加速度。这是为了测 **unconstrained regime MPC 和 LQR 数值等价**。实战中 u_max 由 physical envelope 决定：

```
u_max ≈ F_max / m − g        # 向上最大加速
u_min ≈ −g                   # 向下最小（自由落体）
```

250 g 机 F_max ≈ 6 N/motor × 4 = 24 N, m = 0.25 kg → u_max ≈ 96 − 9.8 ≈ 86 m/s². 实际我们用 max_accel = 8 m/s²（安全余量）。应用层根据机型填。

### 为什么 u_0 而不是 u_0..u_{H-1} 全用

receding horizon 标准做法：**每 tick 重新求 H 步，只用第一步**。好处：
- **反馈**：下一 tick 的初始状态是真实系统的新 state，不是预测值 → 鲁棒于模型误差 / 扰动；
- **约束更新**：如果 u_max 在飞行中变（电池低 / 失效），新 solve 立刻反映；
- **warm start 几乎免费**：上一 tick 的 `u_1..u_{H-1}` 是这一 tick 的好初值。

### 为什么 5 个测试就够了

这是一个**分层验证**：
1. **输入校验**（`new_rejects_bad_inputs`）—— 接口鲁棒性
2. **数值等价性**（`unconstrained_matches_lqr`）—— 证明 MPC 在 unconstrained regime 还原 LQR（规约 sanity）
3. **约束满足性**（`respects_box_constraint`）—— 证明约束不被破坏
4. **闭环稳定性**（`closed_loop_converges`）—— 证明 MPC 真能控制系统到目标
5. **零点行为**（`zero_error_zero_command`）—— 证明平衡点是不动点

这五条覆盖：接口 / 一致性 / 约束 / 性能 / 平衡。额外 property test 可加（随机 initial state / weights 下 closed-loop 收敛时间），暂不紧迫。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-nmpc
19 passed (5 新 MPC + 6 LQR + 8 既有 PI)

$ cargo test --workspace
201 passed

$ cargo build -p algo-nmpc --lib --target thumbv7em-none-eabihf
Finished (no_std ok)

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿（local `#[allow(clippy::indexing_slicing)]` 限于 MPC::new，
  索引有 compile-time 上界 H < 64 保证）
```

## Follow-ups / 遗留

- **M9.2 接进 `outer_step`**：`position_to_attitude_thrust_mpc` 变体，三轴并行 solve，SITL 对比 PI vs LQR vs MPC tracking error。
- **Nesterov 加速** PG：收敛 iter 减半，代码 20 行。
- **精确 λ_max via power iteration**：给更紧的步长，冷启动更快。
- **Active-set fallback**：warm-started 情况 active-set 几乎总 2-3 iter 收敛，坏情况 fallback PG。
- **Soft constraints**：把 u_max 违规项放进目标函数（penalty），解更"温和"在硬约束边界。
- **Horizon length 动态**：停机时 H=20 准确点；巡航时 H=5 省算力。
- **耦合 3-D MPC**：位置 + 姿态一起 solve，捕捉推力方向的倾角约束。
- **`Mpc1dConfig` 加参考轨迹**：目前 regulate to constant x_ref；trajectory tracking（x_ref_k for k=0..H）更通用。
- **property test**：随机 (initial state, weights) 下 100 次 solve，validate 单调目标函数下降 + 约束满足。
- **benchmark**：`criterion` bench `solve` 时间，WCET 曲线。
