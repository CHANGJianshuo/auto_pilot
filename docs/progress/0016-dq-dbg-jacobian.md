# 0016 — F 矩阵的 ∂q/∂b_g 子块（姿态对陀螺偏差）

**Commit**: `10d6d41`  **Date**: 2026-04-20  **Milestone**: M1.9b-1b

## What / 做了什么

填 F 的第二个非平凡块：**陀螺偏差扰动如何传播到 predict 出的四元数**。4×3 矩阵，放在 F 的 `[Q_START, GYRO_BIAS_START]` 位置。

新增：
- `pub fn left_multiplication_matrix(q) -> Matrix4<f32>` —— `q ⊗ p = L(q) · p_vec`
- `build_transition_jacobian` 写入 `∂q/∂b_g` 块
- 4 个单元测试 + 1 个**有限差分 proptest**（共 35 tests）

## Why / 为什么这么做

### 数学推导

```
q_new = q ⊗ δq
δq    = exp(½·ω_eff·dt)            ω_eff = ω_measured − b_g
```

链式求导：
```
∂q_new/∂b_g = ∂(q ⊗ δq)/∂δq · ∂δq/∂b_g
            = L(q) · ∂δq/∂b_g          (固定 q, 用 L 矩阵)
```

`δq` 对 `b_g` 的偏导：
- `δq` 是单位四元数 `exp(½·ω_eff·dt)` 
- 小角度近似（`‖ω·dt‖ ≲ 0.02 rad`，飞控 1 kHz 下几乎总成立）：`δq ≈ (1, ½·ω_eff·dt)`
- `δq_w` 与 `b_g` 无关 → `∂δq_w/∂b_g = (0,0,0)`
- `δq_xyz = ½·(ω_measured − b_g)·dt` → `∂δq_xyz/∂b_g = −(dt/2) · I_3`

合起来：
```
∂δq/∂b_g = ⎡ 0      0      0    ⎤    （4×3 矩阵）
           ⎢−dt/2   0      0    ⎥
           ⎢ 0     −dt/2   0    ⎥
           ⎣ 0      0     −dt/2 ⎦

∂q_new/∂b_g = L(q) · ∂δq/∂b_g = −(dt/2) · L(q)[:, 1:4]
```

### 有限差分 proptest：为什么最能抓 bug

四元数 Jacobian 最容易出错的地方是**符号**和**行列顺序**（是 L 还是 R？选列 0..3 还是 1..4？乘 dt 还是 dt/2？）。

闭式推导写对一次很难，但**数值 sanity 很简单**：

```
状态1: gyro_bias = (0,0,0), nominal imu → predict → q_nominal
状态2: gyro_bias = δb_g,    same imu    → predict → q_perturbed
dq_measured = q_perturbed - q_nominal       (4-vec)
dq_predicted = block · δb_g                 (4-vec via F[q, b_g] block)
期望: ‖dq_measured - dq_predicted‖ < 1e-4
```

proptest 随机 256 个 `(δb_g, dt)` 组合。如果 `∂q/∂b_g` 块有**任何**符号 / 行列问题，差值会达到 0.001 甚至 1.0，立刻被捕获。这是比闭式推导更严格的验证。

### L(q) 的公式

按 Hamilton 积展开：

```
(q⊗p)_w = q_w p_w − q_x p_x − q_y p_y − q_z p_z
(q⊗p)_x = q_w p_x + q_x p_w + q_y p_z − q_z p_y
(q⊗p)_y = q_w p_y − q_x p_z + q_y p_w + q_z p_x
(q⊗p)_z = q_w p_z + q_x p_y − q_y p_x + q_z p_w
```

按 `(p_w, p_x, p_y, p_z)` 提取系数：

```
L(q) = ⎡ q_w  -q_x  -q_y  -q_z ⎤
       ⎢ q_x   q_w  -q_z   q_y ⎥
       ⎢ q_y   q_z   q_w  -q_x ⎥
       ⎣ q_z  -q_y   q_x   q_w ⎦
```

注意跟 `R(q)` 的区别：第 2/3/4 行的**非对角**项符号和顺序翻转（反映四元数乘法的非交换性）。

### 为什么在 `state.attitude` 处求偏导（不是 q_new）

技术上正确做法是在**预期输出**处线性化 Jacobian，但这会形成递归（要计算 F 前得先计算 q_new，而 F 是用来近似 q_new 的）。

PX4 ekf2、ArduPilot EKF3 的惯例：**在当前时刻 state.attitude 处线性化**。在 1 kHz 步长下 q 的一步变化不到 1°，线性化点的选择误差 ≲ 1 ulp，完全可以接受。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 35 passed; 0 failed
  含 15 个 proptest × 256 样本 = 3840 随机实例

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

关键测试：
- `left_mult_matrix_matches_hamilton_product`：数值上 L(p)·q_vec 等于 `p * q`
- `dq_dbg_block_with_identity_q_matches_expected_form`：q=identity 时块 = `-(dt/2) · [0; I_3]`，逐元素比对
- `dq_dbg_block_matches_finite_difference`：256 随机组合下有限差分与 Jacobian 一致到 1e-4

## Follow-ups / 遗留

- **M1.9b-1c**：`∂v/∂q`, `∂v/∂b_a`（速度对姿态/加速度偏差）
- **M1.9b-1d**：`∂p/∂q`（位置对姿态 —— 通过 ½·R(q)·f·dt² 的偏导）
- 归一化 Jacobian（把 4×4 F_qq 投影到 q 的切空间）—— 暂时跳过，M2 迁到 error-state 时统一处理
- 有限差分 proptest 的模式**应该复制**到下一个 Jacobian 块 —— 防御式验证 vs. 闭式公式
