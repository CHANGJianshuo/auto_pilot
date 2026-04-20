# 0015 — F 矩阵的 ∂q/∂q 子块（四元数自耦合）

**Commit**: `c228afc`  **Date**: 2026-04-20  **Milestone**: M1.9b-1a

## What / 做了什么

把状态转移 Jacobian `F` 里**最重要**的一块补上 —— 四元数相对自身的偏导 `∂q_new/∂q`。

```
q_new = q ⊗ δq   where  δq = exp(½·(ω − b_g)·dt)
⇒ ∂q_new/∂q = R(δq)   where R(·) is right-multiplication matrix
```

新增：
- `pub fn right_multiplication_matrix(q) -> Matrix4<f32>` —— Hamilton 乘法右矩阵
- `pub fn build_transition_jacobian(state, imu, dt) -> Covariance` —— 整合 kinematic + 新的 ∂q/∂q 块
- 4 个单元测试 + 1 个 proptest（共 30 tests）

## Why / 为什么这么做

### Hamilton 积的矩阵形式

四元数乘法 `p ⊗ q` 是双线性的，可以写成矩阵-向量积：
- 固定 `p`，变化 `q`：`p ⊗ q = L(p) · q_vec`（左矩阵）
- 固定 `q`，变化 `p`：`p ⊗ q = R(q) · p_vec`（右矩阵）

推导从基定义：
```
(p⊗q)_w = p_w q_w − p_x q_x − p_y q_y − p_z q_z
(p⊗q)_x = p_w q_x + p_x q_w + p_y q_z − p_z q_y
(p⊗q)_y = p_w q_y − p_x q_z + p_y q_w + p_z q_x
(p⊗q)_z = p_w q_z + p_x q_y − p_y q_x + p_z q_w
```

按 `p_w, p_x, p_y, p_z` 提取系数得：

```
R(q) = ⎡ q_w  -q_x  -q_y  -q_z ⎤
       ⎢ q_x   q_w   q_z  -q_y ⎥
       ⎢ q_y  -q_z   q_w   q_x ⎥
       ⎣ q_z   q_y  -q_x   q_w ⎦
```

本 commit 用单元测试 `right_mult_matrix_matches_hamilton_product` 把这个矩阵跟 nalgebra 的 `*` 运算符结果直接比较，确保推导没错。

### 为什么是 R(δq) 而不是 L(δq) 或 L(q)

`q_new = q ⊗ δq`。固定 `δq` 看 `q` 的变化，所以是**右矩阵以 δq 为参数**：

```
∂(q ⊗ δq)/∂q = R(δq)
```

（符号记忆：`A ⊗ B` 对 A 求偏导是**把 B 写到 R() 里**，对 B 求偏导是**把 A 写到 L() 里**。）

### 归一化 Jacobian 的遗留问题

严格地，`q_new_normalized = (q ⊗ δq) / ‖q ⊗ δq‖`。归一化引入一个 4×4 投影 `(I − q̂·q̂ᵀ)` 把径向分量抹掉。**本 commit 跳过这一步**。

影响：
- 径向方向（即 q 整体缩放）的不确定度被 F 过度传播到 P
- 在 ‖q‖ 已经 ≈ 1 时这个过度量 → 0
- 在 predict 之后 `normalize_attitude` 会把 q 拉回单位圆，所以 P 的多余分量会被 Joseph form 或后续测量更新耗散

工业 EKF（PX4 ekf2 在切到 error-state 前、ArduPilot AP_NavEKF3）都曾用这个简化式，代价是收敛略慢，不影响最终稳定性。**M2 以后如果要迁到 error-state**（21 维，把 4D q 换成 3D θ），这整块会重写，所以现在不过早优化。

### determinant = 1 的 proptest 是个好信号

`R(unit quaternion)` 的行列式**严格等于 1**（右矩阵是 SO(4) 嵌入）。proptest 在 ω × dt 矩形里取 256 样本，每次都验证 `|det(∂q/∂q) − 1| < 1e-4`。这捕捉以下类 bug：
- R(q) 公式系数写错（几乎必会让 det 漂移）
- `quaternion_exp` 产生非单位 δq（漏了小角度 fallback）
- 浮点数 overflow 导致 block 异常

### 为什么 `build_transition_jacobian` 是一个独立函数

让调用方看到的就一条线：

```rust
let f = build_transition_jacobian(&state, &imu, dt);
let q = build_process_noise(noise_params, dt);
let p_next = predict_covariance(&p, &f, &q);
```

每次 M1.9b-1b/c/d 补新子块，只改 `build_transition_jacobian` 的实现，**调用方零改动、proptest 零改动**。这就是工程上的单职责 + 开闭原则。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 30 passed; 0 failed
  含 13 个 proptest × 256 样本

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

关键验证：
- `right_mult_matrix_matches_hamilton_product`：数值上 R(q)·p_vec 等于 `p * q` 到 1e-6
- `dq_dq_block_has_unit_determinant`：256 个 (ω, dt) 组合下 det(block) ≈ 1
- 全 workspace clippy `-D warnings` 依然绿

## Follow-ups / 遗留

依序完成其余 F 子块：

| 子步 | 子块 | 数学难度 |
|---|---|---|
| **M1.9b-1b** | `∂q/∂b_g` —— 4×3 | 中（四元数指数对 ω 的偏导） |
| **M1.9b-1c** | `∂v/∂q`, `∂v/∂b_a` —— 3×4, 3×3 | 中（旋转矩阵对 q 的偏导） |
| **M1.9b-1d** | `∂p/∂q`, `∂p/∂b_a` —— 3×4, 3×3 | 低（`½·R(q)·f·dt²` 的偏导） |

完成全部后 F 才真正"活"起来，P 的非对角项开始正确传播。
