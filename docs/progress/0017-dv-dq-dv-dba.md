# 0017 — F 矩阵的 ∂v/∂q 和 ∂v/∂b_a 块

**Commit**: `06c32b7`  **Date**: 2026-04-20  **Milestone**: M1.9b-1c

## What / 做了什么

再填两个 F 子块，覆盖速度对姿态和加速度偏差的线性化：

- `∂v/∂q` —— 3×4 块，通过旋转矩阵对四元数的偏导生成
- `∂v/∂b_a` —— 3×3 块，闭式 `−R(q)·dt`

新增：
- `pub fn rotation_matrix(q) -> Matrix3<f32>` —— body→world 旋转矩阵
- `pub fn rotation_jacobian_wrt_q(q, v) -> Matrix3x4<f32>` —— `∂(R(q)·v)/∂q`
- `build_transition_jacobian` 写入两个新块
- 4 个单元测试 + 2 个有限差分 proptest（共 41 tests）

## Why / 为什么这么做

### 闭式推导

```
a_ned  = R(q) · (accel − b_a) + g
v_new  = v + a_ned · dt
```

固定 q 和 accel、扰动 b_a：
```
∂a_ned/∂b_a = −R(q)
⇒ ∂v_new/∂b_a = −R(q) · dt
```

固定 b_a、扰动 q：
```
∂a_ned/∂q = ∂R(q)/∂q · f_body   (3×4，f_body = accel − b_a)
⇒ ∂v_new/∂q = ∂R(q)/∂q · f_body · dt
```

`R(q)` 的显式形式：
```
R(q) = ⎡ 1−2(y²+z²)   2(xy−wz)    2(xz+wy)  ⎤
       ⎢ 2(xy+wz)     1−2(x²+z²)  2(yz−wx)  ⎥
       ⎣ 2(xz−wy)     2(yz+wx)    1−2(x²+y²)⎦
```

每个分量的 `∂R_{ij}/∂q_k` 是 w/x/y/z 中至多一次线性组合，可以手写。`rotation_jacobian_wrt_q` 把四个 `(∂R/∂q_k) · v` 向量拼成 3×4 矩阵。

### 为什么 `∂v/∂q` 最难验证

闭式公式 × 4 列 × 3 行 = 12 个表达式，每个都是 `w, x, y, z, v_x, v_y, v_z` 的多项式。任何一个符号写反就会导致 EKF 在有姿态误差 + 非零比力时发散。

**有限差分 proptest** 是唯一现实的保护：
```rust
q_plus = q + δq                // 小扰动
v_predict(q_plus) - v_predict(q)   ≈   (∂v/∂q) · δq    // 到 O(δq²)
```

随机 256 次，在 `dt ∈ [1ms, 20ms]`、`‖δq‖ ≲ 0.02` 范围内，**每次差值 < 5e-4**。这就把 12 个系数全验了 —— 任何一个错都会有组合能触发差值放大。

### 为什么把 `rotation_jacobian_wrt_q` 暴露为公开 API

后续 `measurement update` 里，测量函数 `h(x)` 对 q 的 Jacobian 也要用这个模式（例如磁力计观测 `y = R(q)·m_world + b_m` 对 q 的偏导）。重用同一个函数保证一致。

### `rotation_matrix_unit_quaternion_is_orthogonal`

单位四元数生成的旋转矩阵必须满足 `R · Rᵀ = I`。用 90° 绕 z 旋转的具体四元数 `(√½, 0, 0, √½)` 跑数值验证——**结构测试**，保证 `rotation_matrix` 公式没把某个系数的 2× 丢了。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 41 passed; 0 failed
  含 17 proptest × 256 = 4352 随机实例

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

关键断言：
- `dv_dba_block_matches_finite_difference`：256 随机 (δb_a, dt) 下 Δv ≈ block·δb_a 到 1e-4
- `dv_dq_block_matches_finite_difference`：256 随机 (δq, dt) 下 Δv ≈ block·δq 到 5e-4
- 两者的 tolerance 差一个数量级反映的是 dv/dq 有 O(dt²) 的二阶项（R(q_new) vs R(q) 的小差），dv/db_a 是纯线性

## Follow-ups / 遗留

- **M1.9b-1d**：`∂p/∂q`, `∂p/∂b_a`（位置对姿态/加速度偏差的偏导，来自 ½·R·f·dt² 项）—— 最后一个非平凡块
- `rotation_matrix_unit_quaternion_is_orthogonal` 可以升级为 proptest（任意 unit q 都满足正交）
- 后续 EKF measurement update（M1.10+）的 H 矩阵会大量复用 `rotation_jacobian_wrt_q`
