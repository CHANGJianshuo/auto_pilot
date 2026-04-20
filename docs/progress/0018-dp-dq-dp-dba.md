# 0018 — F 矩阵的 ∂p/∂q 和 ∂p/∂b_a 块（最后两个非平凡块）

**Commit**: `704d544`  **Date**: 2026-04-20  **Milestone**: M1.9b-1d

## What / 做了什么

填 F 矩阵的最后两个闭式子块 —— 位置对姿态和加速度偏差的偏导：

- `∂p/∂q` —— 3×4，来自 `½·R(q)·f_body·dt²` 项
- `∂p/∂b_a` —— 3×3，闭式 `−½·R(q)·dt²`

这一步之后，**F 矩阵所有非平凡耦合都写完了**。

新增：
- `build_transition_jacobian` 写入两个新块（复用 `rot_jac` 和 `r_q`，避免重复计算）
- 2 个新 proptest（共 43 tests）

## Why / 为什么这么做

### 数学推导

predict 的位置积分：
```
p_new = p + v·dt + ½·a_ned·dt²      a_ned = R(q)·f_body + g
                                     f_body = accel_measured − b_a
```

位置同时受速度（线性）和加速度（二次）影响。对 q 求偏导只涉及二次项（速度不依赖 q）：

```
∂p_new/∂q = ½·dt² · ∂a_ned/∂q
          = ½·dt² · (∂R/∂q · f_body)
```

对 b_a 求偏导同理：
```
∂p_new/∂b_a = ½·dt² · ∂a_ned/∂b_a
            = −½·R(q)·dt²
```

### 复用 ∂v/∂* 的计算

`rot_jac = ∂R/∂q · f_body` 在 M1.9b-1c 里已经算过给 `∂v/∂q` 用。同样 `r_q = R(q)` 给了 `∂v/∂b_a`。本 commit **不重新计算**，直接复用：

```rust
let dp_dq = rot_jac * half_dt_sq;     // 重用 M1.9b-1c 的结果
let dp_dba = -r_q * half_dt_sq;       // 重用
```

单次 predict 计算 F 的开销：
- `R(q)`：1 次
- `∂R/∂q · f_body`：1 次（3×4 × 3 = 12 flops × 16 ≈ 200 flops）
- `L(q)`：1 次
- 4 个 Jacobian 矩阵块的 copy_from：24 × 4 = 96 flops

全部加起来 < 1 µs 在 Cortex-M7 上，1 kHz predict 预算 1 ms 的千分之一。性能完全够。

### 为什么 proptest tolerance 需要分开

| 块 | 容差 | 原因 |
|---|------|------|
| `∂v/∂b_a` | 1e-4 | 纯线性 `−R(q)·dt`，误差来自浮点 ulp |
| `∂v/∂q` | 5e-4 | R(q_new) ≠ R(q) 的 O(dt²) 漂移 |
| `∂p/∂b_a` | 1e-6 | 纯二次 `−½·R(q)·dt²`，小 dt 下极准 |
| `∂p/∂q` | 1e-5 | O(dt³) 漂移（R 一阶 × dt² = dt³ 误差） |

tolerance 设得**紧但不过紧** —— 足够让"符号写反"或"漏 ½"被立刻捕获，同时容纳浮点舍入。如果 tolerance 过松，bug 会偷偷溜过 proptest。

### 为什么这步后 M1.9b 就"算完了"

下表是我推 EKF 预测阶段以来的所有 F 块：

| 块 | 公式 | 里程碑 | 状态 |
|---|------|-------|------|
| ∂q/∂q | R(δq) | M1.9b-1a | ✅ |
| ∂q/∂b_g | -(dt/2)·L(q)[:, 1:4] | M1.9b-1b | ✅ |
| ∂v/∂q | (∂R/∂q · f_body) · dt | M1.9b-1c | ✅ |
| ∂v/∂b_a | -R(q)·dt | M1.9b-1c | ✅ |
| ∂p/∂v | I·dt | M1.9b-0 | ✅ |
| ∂p/∂q | ½·dt²·(∂R/∂q · f_body) | **M1.9b-1d** | ✅ |
| ∂p/∂b_a | -½·R(q)·dt² | **M1.9b-1d** | ✅ |

其它对角块（`∂v/∂v = I`, `∂p/∂p = I`, biases 自耦合 = I）都是 `kinematic_transition` 起手的 identity 贡献。**F 矩阵整体闭合**。

### M1.9b-1 完成的意义

这是 EKF **predict 半段**的完整数学闭环：
- 状态演化：`State::predict` ✓
- 协方差演化：`predict_covariance(P, build_transition_jacobian(...), build_process_noise(...))` ✓

一次 1 kHz 的 predict 循环现在可以做：
```rust
let f = build_transition_jacobian(&state, &imu, dt);
let q = build_process_noise(ProcessNoise::default(), dt);
state = state.predict(imu, dt);
p = predict_covariance(&p, &f, &q);
```

**P 矩阵的非对角项**现在开始从 F 自然生成（速度-姿态、位置-速度、姿态-陀螺偏差等物理相关）。等 M1.10 接入 GPS/mag/baro 测量更新后，EKF 就能真正收敛。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 43 passed; 0 failed
  含 19 个 proptest × 256 样本 = 4864 随机实例

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

所有 4 个 Jacobian 块都有一个对应的有限差分 proptest：
- `dq_dbg_block_matches_finite_difference` ✓
- `dv_dba_block_matches_finite_difference` ✓
- `dv_dq_block_matches_finite_difference` ✓
- `dp_dba_block_matches_finite_difference` ✓（新）
- `dp_dq_block_matches_finite_difference` ✓（新）

## Follow-ups / 遗留

- **M1.9c**：端到端测试 —— 一个完整的 predict 循环（state + covariance）跑 N 步，验证不变量（‖q‖=1、P PSD、数值稳定）
- **M1.10**：GPS position measurement update（EKF 的另一半 —— 测量更新环）
- **M1.11**：Magnetometer measurement update
- **M1.12**：Barometer measurement update

M1.9b-1 完成，M1 里最难的数学部分已过。剩下主要是测量更新（比 predict 简单得多）+ 驱动层（STM32H7 SPI 等硬件特定代码）。
