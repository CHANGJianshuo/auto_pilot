# 0014 — F 矩阵的"平凡动学"子块（dp/dv = I·dt）

**Commit**: `f9c204c`  **Date**: 2026-04-20  **Milestone**: M1.9b-0

## What / 做了什么

给状态转移 Jacobian `F` 填上**唯一一个纯线性**的子块：位置相对于速度的偏导 `∂p/∂v = I · dt`。其它子块（attitude ↔ velocity、velocity ↔ accel_bias、attitude ↔ gyro_bias）继续用 identity，等 M1.9b-1 推导它们的闭式解。

新增：
- `pub fn kinematic_transition(dt_s) -> Covariance`
- 3 个单元测试 + 2 个 proptest（本 crate 共 25 tests）

## Why / 为什么这么做

### 为什么从 dp/dv 开始

这一块是 F 里**唯一能用代数而不是微分得到**的：

```
p_new = p_old + v · dt + ½ · a · dt²
⇒ ∂p_new / ∂v = I · dt   (准确，非线性项不涉及)
⇒ ∂p_new / ∂p = I         (自动)
```

其它块要做 Hamilton 乘积的偏导、旋转矩阵对四元数的偏导，都是非线性推导。先把纯线性的挑出来落地，让 proptest 可以检测到"predict_covariance 不再是 F=I 平凡传播"。

### 为什么要一个专门的 proptest 检测 cross-block

`kinematic_f_transfers_velocity_uncertainty_to_position` 检验：

```
P_next = F · P · Fᵀ + 0
       = [identity + dp/dv block] · diag(σ²) · [identity + dp/dv block]ᵀ
       ⇒ P_next[p, v] ≈ dt · σ_v²  (cross-block appears)
       ⇒ P_next[p, p] ≈ σ_p² + dt² · σ_v²  (position variance grows from velocity)
```

这个 cross term 是 EKF **正确工作**的必要条件：如果它不出现，GPS 位置更新就无法反推出速度修正。本测试确保 F 矩阵真把"位置不确定度由速度不确定度喂养"这件事做对了。

### 为什么用 `fill_with_identity` + `scale_mut`

nalgebra 的 `fixed_view_mut` 得到的是**借用**，不能直接写 `block = I * dt`（那是赋值新对象）。必须用原地操作：

```rust
let mut dp_dv = f.fixed_view_mut::<3, 3>(P_START, V_START);
dp_dv.fill_with_identity();  // 原地写成 I
dp_dv.scale_mut(dt_s);       // 原地乘标量
```

这避免了构造 3×3 临时矩阵 + 赋值拷贝，也让 clippy 开心（没有索引 / unwrap）。

### 为什么 M1.9b 再分子步

完整 F 的推导是：

| 块 | 维度 | 复杂度 |
|---|---|---|
| ∂q_new/∂q | 4×4 | 四元数 Hamilton 乘积对 q 的偏导。最难 |
| ∂q_new/∂ω = ∂q_new/∂b_g | 4×3 | 四元数指数对 ω 的偏导 |
| ∂v_new/∂q | 3×4 | 旋转矩阵 R(q) 对 q 的偏导 |
| ∂v_new/∂b_a | 3×3 | -R(q) · dt（已知 q 后平凡） |
| ∂p_new/∂v | 3×3 | **I · dt（本 step 已做）** |
| ∂p_new/∂q | 3×4 | 二阶项 ½·R(q)·f·dt² 对 q 的偏导 |
| 其它 | 对角 I | bias / mag / wind 自相关 |

M1.9b-1 做上面表里的剩下 5 块。每块 30-50 行代码 + 单元数值验证 + 一个 proptest。

本 commit 是它们的地基 —— 函数签名、测试样式、enforce_symmetry 回路都已固定。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 25 passed; 0 failed
  含 10 个 proptest × 256 样本

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M1.9b-1**：填 6 个非平凡 Jacobian 子块（大约 +200 行）
- `enforce_symmetry` 在 cross block 生效之前都不改变 P，M1.9b-1 之后才 visible
- `kinematic_transition` 现在每次调用都重建 matrix —— 1 kHz 下是 24×24=576 个 f32 写入 ≈ 2 us，可以接受。若成为瓶颈，改成 thread-local 静态 mut 缓存
