# 0009 — State ↔ 24-D 向量序列化

**Commit**: `33fdc0d`  **Date**: 2026-04-20  **Milestone**: M1.5

## What / 做了什么

给 `algo-ekf::State` 加了"结构体 ↔ 24 维向量"的双向映射，是 EKF 协方差矩阵与雅可比矩阵的数据地基。

新增：
- `pub mod idx` —— 8 对（start, len）索引常量，覆盖 8 个状态分组：
  ```
  q         [0..4)   quaternion (w, i, j, k)
  v_ned     [4..7)
  p_ned     [7..10)
  gyro_bias [10..13)
  accel_bias[13..16)
  mag_ned   [16..19)
  mag_bias  [19..22)
  wind_ne   [22..24)
  ```
  `idx::TOTAL == STATE_DIM == 24` 有单元测试保证。
- `pub type StateVector = SVector<f32, 24>` —— 栈分配、无 `std`
- `State::to_vector()` / `State::from_vector(&StateVector)`
- 4 个新测试（总测试数 4 → 8）：
  - `idx_layout_totals_state_dim` —— 索引范围连续且总和 = 24
  - `default_state_round_trips`
  - `state_vector_round_trip` —— **proptest 256 样本**
  - `to_vector_respects_layout` —— **proptest 256 样本**，验证每个字段在正确索引

## Why / 为什么这么做

### 为什么要打包成一维向量

EKF 的数学层自然表达是 24×24 协方差矩阵 `P`、24×24 状态转移 `F`、24×M 测量矩阵 `H`。这些矩阵按**行列索引**操作状态分量。如果每次都要写：

```rust
// 坏写法
let gyro_bias_cov = P_attitude_to_gyro_bias_3x3;
```

代码里会散落几百个魔法数和起止偏移。我们把它们**全部集中到一个 `idx` module**，然后：

```rust
// 好写法
let slice = P.fixed_rows::<3>(idx::GYRO_BIAS_START);
```

任何数学公式里出现的 start/len 都能一眼对上状态向量文档。

### 为什么用 `fixed_rows::<N>(start)` 而不是 `v[i]`

- `v[i]` 触发 clippy `indexing_slicing = deny`（CLAUDE.md 的 firmware 规约）
- `fixed_rows::<N>(start)` 是 nalgebra 的 block view，编译期固定 N，调用一次而不是 N 次
- 语义上更强：**这是一个 N 维子向量**，不是 N 次独立访问

### 为什么四元数存 `[w, i, j, k]` 而不是 nalgebra 默认的 `[i, j, k, w]`

**EKF 文献惯例是 w 在最前**（Hamilton 约定、ArduPilot EKF3、PX4 ekf2 都这样）。后续所有 Jacobian 公式（`∂f/∂q` 等）对应的都是 w-first 序。我们在 `to_vector` 里显式把 `attitude.w` 放到 `v[Q_START]`，以后写雅可比矩阵时索引能对上。

### `libm::sqrtf` 替换 `f32::sqrt`

之前的 `normalize_attitude` 用 `norm_sq.sqrt()`，在 `no_std` + `libm` feature 下需要显式 `use num_traits::Float` 才能得到 `.sqrt()` 方法。加上这个 import 不干净——`num_traits` 为了一个方法单独导入打破 no_std 清爽度。

直接用 `libm::sqrtf(norm_sq)` 解决：libm 已经是 workspace dep，这是它的本职工作。

### Round-trip proptest 而不是 unit

Round-trip 是"每对 `to/from` 是互逆"的结构性质——proptest 的 256 样本能发现任何字段漏写、索引错位、长度不符的 bug。这种 bug **不会被具体单元测试发现**（因为没人能写出正确索引的 24 个字段的手写 case）。

### 尚未做形式化证明

没给 `to_vector/from_vector` 加 Kani harness，因为：
1. `idx::TOTAL == 24` 等静态关系已经由 `#[test]` 断言在编译/运行期校验
2. 函数体里只有赋值和 `Copy`，没有 panic 路径（`fixed_rows` 的 bounds 在 const generic 下是**编译期**决定）
3. 加 Kani 没有新信息，浪费 CI 时间

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 8 passed; 0 failed  (含两个 256-sample proptest)

$ cargo clippy --workspace --all-targets -- -D warnings
Finished

$ cargo kani -p algo-ekf
VERIFICATION:- SUCCESSFUL (1.66s, 274 internal checks, 0 failed)
```

## Follow-ups / 遗留

- **协方差矩阵 `PCovariance = SMatrix<f32, 24, 24>` 的 PSD 保持**——下一步 EKF predict 时重点。
- **雅可比矩阵生成**：写 `∂f/∂x` 时就是要用 `idx` 常量访问分组子块。
- **序列化到外部**（MAVLink LOCAL_POSITION_ESTIMATE、日志文件）：后续 `comms-mavlink` 里需要。
