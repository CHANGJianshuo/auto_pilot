# 0067 — nn-runtime：residual policy + safety envelope

**Commit**: `44b6ce5`  **Date**: 2026-04-23  **Milestone**: M11a

## What / 做了什么

`nn-runtime` 从"空壳"升级到**可用 residual-policy 骨架**。plan.md 第三大创新点（NN/RL onboard）首次落地，符合用户约束"纯 sim/host 可测"：

```rust
let backend = AffineBackend::new(weights, bias, output_clamp);
let mut policy = ResidualPolicy::new(backend, SafetyEnvelope::small_multirotor_default());

let features = FeatureVector::from_state(pos_err, vel, rpy);
let residual = policy.predict(&features, current_velocity)?;
// residual 是 NED accel 修正量，加到 NMPC 输出上
```

三个独立部件：

1. **Schema**：`FeatureVector([f32; 9])` = (pos_err_ned, vel_ned, rpy_rad)，`Residual(Vector3)` = NED accel 修正
2. **`InferenceBackend` trait**：`.predict(&features) -> Result<Residual, Error>`，确定性
3. **`AffineBackend`**：reference impl `y = W·x + b` + 输出 clamp，既是测试基线也是将来真模型对比的 sanity
4. **`SafetyEnvelope`** + **`EnvelopeReject`** enum：拒绝 non-finite、accel 过大、速度 runaway（velocity projection 100 ms）
5. **`ResidualPolicy<B>`**：绑 backend + envelope，自带 `reject_count` 滚动计数、typed `PolicyError<E>`

**11 新测试**，workspace **244 tests 全绿**，`no_std` + thumbv7em 编译过。

## Why / 为什么这么做

### 为什么先做"residual learning"而不是"end-to-end RL"

三个原因，按重要性：

1. **稳定性可证**：NMPC 本身有数学稳定性证明（M9.x 做完）。NN 输出一个**修正量**，如果 NN 挂掉/输出零/output_clamp 卡死，控制器还是 NMPC，飞机还是飞。端到端 RL 替代控制律需要重新证明整体稳定，工程量 + 验证风险数量级高。

2. **data efficiency**：residual 只需学"模型误差"—— drag 系数、motor lag、payload offset 这类小偏差。端到端要学"如何控制一架飞机"，训练数据需求大得多。

3. **可验证**：所有 residual ∈ 规定 envelope → 对比 NMPC 输出的差异可以 log、可视化、quantify。端到端 NN 输出里看不出"这是响应什么"。

Swift / UZH RPG 的论文路径就是这样。他们 agile maneuver 的 policy 也不是纯端到端——基座是 differential flatness-based controller + NN 补残差。

### 为什么 `InferenceBackend` 是 trait 不直接 tract

tract 0.23.0 系列只有 dev 版本（`0.23.0-dev.5`）发在 crates.io。把 dev 依赖加进生产 workspace：

- **Semver 不稳**：dev 版本没有 API 稳定承诺
- **CI 锁定困难**：更新 Cargo.lock 可能拉到破坏性改动
- **上游 bug 暴露更多**：dev 有未修 issue

Trait 抽象解耦 **"policy 架构"** 和 **"具体推理引擎"**：

- M11a：trait + 参考 `AffineBackend`（纯 Rust，零外部 dep）
- M11b：加 `TractBackend` 实现，tract 到 stable 1.0 再集成，或者 candle / onnxruntime
- 下游代码（`ResidualPolicy`、SITL 测试、将来的 outer_step 集成）**不改一行**

这是 Rust trait-based DI 的典型用法。交换推理引擎成本 → 一个 trait impl。

### 为什么 feature vector 长度 fixed 编译时常量

```rust
pub const FEATURE_LEN: usize = 9;
pub struct FeatureVector(pub [f32; FEATURE_LEN]);
```

而不是 `Vec<f32>`：

- **no_std 兼容**：`Vec` 需要 alloc
- **栈分配**：每次调用 `predict` 不分配
- **schema 稳定**：训练时 9 维、部署时也得 9 维。编译期长度避免 "运行时 shape mismatch" 类的隐蔽 bug
- **backend 可预分配**：知道输入 shape 固定，weights 矩阵可以是 `[[f32; 9]; 3]`（栈），不需要 `Matrix<Dynamic>`

代价：改 feature schema 要同时改 FEATURE_LEN 和训练 pipeline。这**就是 schema evolution 的正确成本**—— firmware + 训练 pipeline 版本必须一致。

### 为什么 `SafetyEnvelope::check` 需要 current_velocity

只看 residual magnitude 不够：residual = (1, 0, 0) 看起来 safe，但如果 vehicle 已经以 15 m/s 飞行，再加速度 1 m/s² 过 100 ms 后速度 > 15，冲出 envelope。

逻辑：
```rust
projected_velocity = current_velocity + residual * 0.1  // 100 ms horizon
if projected.norm() > max_velocity_m_s { reject }
```

**100 ms horizon 是合理缓冲**：
- 太短（10 ms）→ 检查过于"最近未来"，不能抓趋势
- 太长（1 s）→ 假设 residual 在未来 1s 恒定，忽略控制器会继续修正的事实

100 ms ≈ 位置环 5 tick，足够看到短期加速但仍在位置控制响应时间内。

### 为什么 `ResidualPolicy` 自带 `reject_count`

Onboard policy 的最重要**可观测性**：
- 如果 `reject_count` 持续增长 → NN 输出频繁超界 → 模型有问题（过拟合、训练分布 vs 飞行分布 mismatch、参数损坏）
- 通过 `HealthMsg` 发到 GCS → 操作员能看到
- 阈值告警：1 秒内 > 50 rejects → 建议 disable NN 退回 pure NMPC

`saturating_add(1)` 永远不溢出。`reset_reject_count()` 在 mode switch 时调。

### 为什么 `PolicyError` 是 generic enum

```rust
pub enum PolicyError<E> {
    Inference(E),
    Envelope(EnvelopeReject),
}
```

`E` 来自 `InferenceBackend::Error` —— 每个 backend 有自己的错误类型（tract 的 shape error、candle 的 tensor error、affine 的 `AffineError` 空 enum）。generic 让 `PolicyError` 包装任何 backend 的 error 不信息损失。

调用方 `match` 时可以针对 backend-specific error type 分开处理：

```rust
match policy.predict(&f, v) {
    Ok(r) => apply(r),
    Err(PolicyError::Envelope(EnvelopeReject::VelocityExceeded { .. })) => log_warn(),
    Err(PolicyError::Inference(my_backend_error)) => log_error(my_backend_error),
    Err(other) => {}
}
```

### 为什么 reference backend 是 affine 不是 RBF / MLP

设计期选的：
- **affine**：线性层，1 个 matmul + 1 个 add。闭式数学。简单到能手算验证。
- **RBF**：需要 sqrt + exp，每轮重计算
- **MLP**（2+ 层）：需要激活函数（relu/tanh），引入非线性 debug 复杂度

Affine **绝不是生产用**——它只是参考实现证明 trait 合理。真 policy 用 MLP / transformer，通过 TractBackend 加载。但 affine 的存在保证：

1. 任何新 backend 的单元测试 + integration test 已经过一遍 trait（不用 mock）
2. SITL 集成能在 affine 基线上先跑通（residual = 0 effectively）
3. 回归 test "backend 能调用 trait 正确输出" 是常驻的

### 为什么 envelope 不做 attitude check

当前 envelope 只看 accel 和 velocity，不看 tilt（尽管 `max_tilt_rad` 在 struct 里）。原因：

- **residual 是 accel 输出，不是 attitude**。tilt 由后续 attitude 控制器决定（NMPC accel → force balance → desired tilt）
- 如果要检查 "residual 会导致 tilt > max"，需要知道质量、推力、力平衡数学 → 把 `accel_to_attitude_thrust` 的逻辑搬到 envelope 里 → 耦合
- 现实中 tilt 由 `max_accel` 间接约束：accel 小 → tilt 小

`max_tilt_rad` 字段保留给未来 envelope 扩展（可能加个 `check_with_mass_info()` 重载）。现在是 unused field——但 struct 文档里列出来让读者知道意图。

## How it's verified / 怎么验证的

```bash
$ cargo test -p nn-runtime
11 passed:
  feature_vector_from_state_ordering
  affine_zero_backend_outputs_zero
  affine_backend_applies_weights_and_bias
  affine_backend_is_deterministic
  affine_backend_clamps_output
  envelope_accepts_safe_residual
  envelope_rejects_non_finite
  envelope_rejects_too_large_accel
  envelope_rejects_velocity_runaway
  residual_policy_zero_backend_accepted
  residual_policy_counts_rejections

$ cargo test --workspace
244 passed

$ cargo build -p nn-runtime --lib --target thumbv7em-none-eabihf
Finished (no_std ok)

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿
```

## Follow-ups / 遗留

- **M11b TractBackend**：等 tract 发 1.0 stable 或 fork 一个稳定分支，加 `tract-onnx` dep（behind feature），从 `.onnx` 字节加载模型，impl `InferenceBackend`。
- **M11c residual hooking into NMPC outer loop**：新 `PositionController::MpcResidual(mpc, policy)` variant；NMPC 出 `accel_cmd`，policy 出 `residual`，相加后送去 force-balance。
- **M11d SITL residual learning demo**：affine backend 手工设置"模型阻力估计"weights，realistic sim 下 NMPC + residual 应当比纯 NMPC 跟踪误差更小。
- **Feature schema 扩展**：加 wind 估计、motor tachometer rpm、电池电压。增加 `FEATURE_LEN` 到 16 一档。
- **stochastic policies**：将来 RL 训练的 policy 可能需要 action sampling。trait 改成 `fn predict(&self, f, rng: &mut Rng) -> ...`，或加独立 `StochasticBackend` trait。
- **多 model ensembling**：3 个 backend 投票，median 出 residual → 鲁棒 safety。
- **property test**：随机 weights + features → residual.norm() 应保持在 output_clamp 以内。
- **Kani 证 envelope invariants**：`envelope.check(r, v)` 返 `Ok(r')` 时 `r'.norm() ≤ max_accel_m_s2`，形式化证。
