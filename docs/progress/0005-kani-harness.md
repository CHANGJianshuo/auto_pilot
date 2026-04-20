# 0005 — Kani harness（项目第一个形式化证明）

**Commit**: `e421dc2`  **Date**: 2026-04-20  **Milestone**: M1.1

## What / 做了什么

1. 在容器里装了 **Kani 0.67.0**（`cargo install kani-verifier` + `cargo kani setup`，后者下载 CBMC + nightly toolchain 约 2 GB）。
2. 重构 `State::normalize_attitude`：把退化检查从 `‖q‖` 改成 `‖q‖²`。**退化情况（NaN / Inf / 低于 floor）在进入 sqrt 之前就 early return**。
3. 在 `crates/algo-ekf/src/lib.rs` 尾部加了 `#[cfg(kani)] mod kani_proofs`，包含 1 个 `#[kani::proof]`：
   - `check_zero_quaternion_returns_none` — 证明零四元数一定被拒绝
4. `Cargo.toml` 的 `[workspace.lints.rust]` 声明 `unexpected_cfgs = { check-cfg = ['cfg(kani)'] }`，让 clippy 不因 `#[cfg(kani)]` 报 "unknown cfg" 警告。

## Why / 为什么这么做

### 为什么重构 `normalize_attitude`

原版把 `sqrt` 放在函数开头：

```rust
let norm = self.attitude.norm();   // <-- sqrt 总是先跑
if norm < FLOOR || !norm.is_finite() { return None; }
```

这意味着**任何** Kani 证明（哪怕是输入全具体的 zero-quaternion）都要分析 sqrt，而 Kani 不能建模 `libm::sqrtf`（x86 上用内联汇编，Kani 不支持 `TerminatorKind::InlineAsm`）。

新版把退化检查放在 sqrt 之前：

```rust
let norm_sq = q.w*q.w + q.i*q.i + q.j*q.j + q.k*q.k;   // 无 sqrt
if !norm_sq.is_finite() || norm_sq < FLOOR*FLOOR { return None; }
let norm = norm_sq.sqrt();                             // 只在合法输入上 sqrt
```

两个好处：(1) 退化路径可以被 Kani 静态证明。(2) 生产环境下零四元数直接短路，省一次 sqrt。

### 为什么只留一个具体-输入 harness

试过三个 harness：
- `check_zero_quaternion_returns_none`（全具体） → **1.8 s 通过**
- `check_nonfinite_component_returns_none`（符号 f32 + NaN 约束） → 运行 **20+ 分钟**没结束
- `check_below_floor_returns_none`（符号 f32 + 平方和小于阈值） → 同上，20+ 分钟卡死

CBMC 默认的 CaDiCaL SAT 求解器在 f32 浮点理论 + 非线性约束下性能崩塌。后两个 harness 搬到"慢 CI 车道"再处理，可能要切 `--solver=z3` 或用区间抽象。**256 样本 proptest 目前已覆盖这两条路径**，所以形式化缺口没有实际危险。

这是工程上常见的取舍：**形式化证明覆盖不做假设的部分，性质测试覆盖需要浮点算术的部分**。二者互补。

### 为什么值得装 Kani

- 以后会有很多"无 sqrt"的不变量可以证：状态机转移的可达性、buffer 越界、整型溢出、控制分配矩阵的维度、FDIR 状态机的死锁自由。
- 已经装好 CBMC + nightly toolchain，后续 harness 只需 `cargo kani` 一行跑。

## How it's verified / 怎么验证的

```bash
# 容器内
$ cargo kani -p algo-ekf
...
SUMMARY:
 ** 0 of 282 failed (8 unreachable)
VERIFICATION:- SUCCESSFUL
Verification Time: 1.8149605s

Manual Harness Summary:
Complete - 1 successfully verified harnesses, 0 failures, 1 total.

# proptest 不变
$ cargo test -p algo-ekf
test result: ok. 4 passed; 0 failed

# clippy -D warnings 不变
$ cargo clippy --workspace --all-targets -- -D warnings
Finished `dev` profile
```

## Follow-ups / 遗留

- **把 Kani 加入 Dockerfile**：现在 Kani 是容器运行时通过 `cargo install` 装的，重新 build 镜像会丢。下个 iteration 在 Dockerfile 第 4 节加一行 `cargo install kani-verifier && runuser -u dev -- cargo kani setup`。
- **CI 集成**：CI workflow 里加一个 `kani` job（单独 job，因为要下载 CBMC）。
- **慢车道 harness**：两个符号 f32 harness 保留为 TODO，等换 solver 或用 `assert!`/`kani::assert` 切分后再启用。
- **其他可证不变量**（优先级排序）：
  - `core-rtos::Priority` 的排序传递性 / 反对称性
  - `algo-alloc` 控制分配矩阵条件数下界（需无 sqrt 的公式）
  - `algo-fdir::HealthLevel` 状态机无死锁
  - MAVLink 解析器的 buffer 越界
