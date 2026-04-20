# 0006 — Priority 排序的 Kani 证明

**Commit**: `01c8c26`  **Date**: 2026-04-20  **Milestone**: M1.2

## What / 做了什么

给 `core-rtos::Priority` 增加：
- `Priority::ALL: [Priority; 8]` —— 所有变体的常量数组，给证明枚举用
- `Priority::rank() -> u8` —— 显式 match（避开 `as` cast）
- `Priority::from_rank(u8) -> Option<Priority>` —— 无 panic 的反向映射
- 4 个单元测试（往返、严格递增、越界 None、cmp 与 rank 一致）
- **5 个 Kani 证明**（`#[cfg(kani)]`）：
  - `check_ordering_is_total` —— 全序
  - `check_ordering_is_transitive` —— 传递性
  - `check_ordering_is_antisymmetric` —— 反对称
  - `check_rank_round_trip` —— `from_rank(p.rank()) == Some(p)` 对所有 8 个变体
  - `check_out_of_range_rank_is_none` —— `r ≥ 8 ⇒ from_rank(r) == None`

## Why / 为什么这么做

### 为什么证这些性质

工业级飞控的调度器一定要 Rate-Monotonic 式的静态优先级排序——任务优先级是**全序**、**无歧义**。如果偶然某两个 variant 被编译器重排（比如 repr 变更、新加一个 variant 放错位置），Kani 会立刻在 CI 里炸，**不用跑任何集成测试**。

具体证明价值：
| 性质 | 如果破坏会导致 |
|-----|---------------|
| 全序（totality） | 调度器无法决定谁先运行 → 优先级反转 |
| 传递性 | 路径依赖的排序 → 不确定行为 |
| 反对称 | 两个不相等的变体互为"≤" → 同等处理，饥饿风险 |
| `from_rank` 无 panic | 从外部（DDS 消息、寄存器）反序列化不会崩 |

前三条是"派生 trait 应当自动成立"——但 `derive(PartialOrd, Ord)` 的实现依赖 `#[repr(u8)]` 的声明顺序，一行改错就全破。Kani 把"这个契约始终成立"钉死。

### 为什么这次 Kani 秒级（49 ms）而上次（M1.1）要 1.8 s

- `Priority` 的域是 `u8` 里的 0..8，**8 个值**。
- f32 的域是 **2³² 个位**，加上 NaN/Inf 特殊情况和浮点理论。

SAT 求解器的工作量跟状态空间的 `log₂` 成比例——整数枚举对 CBMC 就是送分题。

### 为什么写 `from_rank` 返回 `Option` 而不是 panic

CLAUDE.md 的 firmware 规约第一条：**never panic**。`Priority::from_rank(99)` 如果 unwrap 或 panic，整个 FMU 重启。Option 让调用者可以优雅降级（比如 "unknown priority, treat as Background"）。

## How it's verified / 怎么验证的

```bash
# 容器内
$ cargo test -p core-rtos
running 4 tests
test tests::ordering_matches_rank ... ok
test tests::all_ranks_round_trip ... ok
test tests::out_of_range_rank_is_none ... ok
test tests::ranks_are_strictly_increasing ... ok
test result: ok. 4 passed

$ cargo kani -p core-rtos
SUMMARY:
 ** 0 of 71 failed
VERIFICATION:- SUCCESSFUL
Verification Time: 0.049186397s
Manual Harness Summary:
Complete - 5 successfully verified harnesses, 0 failures, 5 total.

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- M1.3 候选：`algo-fdir::HealthLevel` 状态机证明——类似的枚举推理，可直接复用这里的模式。
- 以后加新 `Priority` variant 时：
  1. 加到 enum
  2. 加到 `ALL` 数组
  3. 加到 `from_rank` match
  4. 运行 `cargo kani` 确保证明仍通过
- 上游改动风险：如果以后改成 `#[repr(u16)]` 或加 `#[derive(Serialize)]` 带不同顺序，Kani harness 会立刻发现。
