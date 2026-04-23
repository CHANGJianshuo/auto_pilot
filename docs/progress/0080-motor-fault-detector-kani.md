# 0080 — MotorFaultDetector Kani 形式化（M20d）

**Commit**: `3eed5b4`
**Date**: 2026-04-24  **Milestone**: M20d

## What / 做了什么

M20a-c 把 motor-fault detection 做到了 end-to-end 能跑（SITL 验证）。M20d 把它**形式化证明**，closing 掉了 M20 task 的最后一格。

核心重构：

```rust
pub enum DetectorDecision {
    Quiet,              // residual 低于阈值
    Inconclusive,       // 残差大但没能归因到具体电机
    BestMatch(u8),      // idx 号电机是最佳故障假设
}

impl MotorFaultDetector {
    pub fn observe(&mut self, cmd, ω̇, E, J⁻¹) -> HealthLevel {
        let decision = /* numeric scoring (f32) */;
        self.apply_decision(decision)
    }

    pub fn apply_decision(&mut self, decision: DetectorDecision) -> HealthLevel {
        /* pure state transition — no f32 */
    }
}
```

这次 split 之后，Kani 的证明对象变成 `apply_decision` —— **纯状态转移函数**。CBMC 不需要碰任何 f32。

### 6 个新 Kani 证明（workspace 28 → 34）

| # | 证明 | 核心不变量 |
|---|---|---|
| 1 | `apply_decision_alive_is_monotone` | `alive[i]` 一旦为 false，任何 decision 都不能让它回 true |
| 2 | `apply_decision_level_is_monotone` | `HealthLevel.severity()` 永不降低 |
| 3 | `dead_count_is_at_most_four` | dead_count ≤ 4（整型溢出 guard）|
| 4 | `quiet_resets_persistence` | `Quiet` 零化所有 persistence |
| 5 | `inconclusive_does_not_mutate_state` | `Inconclusive` 不改 alive / persistence / level |
| 6 | `bestmatch_resets_non_matching_counters` | `BestMatch(idx)` 只保留 idx 号 counter，其他 3 个归零 |

## Why / 为什么这么做

### 为什么要为 detector 加形式化

*alive monotonicity* 是整个 motor-FDIR 安全模型的**基石**：

- 整个 `app-copter::rate_loop_step` 里的 AND-monotone 更新逻辑（`motor_alive[i] &&= det.alive()[i]`）依赖 detector 自己是 monotone 的
- 如果未来有人"重构 detector，让它在残差消失时 un-declare 电机"，直观上看似合理但**会废掉**整个 fail-safe 模型：已经切换到 failover allocator 的飞控会突然切回 4-motor allocation，但物理电机仍然死着 → 直接 crash

写成单元测试（M20a 里有）只能验证**今天的实现**不 un-declare。形式化证明：**任何未来能通过编译的改动**要么保持这个不变量，要么 proof 失败 → 评审者必须面对这个问题。

相比 unit test 的作用是**回归 guard**（这段代码昨天这样，今天还这样），Kani 的作用是**契约 guard**（这段代码今天是这样，**将来**也必须这样）。差异关键。

### 为什么把 observe 拆成两半

CBMC（Kani 的 backend）对 f32 的建模虽然 sound 但**慢且不完整**：

- 数值稳定性分析涉及区间算术，比整数推理慢 100×
- sqrt / 除法的符号化需要 SMT 级 float theories，很多求解器支持不完整
- M20a 的 `observe` 里有 ~20 乘法 + 1 sqrt + 循环 4 次 + 比较 → CBMC 可能几分钟才收敛，或直接超时

典型做法：**把决策点和状态机分开**。决策部分（"哪个电机最可疑"）用 f32 算，但它的**结果**只有三种可能：`Quiet / Inconclusive / BestMatch(idx)`。状态机只需要按这三种 outcome 执行写操作 —— **这里是整数/布尔，CBMC 秒出**。

决策本身我通过**单元测试 + SITL**覆盖（M20a 有 6 个单元测的已经覆盖了各个 idx 的 attribution），Kani 只负责"不管 decision 怎么算出来的，只要是这三种 variant 之一，状态机一定正确"。

这种 split 模式我们已经用过三次：
- M1.14 `SensorRejectionCounter` 的单步 observe → 同样避开长序列 CBMC 爆炸
- M16a/b/c 的 `RtlPhase::advance` / `LandingState::advance` / `TakeoffState::advance` → pure transition enum
- M20d 现在

套路已经成型，未来任何需要"f32 决策 + 离散状态"的算法都可以依这个模板加 Kani 证明。

### 为什么 `persistence` 字段的类型是 `[u32; 4]` 而不是 `heapless::Vec<u32, 4>`

固定大小数组 + index iteration → Kani 能**静态展开**，CBMC 用 4 个 SMT 变量就表示完整状态。`heapless::Vec` 带长度字段、有 `push` / `pop` 逻辑，Kani 要证明"长度永远是 4" 就多一层。

简单永远胜过灵活——在 no_std safety-critical 代码里这个取舍几乎总是对的。

### 为什么 `DetectorDecision::Inconclusive` 是单独 variant，不合并进 `Quiet`

语义差别：
- **Quiet** = "这一 tick 没有故障证据" → 重置 persistence（清除可能的假阳性 streak）
- **Inconclusive** = "有故障证据，但不知道是哪个电机（比如所有电机都已标死）" → **不动** persistence

合并后行为等价吗？不等价。想象：所有电机都已标死 + 真正的强扰动（碰撞）。Quiet 路径会重置 persistence，但其实我们不该重置——我们只是**无法归因**。保留 streak 让未来的更高层 FDIR 还能看到"这个电机曾经快要被标死"。

这两个 variant 的区分也让 `check_inconclusive_does_not_mutate_state` 成为可证的独立 proof——一旦合并就少一个 guard。

### 为什么每个 proof 单独成函数而不是合并成"大断言 proof"

CBMC 出错信息指向的是**单个 `#[kani::proof]` harness**。粒度细：某条 proof 失败时评审者直接看到"alive_is_monotone 断言失败" —— 立刻定位到问题。粒度粗："big_proof 失败"需要从长串断言里挑出哪条失败了。

同时 harness 之间的 SMT 独立缓存 —— CI 如果 kani-verify rerun 时某一个 proof 不受改动影响，可以直接 skip。

## How it's verified / 怎么验证的

```bash
$ cargo kani -p algo-fdir
# 原 6 + 新 6 = 12 harnesses
# Expected verification time: < 100 ms total on laptop
```

CI job `kani` 已经包含 `algo-fdir`（`.github/workflows/ci.yml` 里的 for-crate 循环），零配置拾取新 harness。

Workspace Kani 证明总数：**28 → 34**，across 5 crates。

Unit tests (algo-fdir) 从 25 → **25**（refactor 不加测，因为外部语义不变）。SITL (sim-hil) 从 43 → **43**（同上）。

## Follow-ups / 遗留

- **M20e — 多电机失效 detector**
    - 当前 scoring 逻辑假设一次挂一个。如果同一 tick 两个电机同时挂，残差方向是**混合**的 → `BestMatch(idx)` 只能选一个，另一个漏。修法：detection 走完一遍 declare 一个后，再 run 一遍 score remaining，直到 Quiet 为止。
    - 多-declaration iteration 的 Kani 证明会比单-declare 复杂一层，但同样 pattern 可用。
- **检测延迟 tuning**：默认 `n_ticks_to_declare = 50` 是估计值。M20c 的 SITL 实测延迟 ~80 ms（motor lag + LPF + persistence），比纯 persistence 大 30 ms。理想硬件测试后 re-tune。
- **Graded failure** 软失效：当前 threshold 只对"完全失电"灵敏，对"50% 退化"刚过阈值、60% 不触发。
- **MAVLink MOTOR_INFO**：detector 输出的 `alive()` + `persistence()` 导出给 GCS，让飞行员看到每个电机的"health score"。
- **位置控制器 yaw-aware handoff (M21)**：3-motor allocation 后飞机会 yaw-spin → position controller 的 body-frame 指令漂 → xy drift。需要专门的 Mueller-style handoff。
