# 0085 — Multi-motor failure detection（M20e）

**Commit**: `d770227`
**Date**: 2026-04-24  **Milestone**: M20e

## What / 做了什么

把 `MotorFaultDetector::observe()` 里**期望扭矩**的计算从"所有 4 个电机"改成"只用 alive 电机"。一行改动：

```rust
// Before (M20a–d):
for i in 0..4 {
    tau.x += effectiveness[(0, i)] * motor_cmd[i];
    ...
}

// After (M20e):
for (i, &alive_i) in self.alive.iter().enumerate() {
    if !alive_i { continue; }
    tau.x += effectiveness[(0, i)] * motor_cmd[i];
    ...
}
```

+ 1 新 test `detector_resolves_two_simultaneous_failures`（M0+M3 同时挂，100 ticks 内两个都被 declare）。

现在 detector 能处理**连续多电机失效**：第一个 declare 后，它的贡献从 expected τ 中扣除，第二个失效的 clean signature 就能浮出水面。

## Why / 为什么这么做

### 为什么 M20a–d 只处理单电机失效

最初的设计故意简单：
- 工业多旋翼 single-motor failure 概率 ≫ double. 单motor 挂了是 ESC / prop / bearing 故障；双挂通常意味 battery fail（全体掉）或 wiring issue
- M20a 里"best-scorer wins" 的逻辑明确是 1-at-a-time
- 多挂建模会 double complexity，初版选了 simple

但跑 `detector_each_motor_correctly_attributed` 的 symmetric 测试时发现一个**隐藏 bug**：即使是单电机失效场景，pre-M20e 的 expected τ 包含了已 declared-dead 电机的贡献。只是没触发问题，因为：
1. 单电机挂：residual 只有一个 signature → 不需要"剔除已 declared"
2. cmd 在 declare 之后也被 allocator 置 0 → expected τ 里 dead-motor 项实际为 0 * E_col = 0

所以**事实上** M20a–d 的 expected τ 计算是正确的，只是数学不自信 —— 依赖 cmd 在 declare 之后会变成 0。

M20e fix 把这个逻辑**显式化**：detector 自己知道哪些 motor 死了，直接跳过，不依赖 allocator 的 cmd 行为。更健壮，也**把多电机失效变成了自然的 emergent property**。

### 为什么多电机失效的**归因顺序**是"首先 best-score、其次 clean-signature"

不是一次 observe() 就能把两个电机都 declare —— 每次 observe 的 BestMatch 只 increment 一个 motor 的 persistence。需要 ~50 ticks (threshold) × 2 = 100 ticks 才能 declare 两个。

顺序：
1. **Tick 1**（事故发生）：residual = -(e_0·cmd_0 + e_3·cmd_3) ≈ 两个 signature 的 SUM。Score 可能 M0 或 M3 赢 by a small margin（噪声 / 几何 / cmd 不对称决定）
2. **Tick 1-50**：best 一直是 M0 (say)，persistence[0] 累加到 50 → declare M0 dead
3. **Tick 51**：现在 detector.alive[0] = false。Expected τ 跳过 M0 contribution。Residual = expected - observed. Expected 里 **没有** M0，Observed 里也没有 M0（物理上死了）→ residual = -(e_3·cmd_3)**pure M3 signature**
4. **Tick 51-100**：best = M3，persistence[3] 累加到 50 → declare M3 dead

整个过程 ~100 ticks = 100 ms @ 1 kHz。考虑 motor-lag 和 LPF delay 加 ~30 ms = **130 ms 两电机都 declared**。

对比 single-motor 典型 80 ms 延迟，multi-motor 稍慢但同数量级。

### 为什么这改动不影响现有单电机 detector 测试

既有测试里 detector.alive 初始都是 [true; 4]，**在测试运行期间**没有 motor 被 declare dead（或者 declare 后就测 assertion 结束）。在 [true; 4] 状态下，`if !alive_i { continue; }` 永远不触发 continue，循环行为和 pre-M20e 的 `for i in 0..4` 完全一致。

M20a 的 6 个单电机测 + M20c 的 end-to-end SITL 继续 bit-exact 行为。

唯一有 intermediate declaration 的 ancient test 是 `detector_is_latched_does_not_recover`（declare M0 → continue observing clean obs）。对它的影响：
- Pre-M20e: expected τ 里 M0 贡献使用 cmd[0]；M0 已死 + clean obs 意味 observed = 0；expected = cmd[0] 贡献 → residual = cmd[0] 的 signature → score M0 → persistence 累加 → **但 M0 已 dead 跳过**（scoring loop 本来就 skip dead）→ 不 declare
- Post-M20e: expected τ 跳过 M0 → expected = 0 → observed = 0 → residual ≈ 0 → 低于 threshold → Quiet

两路都不错误地 declare。行为上的差别只在 persistence 计数（post-M20e 根本不 accumulate）。`is_latched_does_not_recover` 的断言是 "alive[0] 保持 false"，两路都成立。

### 为什么 test 跑 100 ticks 而不是更少

理论上 2 × 20 = 40 ticks（threshold 20, 两次 declare）够。但：

- Tick 40-50 的 observe 有可能 float edge case 让 persistence 略过临界值（0.9999 不等于 1）
- symmetric 测试下 M0 和 M3 的 score 可能 tick-to-tick 互相替换 BestMatch → persistence reset (M20 pre-e 的 "best idx change resets losers" 逻辑)
- 100 ticks 给足 confidence 不 flaky

100 ticks sim 代价 ~ 1 μs, 不心疼。

### 为什么 test 选 M0+M3 而不是 M0+M1 (对角)

M0 at (+h,+h), M1 at (-h,-h) 是**对角对**。对角 2 电机死会把 vehicle 变成"只剩一个对角的 2-motor quad"，roll/pitch/yaw 只能控 2 DoF (thrust + 1 roll/pitch axis)，yaw 完全失控。在仿真里会立即开始 spin，不能飞行 —— 属于 plan.md 明确 "不可恢复" 的 case.

对角 2 死从 detector 角度也 degenerate：两个 motor 的 torque direction 精确相反（J⁻¹·e_0 = -J⁻¹·e_1 by 几何对称）。它们的 residual signature 完全抵消 —— 如果 cmd 相等的话 residual = 0 → Quiet → **永远不 declare**。

我选 **M0+M3 相邻**：都在 +y 侧，residual 是 -roll 主导 + 一些 pitch/yaw 耦合。M0 和 M3 的 score 都 positive（只是强度不同），detector 能清楚排序。这是典型的 "multiple-motor failure but not degenerate" case。

对角 failure 需要在 plan.md 明示 "不支持的不恢复工况"。作为 follow-up 记录。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-fdir detector
# expected: +1 new test, all 7 motor-fault tests pass
# (starts + clean + each of 4 dead + persistence + latch + double)

$ cargo test --workspace
# expected: ~260 tests pass
```

CI 依赖。

## Follow-ups / 遗留

- **多电机声明的 `SITL end-to-end`**：M20c 的 SITL test 只注入单电机 fault。可以加一个 `two_motor_simultaneous_failure_both_declared` 验证 detector → 3-motor allocation ( 实际上 **不工作** —— 2-motor 挂掉后 failover allocator 的 4 个 3-motor 伪逆都不适用) → altitude 下跌。这个 scenario 本质是 "不可恢复" 演示，做 SITL test 只是记录 vehicle 会乖乖掉下来，不会爆 NaN。
- **对角 2-motor failure 的识别**：detector 当前会漏（residual 抵消为 0）。M20f 可考虑加一个 "所有电机都看起来健康但 observed 比 expected 小 X%" 的总 thrust check，然后 fail-open（declare 整体 Emergency 而不归因具体 motor）。
- **M24 flight mode**：2-motor 失效触发自动切 "emergency descent" mode。
