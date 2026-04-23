# 0086 — FlightMode enum skeleton + Kani（M24a）

**Commit**: `6a0aa30`
**Date**: 2026-04-24  **Milestone**: M24a

## What / 做了什么

新 public API 在 `app-copter::lib.rs`：

```rust
pub enum FlightMode { Stable, Acro, TakingOff, Landing, Rtl }
pub enum ModeTransitionResult { Stay, Enter(FlightMode), Reject }

impl FlightMode {
    pub fn request(self, target: FlightMode) -> ModeTransitionResult;
}
```

5 个 mode + 3 个 transition result。规则：
- **Self-transition → Stay**（无事发生）
- **→ Landing 总是允许**（safety preempt）
- **→ Stable 总是允许**（从任何非-Stable，operator abort）
- **→ Acro / TakingOff / Rtl 只能从 Stable 进**（互斥 automation）
- **其他 → Reject**

6 新 Kani proof（workspace 34 → 40）+ 6 新 unit test。

**这个 commit 不改变任何运行时行为** —— 它只引入类型 + transition 函数 + 证明。现有 `landing_state / takeoff_state / rtl_phase / motor_fault_detector_enabled` 字段没动。M24b 将迁移 FlightState 用 FlightMode 替换这些 ad-hoc 字段。

## Why / 为什么这么做

### 为什么引入 FlightMode 而不是直接用现有字段

现状：FlightState 里有 4 个彼此半独立的 mode 字段：
- `landing_state: LandingState { Idle, Landing }`
- `takeoff_state: TakeoffState { Idle, TakingOff { target_z_ned } }`
- `rtl_phase: RtlPhase { Idle, Climbing, Returning }`
- `motor_fault_detector_enabled: bool`（M22 引入）

问题：
- **无法强制互斥**。类型上可以同时 `landing_state == Landing` 且 `rtl_phase == Climbing`。`outer_step` 里用 if-else 链保证行为一致，但**类型系统不帮忙**
- **无法统一 mode-entry callback**。Acro mode 进入应该 `motor_fault_detector_enabled = false`；没有 mode-entry 概念，只能每次 handler 手动改
- **新 mode 增加 friction**。加 Offboard / GPSFailsafe / Auto-landing 之类需要再加字段 + 每个字段交互

`FlightMode` 是 **invariant-first** 设计：只有**一个** enum 说现在在什么 mode，transition 是纯函数可证，进入某个 mode 是单一入口 → callback 可集中定义。

### 为什么 skeleton 不是直接 migration

按我在 auto_pilot 的 commit 习惯（见 `docs/feedback_progress_docs.md` + M16a 先 pure-fn 后 integration 的 pattern），**先落合约**、**后迁移**。

对比 pure-fn 先行 vs 一票到底：

**一票到底**（enum + refactor 所有 state fields + 迁移 outer_step + 迁移所有 test）：
- PR diff ~1500 行
- CI 一绿一红很难诊断（test regression 来自 enum 定义 bug 还是 migration bug？）
- 无法独立 revert （enum 和 migration 耦合）
- Kani 和 migration 证据混在一起

**pure-fn 先行**（enum + pure fn + tests + Kani；0 改 runtime）：
- PR diff ~220 行，评审 10 分钟
- 现有 test 零 regression（保证无行为变化）
- Kani 验证 invariant 在 migration 之前**已证明**
- M24b 的 migration 可以**对比**这个已 Kani-verified 的 spec 作为 ground truth

这套"decision pure-fn → apply fn → integration"的三分法已经成功用在 M1.14 / M16a/b/c / M20d。M24 按同样 pattern。

### 为什么规则是这五条而不是更复杂的

替代方案考虑：
1. **允许 Landing → Rtl**：不，在地面触地后只能 disarm + ground reset，RTL 毫无意义
2. **允许 Rtl → Landing 自动 handoff**（M16a 已有）：对，但这是 **RtlPhase 内部 transition**（Returning → Handoff），不经过 FlightMode::request —— 由 outer_step 直接调用 `rtl_phase.advance()`
3. **允许 TakingOff → Rtl 衔接**：不，takeoff 完成后 mode 回到 Stable，然后由外部命令（MAVLink 或 mission）设置 Rtl
4. **Acro → Landing 应该允许吗**：是，Landing 是 universal preempt。即使 pilot 在做 flip，触发 Landing（比如 battery critical）应该立即生效

第 5 条**兜底 Reject** 确保没有隐式 transition 漏网。任何未显式允许的转换都失败 —— "安全默认"。

### 为什么 transitions 是"Stay / Enter / Reject"三态而不是 `Option<FlightMode>`

`Option<FlightMode>` 含义：`None` = 保持不变，`Some(new)` = 切换。但这把 **"保持不变"** 和 **"明确拒绝"** 混在一起。

业务语义上它们不同：
- **Stay**: `request(Landing, Landing)` — 用户请求已激活的 mode，啥事没有，无需 log
- **Reject**: `request(Acro, Rtl)` — 用户请求被拒绝，**应该 STATUSTEXT 通知**"Can't switch from Acro to Rtl"

三态让 caller 明确处理。Rust style 上，把策略决定（"无事发生" vs "明确拒绝"）暴露给 caller 是 idiomatic。

### 为什么 Acro 不能 preempt 掉 Rtl

Rtl 是安全关键的 return-home 流程。如果允许 `Rtl → Acro`，pilot 可以在 RTL 中途改 Acro 做 flip，然后手忙脚乱无法回家。

规则"Acro 只能从 Stable 进"强制 pilot 先 Rtl → Stable（显式 abort）再 → Acro。两步，每步都 intentional。

### 为什么 `request` 是 `pub fn` 不是 `const fn`

想 const：编译期求值可能在 matches! 中优化掉分支。但：
- 现在 const-eval 对 `PartialEq::eq` derived impls 不 stable
- 我自己手工写一个 const-friendly equality 开销大（5 个 enum variant 交叉）
- Runtime 开销本来就 5 个分支的 if-else，几 ns，没优化价值

仅保留 `#[must_use]` 强制 caller 处理结果。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib flight_mode
# 6 new unit tests

$ cargo kani -p app-copter
# 16 proofs (10 RTL/Landing/Takeoff + 6 FlightMode)

$ cargo test --workspace
# expected: 259 → ~265 tests
```

CI 依赖。

## Follow-ups / 遗留

- **M24b — migration**：
    - 加 `FlightState.flight_mode: FlightMode` (默认 Stable)
    - `outer_step` 的 if-else 链（landing_state == Landing、takeoff_state != Idle、rtl_phase != Idle）改成 `match flight.flight_mode`
    - MAVLink command handler 用 `flight_mode.request(target)` 而不是直接写 landing_state / takeoff_state
    - Mode-entry callbacks：
        - `Enter(Acro)` → `motor_fault_detector_enabled = false`
        - `Leave(Acro)` → `motor_fault_detector_enabled = true`
        - `Enter(Landing)` → `landing_state = Landing`
        - etc.
    - Compat layer：保留 `landing_state / takeoff_state / rtl_phase` 作为 **derived** 字段（由 flight_mode 派生），防止外部 API 破坏
- **M24c — TakingOff payload**：现在 FlightMode::TakingOff 没带 target altitude。实际使用时需要。要么 variant with payload (`TakingOff { target_z_ned: f32 }`)，要么把 target 放 FlightState 另一个字段
- **Kani proof ideas for M24b**：
    - "entering Acro flips detector_enabled to false"
    - "leaving Acro flips detector_enabled back to true"
    - "any flight_mode produces deterministic (landing_state, takeoff_state, rtl_phase)" - 派生一致性
- **STATUSTEXT on Reject**：`MAV_SEVERITY_WARNING` + "MODE: Acro→Rtl rejected" 让 pilot 知道为什么切换没生效
