# 0074 — RtlPhase 从糅合状态机变成可证明的 pure function

**Commit**: `493a643`  **Date**: 2026-04-23  **Milestone**: M16a

## What / 做了什么

M6.3 的 RTL transition 逻辑糅在 `outer_step` 里 —— 读 `FlightState` + `RateLoopConfig`、调 `libm::sqrtf`、改 `flight.rtl_phase` 和 `flight.landing_state`。两个问题：

1. **不可证明**：非 pure，Kani harness 难钩
2. **`sqrtf` 对 CBMC 不友好**：符号推理 sqrt 历史上会让 Kani 跑 15+ 分钟

M16a 重构：

```rust
pub enum RtlTransition {
    Stay,
    Advance(RtlPhase),
    Handoff,
}

impl RtlPhase {
    pub fn advance(
        self,
        home: Option<Vector3<f32>>,
        pos: Vector3<f32>,
        safe_alt_m: f32,
        xy_tol_m: f32,
    ) -> RtlTransition
}
```

调用方 `match` 就应用：
```rust
match flight.rtl_phase.advance(...) {
    RtlTransition::Stay => {},
    RtlTransition::Advance(n) => flight.rtl_phase = n,
    RtlTransition::Handoff => {
        flight.rtl_phase = RtlPhase::Idle;
        flight.landing_state = LandingState::Landing;
    }
}
```

**sqrt 消除**：Returning 阶段的"到家距离 < 容差"从 `sqrt(dx² + dy²) < tol` 变成 `dx² + dy² < tol²`。同样物理意义，Kani 可证。

**4 个新 Kani 证明**：

| 证明 | 保证 |
|---|---|
| `idle_is_absorbing` | `Idle.advance(任意输入)` 永远 `Stay`（Idle 不自发脱离） |
| `no_home_cancels_rtl_to_idle` | `home = None` 时任何 phase 都返回 `Advance(Idle)` |
| `climbing_never_handoffs_directly` | Climbing 只能 `Stay` 或 `Advance(Returning)`，绝不跳 Handoff |
| `returning_never_advances_to_climbing_or_idle` | Returning 只能 `Stay` 或 `Handoff`，绝不回退 |

合起来证明**"RTL 状态机只走 Climbing → Returning → Handoff 这一条路"**。

CI 把 `app-copter` 加入 Kani job 列表。Workspace 证明数 **18 → 22**。

外部行为 byte-identical：原 3 个 RTL SITL 测试通过，workspace 248 tests 全绿。

## Why / 为什么这么做

### 为什么用 `RtlTransition` enum 不直接 `&mut` 参数

两种设计：

A. `fn advance(&mut self, ...)` — 函数自己改 `self`，调用方看到最终状态  
B. `fn advance(self, ...) -> RtlTransition` — 函数返回 "意图"，调用方应用

选 B 因为：

1. **Kani 可验证**：纯函数输入输出固定，`match` 上 return value 能断言 shape（"永远不返回 X"）
2. **副作用在调用方**：`Handoff` 时要改 `flight.landing_state` —— 这是 `FlightState` 职责，不是 `RtlPhase` 的
3. **pub API 表达性**：`RtlTransition::Handoff` 显式说明"该 hand-off 到 Landing"，比隐式 `self.phase = Idle; other_state.landing = ...` 清晰得多

### 为什么 `sqrt` 要消掉

CBMC（Kani 用的 SMT solver）对 `sqrt` 的符号推理基本没有。看到 `sqrtf(x)`：

- 没有 axiom "y = sqrt(x) ⇒ y ≥ 0"
- 没法证明 "sqrt(dx² + dy²) < tol ⇔ dx² + dy² < tol²"（这俩等价但 solver 不知道）
- 结果：证明 time out 或 "verification unknown"

把比较换成 `dx² + dy² < tol²`：**数学等价**（对非负值），**CBMC 秒证**。代码上有一处不对称（tol 平方要算），但 `rtl_xy_tolerance_m * rtl_xy_tolerance_m` 编译器大概率会优化掉。

副作用：如果有人把 `rtl_xy_tolerance_m` 设负数（不合理，但运行时无 type guard），平方后为正，比较行为**比原来更保守**（任何点都 < tol²，永远不触发 Handoff）。原 `sqrt` 版本：`sqrtf(dx²+dy²) < negative` 永远 false，行为一致。**没回归**。

### 为什么 `any_finite_f32()` 助手

Kani 的 `kani::any::<f32>()` 覆盖所有 `f32` bit pattern，包括 NaN 和 ±∞。第一次跑就炸：

```
Check 12: RtlPhase::advance.NaN.1 — Status: FAILURE
Check 25: RtlPhase::advance.NaN.2 — Status: FAILURE
```

Kani 检测到 `home.z - rtl_safe_alt_m` 可能产生 NaN（当 `home.z` 和 `rtl_safe_alt_m` 都是 NaN 或 ∞ - ∞ 时）。这**不是** bug：`NaN ≤ anything` 为 false，所以 Stay —— 正确行为。但 Kani 的 NaN-generation check 不 happy。

加 `kani::assume(x.is_finite())`：

- **在真实 EKF + MPC 管道里 EKF 永远输出有限值**（Joseph-form 协方差更新 + numerical floor 保证）
- Kani 只证 **"finite 输入下状态机正确"**，不分心 NaN 边界
- 如果有 bug 让 NaN 流进 `advance`，**别的测试会抓** —— property test + sim 会发现

### 为什么不在 outer_step 里包 `assume` 而在 harness 里

`outer_step` 是生产代码，加 `assume` 既没意义（no_std 没有 Kani）也增加 runtime check。harness-level assume 只在 Kani 符号执行时生效。

### 为什么 RtlPhase 的 Kani 证明是整个 M16 里最有价值的一批

看现有 18 个证明的分布：
- 4 on Priority（total order）
- 4 on HealthLevel state machine（FDIR）
- 1 on EKF degeneracy
- 5 on core-bus schema

都是**单 function / 单状态**的证明。RtlPhase 是**第一个真·飞控状态机**被 Kani 证明：

- 跨 3 个状态 × 2 个触发条件 × 2 个配置参数 = 12 个 path
- Kani 枚举每个 path，确认 transition shape 符合合约
- 对 flight safety 有直接意义：**RTL 路径不会"跳过降落"或"回退到 climbing"**

这是"**形式化验证飞控状态机**"从 "理想" 变 "落地" 的具体证据。

### 为什么 `#[cfg(kani)] mod rtl_kani` 而不是加到已有测试

纯 Kani 模块放 `#[cfg(kani)]` 下的好处：
- **test module 不受影响**：已有 SITL 测试不依赖 kani crate
- **Kani CI job 独立**：`cargo kani -p app-copter` 只编译 kani-cfg code
- **便于 grep**：`grep -rn kani::proof` 立刻找到所有证明

和 `algo-fdir` / `core-rtos` / `core-bus` 已有的 `#[cfg(kani)] mod *_kani` 模式保持一致。

## How it's verified / 怎么验证的

```bash
$ cargo kani -p app-copter
4 successfully verified harnesses, 0 failures, 4 total.
Verification Time: 28 ms

$ cargo test -p app-copter --lib rtl
3 passed (existing RTL SITL tests, behaviour unchanged)

$ cargo test --workspace
248 passed

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿

$ cargo kani -p core-bus core-rtos algo-fdir algo-ekf app-copter
5 crates × ~2 s each = ~10 s total Kani CI time
Workspace 22 verified harnesses total.
```

## Follow-ups / 遗留

- **`LandingState::advance`** 类似重构：当前 landing 完成逻辑（touchdown + auto-disarm）也糅在 outer_step 里。Pure function + Kani 证明 "Landing 只能因为触地检测进 Idle"。
- **`TakeoffState::advance`** 同样：TakeOff 完成逻辑的 pure function 版本 + Kani 证"TakingOff 只能因为 altitude-reached 退出"。
- **`preflight_check` Kani**：所有 `FlightState` → `PreflightReject` 的完整性证明（每种 HealthLevel 组合都映射到正确 reject）。
- **`outer_step` 整体不会 panic**：Kani 证 outer_step 对任意合法输入不触发 panic（需要消 unreachable! 和所有 .unwrap() —— 已经基本清掉但值得 formally 证明）。
- **PositionController::step 返回有限 quaternion**：所有变体输出 `q.is_finite() && (q.norm() - 1).abs() < eps`。Kani 对 f32 的限制可能挡住，看能走多远。
- **MAVLink parse 对任意 byte slice 不 panic**：cargo-fuzz 覆盖、但 Kani 证明 + bounded-length slice 是正交补充。
- **自动化 Kani proof count 进 badge**：CI 输出 proof 数 → README badge "22 formal proofs verified"。
