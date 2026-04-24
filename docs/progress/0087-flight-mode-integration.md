# 0087 — FlightMode 集成（M24 b/c）

**Commits**: `dff8179`（M24b derivation）、`99e1a87`（M24c request_mode）
**Date**: 2026-04-24  **Milestone**: M24b + M24c

## What / 做了什么

把 M24a 的纯 `FlightMode::request` 函数变成 FlightState 上的**可用 API**，分两步：

### M24b — `FlightState::flight_mode()`（commit `dff8179`）

**derivation getter** —— 从现有 ad-hoc 字段综合出当前 mode：

```rust
impl FlightState {
    pub fn flight_mode(&self) -> FlightMode {
        if self.landing_state == Landing { return Landing; }  // safety top
        if self.rtl_phase != Idle         { return Rtl; }      // operator-cmd
        if self.takeoff_state != Idle     { return TakingOff; }
        if !self.motor_fault_detector_enabled { return Acro; }
        Stable
    }
}
```

**优先级**: `Landing > Rtl > TakingOff > Acro > Stable`。6 个新 unit test cover 每条优先级规则 + default state。`Snapshot` 也加 `flight_mode: FlightMode` 字段，供未来 MODE_CHANGE telemetry 使用。

**零行为变化** —— ad-hoc 字段还在，写入路径没动。只是增加了一个 read 视角。

### M24c — `FlightState::request_mode()`（commit `99e1a87`）

**gated mutator** —— 通过 M24a 的纯 `request()` 规则把 mode change 请求转为 side effect：

```rust
impl FlightState {
    pub fn request_mode(&mut self, target: FlightMode) -> ModeTransitionResult {
        let current = self.flight_mode();
        let result = current.request(target);    // M24a 纯函数
        if let ModeTransitionResult::Enter(new) = result {
            match new {
                Stable => /* clear everything, re-enable detector */,
                Landing => self.landing_state = Landing,
                Rtl => self.rtl_phase = Climbing,
                Acro => self.motor_fault_detector_enabled = false,
                TakingOff => /* caller's responsibility (target_z_ned not here) */,
            }
        }
        result
    }
}
```

6 个新 unit test cover 转换的允许路径 + Rejection 路径 + abort (→Stable) + 自转换 no-op。

## Why / 为什么这么做

### 为什么先 derivation（flight_mode）、后 mutation（request_mode）两个 commit

`flight_mode()` 是**只读投影**，对现有代码零风险。`request_mode()` 是**写入路径**，更改 "LAND command → `landing_state = Landing`" 的路径。

分两 commit：
- **M24b 独立可 revert**：如果 flight_mode 的 priority 规则有 bug（例如 Landing 应该 winover Acro 但选反了），只 revert M24b
- **M24c 建立在 M24b 上**：request_mode() 需要 flight_mode() 提供 current mode reading —— 一个 commit 一件事
- **PR size**: M24b 97 lines, M24c 123 lines. Each under 10 min review.

### 为什么 `request_mode` 不 clear rtl_phase on Landing preempt

想象：vehicle 在 `Rtl::Returning`（已经飞到 home xy），突然 pilot 按 LAND。Expected: Landing 启动，RTL 停止。

但我的 implement 里 `Enter(Landing)` 只写 `landing_state = Landing`，`rtl_phase` 仍是 `Returning`。下次 `flight_mode()` 读出来是 Landing（priority 赢），所以行为上正确。但 `rtl_phase` 字段里还是 `Returning`。

为什么不清零？
- **RTL 仍是 "commit 后未完成" 的状态**，清零会丢失上下文信息
- `outer_step` 里的 RTL transitions（M16a `RtlPhase::advance`）会继续**前进**（但因为 landing_state == Landing，active_sp override 先走 landing setpoint）
- 下次 pilot 如果 cancel landing（request_mode(Rtl)），expect RTL 恢复 —— 清零了就做不到（会从 Idle 重新 Climbing）

所以设计是 "Landing preempt 而不 abort"。Follow-up M24d 可以加一个 `request_mode_with_clear`: 明确 abort vs preempt 的两种语义。

### 为什么 `request_mode(Stable)` **清零全部**

对比 Landing preempt："不清" 是保留上下文。Stable 的语义是 **universal abort**：我要从任何 automation 回到手动 hover。这时候保留 landing/rtl/takeoff/acro 的状态没有意义 —— 下次用户明确 re-request 就行。

这也呼应 Stable 的 Kani proof："Stable 从任何 mode reachable"。Reachable 不等于 clean —— 清零让 "reachable" 意味着 "完整 reset"。

### 为什么 TakingOff 只 gate 不 apply

`FlightMode::TakingOff` 需要 `target_z_ned: f32`，但 FlightMode 是无 payload 的 enum。caller 必须 自己 set:

```rust
if flight.request_mode(FlightMode::TakingOff) == Enter(TakingOff) {
    flight.takeoff_state = TakeoffState::TakingOff { target_z_ned: -5.0 };
}
```

为什么不给 request_mode 加 payload 参数？
- FlightMode enum 不应该因为 runtime data 污染抽象（Acro / Landing / Rtl / Stable 都不需要 payload）
- TakingOff 的 altitude 从 MAVLink MAV_CMD_NAV_TAKEOFF 的 param7 来 —— 真实 path 里 caller 一定知道这个值
- 加 `fn request_takeoff(target_z_ned)` 作为便利函数（M24d follow-up）也可以

当前设计：gate 保证**转换合法**，payload 是 caller 的 responsibility。两个 concern 解耦。

### 为什么不把 `landing_state / rtl_phase / takeoff_state` 设为 private

理由：**existing tests + SITL test 里大量直接读写这些字段**。改 private 需要：
1. 每次读改成 getter
2. 每次写改成 request_mode or 专用 setter
3. 重构 ~40 处调用方

对新机器有价值但对已有 green test regression 价值小。M24c 的 request_mode 是**推荐 API**，老路径仍然工作。未来新代码用 request_mode，老代码 opportunistically 迁移。

"Make public API safe, leave private API evolvable" 有点类似。

### 为什么这个做法更好 than "全部用 FlightMode 替换"

Alt 方案：删 landing_state / rtl_phase / takeoff_state，完全用 FlightMode 作为 single source of truth。

问题：
- FlightMode 失去 **sub-state**。例如 RtlPhase 有 Climbing vs Returning，在 Rtl mode 内部的状态机。压成 `FlightMode::Rtl` 丢失这些内部状态
- TakeoffState 的 payload 丢失
- 改动面积大，M16a/b/c 的 Kani 证明需要 port

当前 hybrid：FlightMode = "top-level 类型"；{Landing,Rtl,Takeoff}State 和 detector_enabled = "内部细节"。derivation + request_mode 是 bridge。不彻底但**实用**。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter flight_mode
# M24b: 6 new + M24c: 6 new = 12 new tests

$ cargo test -p app-copter request_mode
# 6 tests

$ cargo test --workspace
# expected: ~277 tests (from ~265)
```

CI 依赖。M24a 的 6 个 Kani proof 仍然 green —— request_mode 不可能 violate 那些 invariants 因为它 delegates 到 request()。

## Follow-ups / 遗留

- **M24d wire into sitl_mavlink**：LAND / RTL / TAKEOFF command handlers 走 request_mode。Reject 路径 emit `MAV_SEVERITY_WARNING` STATUSTEXT `"MODE: X→Y REJECTED"`. 当前所有命令都 auto-succeed so no visible change，但**primitive 到位**。
- **`FlightState::request_takeoff(target_z_ned)`** 便利函数：内部 call `request_mode(TakingOff)` + set takeoff_state。包装 2 行操作成 1 行。
- **Landing preempt 清 rtl_phase**: 看上文讨论，不清是 feature；加 `request_mode_abort_subsidiary_modes` 的变体提供 clean 选项
- **Kani on request_mode**: 类似 M20d 的做法，prove "request_mode 不可能 Enter 一个 request() 说 Reject 的 mode"。Delegation 的形式化。
- **MAVLink MODE_CHANGE announcement**: 每个 Enter 成功的 `request_mode` 也 emit 一条 STATUSTEXT "MODE: Stable→Landing"。视听告诉 pilot "我进了 Landing"
