# 0008 — HealthLevel 状态机 + Kani 证明

**Commit**: `94f1c24`  **Date**: 2026-04-20  **Milestone**: M1.4

## What / 做了什么

给 `algo-fdir::HealthLevel` 搭起一个小而硬的状态机：

- 4 个 level：`Healthy(0) → Degraded(1) → Emergency(2) → Failed(3)`
- `severity() -> u8` 显式 match（避开 `as` cast）
- `transition_in_flight(proposed) -> HealthLevel` —— 接受或拒绝新 level，**只允许恶化不允许恢复**
- `reset_on_ground() -> HealthLevel` —— 单独的函数做 Healthy reset，让 in-flight 路径保持"单向"
- `requires_abort() -> bool` —— Emergency 及以上返回 true
- `ALL` 常量、`#[derive(PartialOrd, Ord)]` + `#[repr(u8)]`
- **6 个单元测试**（严格递增、恶化接受、恢复被拒、idempotent、reset、abort threshold）
- **5 个 Kani 证明**：
  - `check_transition_is_monotone` —— 转移后 severity ≥ 原值（核心 FDIR 不变量）
  - `check_transition_honours_deterioration` —— 更坏的 proposed 一定被采纳
  - `check_transition_is_idempotent` —— 转到自己是 no-op
  - `check_reset_yields_healthy` —— reset 永远出 Healthy
  - `check_abort_threshold_matches_severity` —— `requires_abort` 与 severity 门限完全一致

## Why / 为什么这么做

### FDIR 的"单向性"为什么是个硬约束

设想一个场景：IMU 投票器短暂抖动，先报 `Emergency`，100 ms 后"好像又正常了"。如果系统允许 `Emergency → Healthy` 自动回退，控制律会在两种模式间抖动，乘客可能把"已经该降级的系统"当作健康的用。

工业级 FDIR 的标准做法：**一次坏了就坏到底，飞完当前任务或返航**。要恢复，得地面人工介入（root-cause 分析 + 签字）。本 commit 把"人工介入"具象化为 `reset_on_ground()` ——一个不传 `self` 的关联函数，编码在类型层面说"这条路径不通过 in-flight 代码"。

### 为什么用 `#[derive(PartialOrd, Ord)]` + 显式 `severity()`

有了 `#[repr(u8)]` 和变体顺序，`Ord` 派生会按声明顺序。但工业级代码**不想依赖声明顺序**——今天它是对的，明天有人为字母排序重排变体就完蛋。

我给了两条"冗余但互相校验"的真值：
1. `#[derive(Ord)]` 让 `lvl1 < lvl2` 直接可用（调用点友好）
2. `severity() -> u8` 显式 match（任何重排都会立刻在 match 里暴露）

`check_abort_threshold_matches_severity` proof 检验二者不矛盾。

### 为什么把 "reset" 做成关联函数而不是方法

```rust
// 我们选的
HealthLevel::reset_on_ground() -> HealthLevel  // 不接受 self

// 拒绝的
fn reset(self) -> HealthLevel { HealthLevel::Healthy }  // 接受 self
```

第二种写法会让 `lvl.reset()` 在任何代码里能调用——包括 in-flight 路径。Kani 没法证"这个方法不从 in-flight 被调用"，因为那不是语法属性。

第一种写法在类型上就说明：调用方必须**有意识地敲** `HealthLevel::reset_on_ground()`，grep 就能找到所有地面复位点。IDE 用户也无法误触。

### 规模

4 个变体 × 1 个转移函数 = 小得发亮的状态空间。Kani 在 **18.6 ms** 里穷举了所有组合。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-fdir
test result: ok. 6 passed; 0 failed

$ cargo kani -p algo-fdir
SUMMARY:
 ** 0 of 1 failed
VERIFICATION:- SUCCESSFUL
Verification Time: 0.018639045s
Manual Harness Summary:
Complete - 5 successfully verified harnesses, 0 failures, 5 total.

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **FDIR 的下一层**：现在 `HealthLevel` 是"当前状态"容器，还缺"**什么条件触发什么转移**"。比如：
  - `sensor_vote_failed(sensor_id) → Degraded`
  - `actuator_saturation_accumulated(time) → Emergency`
  - `esc_current_exceeded(time) → Emergency`
  - `battery_below_rtl_reserve → Degraded` 继而 → `Emergency`
  
  这些是后续工作。本 step 只做 level 本身的不变量。
- **多个子系统健康合并**：大飞机有多个子系统（IMU、GPS、电池、ESC、RC），顶层健康是 `max(severity)` 的合成。后续加 `CompositeHealth { imu, gps, power, ... }` 结构。
- **恢复历史记录**：真工业机还要记 reset 按钮的时间戳 + 操作员 ID，给日志和飞行数据记录器。
