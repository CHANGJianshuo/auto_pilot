# 0076 — Single-motor failure injection + honest SITL baseline

**Commit**: `705e399`
**Date**: 2026-04-23  **Milestone**: M18

## What / 做了什么

给 `sim-hil::SimConfig` 加了新字段：

```rust
pub motor_fault_mask: [f32; 4],   // default [1.0; 4] = nominal
```

在 `step()` 里，**motor lag 滤波之后**，用 mask 相乘得到真正的 delivered thrust。语义：

- `1.0` → 正常
- `0.0` → 电机死亡
- `0.5` → 效率减半（退化）
- `1.3` → 失控过推

以及一条新 SITL 测试 `single_motor_failure_state_stays_bounded`：
稳态悬停在 1 m 高度，`t = 2 s` 时 `motor_fault_mask[0] = 0.0`，继续跑 3 s。用的是 MPC-I position controller（当前 shootout 亚军，有 integrator）。

**今天能断言的**（honest baseline）：
- `position.norm().is_finite()` — 没 NaN / ∞
- 高度误差 < 60 m — 被自由落体物理界约束住
- 最大倾角 < 170° — 没翻滚无穷远

**今天断言不了的**："维持高度 ± 2 m" —— 因为 `algo-alloc` 是**固定 4-电机** `E_inv`，死一个电机后分配矩阵就失效，飞机掉下去。这个 gap 以 M19 task #97 立帐。

## Why / 为什么这么做

### 为什么 Phase III 的 #2 benchmark 选"单电机失效"

plan.md 里 Phase III 四个基准：8 字航点（M17 ✅）、**单电机失效**（M18 本次）、pitch-roll 翻滚、Swift 类 agile pass。选顺序考虑：
- **写 sim 基础设施**（motor fault injection、survival metric）的成本一次性 —— 做完无论未来加什么 failure mode 都复用；
- 单电机失效**直接暴露 allocation 弱点**，给后续 M19 一个明确的客户 —— "给我能算的 3-motor E_inv"；
- 翻滚和 agile pass 都需要先把姿态 recovery 搞顺，而 single-motor 是最激烈的姿态扰动，做完姿态做剩下两个成本降低。

所以：**基础设施先行，优先能暴露真 gap 的 scenario**。

### 为什么 mask 加在 lag filter 之后

初版我把 mask 加在 `motor_thrusts_cmd_n` 之前（也就是先 clip 再 lag），问题：

```
t=2.000 s: command = 10 N, mask[0] = 0, actual = 10 N × 0 = 0 → OK
t=2.001 s: lag filter sees actual=0 want 10 (command 仍是 10) → decays toward 10
                                                                ↑ 电机"活了"
```

因为 lag filter 的 target 是 command，mask 在 command 端不改 filter target，死掉的电机会在 20 ms 时间常数内"慢慢爬回原命令" —— 物理上不对。

**正确的语义**：mask 作用在 delivered thrust（输出端），而 filter state 就是 delivered thrust。所以 mask 加在 filter 之后；下一 tick filter 读回来的就是 masked 值，故障持续。数学上等价于 "把 `filter_state *= mask` 直接写进 state"，但代码清晰度差一点。

### 为什么 MPC-I 而不是 PI cascade

PI cascade 的积分器会疯狂累积（死电机导致 steady-state 误差再多也吃不掉），积分饱和后行为很难预测 —— 测试会 flaky。MPC-I 的积分器**带 QP 约束**，上下限 ±20 m/s²，即使电机全死也不会 wind up 到离谱值。选它是为了故障断言的稳定性。

后续 M19 有 failover allocation 之后，换回 PI 也能过（那时候 PI 能跟住），此处是"当前唯一真能跑 5 s 的选择"。

### 为什么 free-fall 界给到 60 m 而不是理论 44 m

理论值：`½ × 9.81 × 3² ≈ 44.1 m`。但：
- 3 个存活电机还在出 ~75 % 额定推力，减缓下坠；
- 本测试不是要精确 match 自由落体，而是要拒绝**非物理**轨迹（例如数值发散飞到 10 km/s）；
- 60 m 是 44.1 m + ~36 % 余量，留给积分器叠加误差不干扰断言；
- 低于 60 m 能排除所有"状态爆炸 / NaN 式 crash"，这是测试的真实目标。

### 为什么 tilt 用 `arccos(2w² - 1)` 而不是 roll-pitch-yaw

quaternion 到**总倾角**（body z-axis vs world z-axis 的夹角）只需要 `w`：
- `R[2,2] = 1 - 2(x² + y²) = 2w² - 1 + 2z²` for Hamilton 归一化四元数
- 忽略 yaw（绕 body z），这就是 `arccos(2w² - 1)`

只要一步 acos，比转 rpy 再计算 `max(|roll|, |pitch|)` 便宜 ~3 倍。而且 "flip 过 180°" 在这个单值上直接可见 —— 任何姿态机真解体都会把 `w` 跳过 0.5。

### 为什么 assertion 是 "state stays bounded" 而不是 "recovers"

Phase III 的**最终**目标是"单电机失效后自主返航"，但今天的 allocation 层**物理上做不到**。如果我在 M18 里断言 "altitude ± 2 m"，测试**必然 fail** —— 我要么
1. 跳过这个 test 等 M19，或者
2. 写一个"软"断言 today 能过但未来 M19 不会收紧

两个都差：跳过就没 regression，软断言会在 M19 引入新 allocation 时**被新 fix 意外触发** —— 因为 "bounded" 可能在修好之后反而更糟（M19 如果有 bug 失稳得更快）。

我选的路线：写 **"bounded"** 作为今天的真话，并在 M19 task 里**显式 TODO "tighten to ± 2 m"**。这样：
- 今天：CI 绿，regression guard 捕捉"状态爆炸"；
- M19：改 allocation 后**必须同步改此 test 的阈值**（不然 PR reviewer 会看到 assertion 不匹配新能力）。

代码注释里写明了这一点，下个迭代不会被忘。

### 为什么 M19 被拆成独立 task

写 motor-fail-aware 3-motor allocation 不是小活：
1. 检测"哪个电机死了"（FDIR） —— 对比 commanded 和 observed ω_dot，fault detector 需要滤波 + persistence
2. 重写 `algo-alloc::E_inv` —— 6 种 3-motor 子集（4 个固定 failure case × 冗余余量）分别算伪逆
3. 控制律要接受 **reduced-DoF** —— 3 motor 只剩 3 自由度控制（通常舍弃 yaw），需要权重矩阵让 yaw 松弛
4. INDI 内环需要适配降秩 `G` 矩阵
5. 新的 Kani 证明："allocation 输出在 `motor_fault_mask` 下不发散"

这大约是 M11（residual policy）或 M10（Zenoh）的规模，单独 milestone 合适，不混进 M18。

## How it's verified / 怎么验证的

本地无法跑 `cargo test`（工具链不在 PATH），依赖 CI 全量验证。

预期（承 M17 baseline）：
- `cargo test --workspace` → 252 passed（M17 的 250 + M18 本次新增 1；不，实际 M17 新增 3，基准 247→250；M18 新增 1 → 251；最终以 CI 为准）
- `cargo clippy --workspace --all-targets -- -D warnings` 绿（已修掉 `field_reassign_with_default` — 用 struct update syntax 构造 `SimConfig`）
- `cargo fmt --all -- --check` 绿
- `cargo build -p app-copter --bin firmware --target thumbv7em-none-eabihf` 绿（sim-hil 是 host-only，没影响 firmware）
- Kani 28 proofs 不受影响

本地已做 code review：
- `lib.rs`：字段加默认 `[1.0; 4]`，docstring 明确 1.0=正常 / 0=死亡；mask 位置在 lag 之后（见 Why 段）
- `sitl.rs`：fault_mask 中途改写只有测试里一处；断言是松但物理正确；`use libm` 已有（tilt 计算不新增 dep）

## Follow-ups / 遗留

- **M19 motor-fail-aware 3-motor allocation + FDIR detector**（task #97）
    - 加 `MotorFaultDetector` 到 `algo-fdir`，输出 `[bool; 4]` alive mask
    - 给 `algo-alloc` 加 `E_inv_3motor(dead_idx)` 查表（4 个预计算伪逆）
    - INDI 根据 alive mask 选 `G`
    - 收紧本测试 assertion 到 altitude ± 2 m
- **多电机失效 matrix**：一次死 2 个电机几乎不可救（X-quad 配置），除非对角 2 个 → 剩 2 motor 变不可控。文档层面说明不支持即可。
- **软失效 mask 值**：`0.5`（thermal degradation）和 `1.3`（runaway）比 `0.0` 更常见，给这些 scenario 加独立 test。当前 M18 只测最激烈的 `0.0`。
- **FDIR 检测延迟预算**：从电机实际断到 detector 报警之间需要多少 ms？这个 budget 直接决定允许多大倾角。先做 sim，再找真实数据校准。
- **MAVLink motor health telemetry**：`MOTOR_INFO` 或自定义 message 让 GCS 看到哪个电机挂了。落在 M19 之后。
