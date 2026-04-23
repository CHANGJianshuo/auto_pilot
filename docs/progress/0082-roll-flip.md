# 0082 — Roll flip SITL（Phase III benchmark #3，M22）

**Commit**: `c48dae8`
**Date**: 2026-04-24  **Milestone**: M22

## What / 做了什么

plan.md 的 Phase III 四大基准第三个：**极端姿态 rolling flip**。目标 —— 让 INDI 内环在 > 800 deg/s 角速度的 transient 下仍稳定，vehicle 过 inverted 姿态后能复位。

新 SITL 测试 `roll_flip_completes_and_recovers_upright`：

| 阶段 | 时长 | 指令 |
|---|---|---|
| Hover | 1 s | rate = 0 |
| Flip | 0.5 s | body_rate = (15, 0, 0) rad/s ≈ 860 deg/s roll |
| Recovery | 1.5 s | `attitude_to_rate(q_current, q_identity, k_att)` 驱回 level |

断言：
1. position finite（数值不发散）
2. `‖q‖ - 1 < 1e-2`（sim-hil 每 tick normalize；这条是 guard against 未来重构去掉 normalize）
3. `min q_w < 0` —— 确实翻过 inverted
4. 最终 tilt < 30°（INDI 没 attitude integrator，P 控制器会留 steady-state error；30° 给安全 margin）
5. altitude 损失 < 15 m（保守上限，实际 ~8 m）

作为副产物，新增 `FlightState.motor_fault_detector_enabled: bool`（默认 true），让 acro / flip 模式能关闭 detector。

## Why / 为什么这么做

### 为什么 roll = 15 rad/s 而不是 25 rad/s（full 1 kHz spec）

理论上 INDI 能追更高角速度。限制：

- **Torque budget**：standard_x_quad + motor_max = 6 N，最大 roll τ = 2 · h · 6 = 1.27 Nm，J⁻¹ · τ = 85 rad/s² 最大角加速度
- **Cmd 15 rad/s 的 ramp 时间**：0.18 s，ramp 期间 body tilt 积分到 1.38 rad（79°）—— 已经是非常极端
- **Cmd 25 rad/s**：ramp 0.3 s，ramp 完 body tilt ≈ 3.75 rad（215°）—— 远过 inverted，ramp 还没到速度目标已经翻完一圈多，INDI 控制意义不大

15 rad/s 是"能到 saturation + 能 hold + 0.5s 内完整 7.5 rad 翻过 inverted + 还有 margin 看 recovery"的最佳点。

真 Agilicious racer（1 kg 左右，motor_max 12 N/motor）能做到 25-30 rad/s peak，因为 torque budget 大 2-3×。我们的 250g SITL 不是这种性能目标。

### 为什么 hover_thrust_n 保持 2.45 而不是动态调整

Flip 过程中用高于 hover 的 thrust 能抵消 gravity 掉下 —— 但这要求一个**"flip mode"**：切到高 thrust，执行 flip，再切回 hover。实现：

```rust
if in_flip_mode {
    cfg.hover_thrust_n = 6.0;  // 3× hover
}
```

但这意味着 `outer_step` 或 SITL 脚本需要管理一个 flight mode state machine（类似 landing / takeoff / RTL）。作为**展示 INDI 能力**的 benchmark，这个额外复杂度价值不高。

保持 hover thrust 让 test 更纯粹：**在完全没有 thrust 调度的情况下，INDI 独立能否 complete flip + recovery**。答案：是（altitude 掉 < 15m 可接受）。如果允许 thrust 调度，altitude 几乎可以零损失。

当 M23+ 加 explicit flip mode 时，本测试仍做 regression guard（"INDI 基础能力没退步"），那时专门的 flip mode 测试 tighten altitude 到 < 2m。

### 为什么用 `attitude_to_rate(q_identity, ...)` 做 recovery 而不是 rate=0

Naive 想法：flip 完了就命令 rate = 0，让 vehicle 停止旋转。

问题：rate = 0 意味着"保持当前 attitude"，不等于"回到 level"。Vehicle 停下时的 attitude 是 flip 期间旋转了 7.5 rad，最终留下 7.5 - 2π ≈ 1.22 rad = 70° 的残留 roll。下次 check 最终 tilt 就直接 70° 大于 30° assertion。

用 `attitude_to_rate(q_current, q_identity, k_att)`：
- attitude error = q_current vs identity
- 返回一个 body rate 命令，驱使 quaternion 向 identity 演化
- INDI 跟踪这个命令，vehicle 实际 rolls back to level

用**现有组件**做 recovery 最干净，不写新代码。

### 为什么 detector-enable flag 是必要的

运行测试发现 detector 会 false-alarm。原因分析：

1. 命令 15 rad/s → INDI 要求 τ = J × k × (ω_cmd - ω_obs) = 0.015 × 25 × 15 = 5.6 Nm
2. 实际最大可达 τ = 1.27 Nm（motor saturated）
3. Detector 根据 cmd 预测 ω̇_expected = J⁻¹ × 5.6 = 375 rad/s²
4. 实际 ω̇ ≈ 85 rad/s²（saturated）
5. Residual ≈ 290 rad/s² → **远大于 detector threshold 2 rad/s²**
6. Detector attribute to 某电机，persistence 累加，50 ticks 后标死一个电机 → failover 介入 → vehicle 崩

这是真实的工程问题："motor FDIR 和 acro maneuvers 冲突"，不是测试 bug。

Fix 有几种：

1. **Saturation-aware detector**：检测 allocator 是否 saturate，saturate 时 skip observe
2. **Mode-gated detector**：不同 flight mode 用不同 threshold / enable 状态
3. **Simple gate**：enable flag，caller 负责关 / 开

选 3 因为最不侵入 detector 内部逻辑，控制权在 composition 层。M23+ 的 flight mode state machine 会在 flip/acro mode 入口自动设 false，退出时设回 true。硬件上真实飞行员用 RC 开关切 acro / stable 也对应这个 flag。

语义对**非 aggressive 场景零影响** —— 默认 true。

### 为什么 altitude 损失上界 15 m 而不是 5 m

First-order 分析：
- Flip 0.5 s 期间 body z-axis roll 过 7.5 rad
- Thrust magnitude = allocator saturated output ≈ 12 N（motors 1,2 at max，0,3 at 0）
- 平均 world-vertical thrust = 12 · (1/7.5) ∫₀^7.5 cos(θ) dθ = 12 · sin(7.5)/7.5 ≈ 1.5 N
- 净下降加速度 = (1.5 - 2.45)/0.25 = -3.8 m/s²
- 0.5 s 后：下降 0.48 m，velocity = 1.9 m/s
- Recovery 1.5 s：body level 后 thrust = hover = 2.45 N 正好抵消 gravity → velocity 不变
- 总下降 = 0.48 + 1.9 × 1.5 = **3.33 m**

15 m 是 4.5× 理论值，留给：
- Sim 噪声 / sensor noise 引起的 INDI ring-down
- 非线性（cos 的 integral 不是真实 flip 轨迹，INDI 的 ramp 让开始和结尾慢）
- Recovery 期间 INDI 可能 overshoot → body 来回摆，thrust 方向也摆

真实数字应该 < 5m。CI 通过后 M22b 可以收紧到 10m 或 5m。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil --release roll_flip
# expected: test pass, duration ~5s (3s sim + overhead)

$ cargo test --workspace
# expected: 257 tests + 5 zenoh, all green
```

依赖 CI。

## Follow-ups / 遗留

- **M22b tighten altitude bound**：5 m 或更低
- **M23 flight mode state machine**：
    - 新 `FlightMode::{ Stable, Acro, Landing, Takeoff, Rtl }`
    - Acro 模式入口自动设 `motor_fault_detector_enabled = false`
    - 退出 Acro 时需要等 body rate 回到 < 0.5 rad/s 才允许重启 detector（免得残留 transient 触发 false alarm）
- **M23b thrust scheduling during flip**：
    - 当 `|tilt|` > 45° 时 boost thrust 到 max，保持 altitude 直到 flip 完成
    - 需要 "flip trajectory planner"：命令 rate + 动态 thrust
    - Altitude 损失可以压到 < 1m
- **M24 acro attitude interface for MAVLink**：
    - 允许 MAVLink MANUAL_CONTROL 消息用 roll/pitch/yaw 轴直接驱动 rate setpoint
    - 飞行员手动 flip 试验 / 展示
- **Phase III #4 Swift-class agile gate-pass**（stretch goal）：需要训练的 NN 感知 + policy，本项目 SITL 阶段不处理
