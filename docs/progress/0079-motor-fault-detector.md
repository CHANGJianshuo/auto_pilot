# 0079 — 电机故障检测器 end-to-end（M20 a/b/c）

**Commits**: `074b3d5`（M20a algo-fdir）、`1426980`（M20b app-copter wire）、`0ecef90`（M20c SITL end-to-end）
**Date**: 2026-04-23  **Milestone**: M20

## What / 做了什么

M19 完成了"失效 → 切换 allocator"的机制，但**如何知道电机死了**这一步仍是 oracle —— 测试里手写 `flight.motor_alive[0] = false`。真实硬件没这条捷径。M20 把这个 oracle 换成一个真正能从 ω̇ 残差反推的 **MotorFaultDetector**。

### M20a — `algo-fdir::MotorFaultDetector`（commit `074b3d5`）

```rust
pub struct MotorFaultDetector { /* persistence[4], thresholds, alive, level */ }

impl MotorFaultDetector {
    pub fn observe(
        &mut self,
        motor_cmd: &SVector<f32, 4>,           // 上一 tick 发出的推力
        omega_dot_observed: Vector3<f32>,       // LPF 后的 ω̇
        effectiveness: &SMatrix<f32, 4, 4>,     // 正向 E（来自 algo-alloc）
        inertia_inv: &Matrix3<f32>,              // J⁻¹
    ) -> HealthLevel;

    pub fn alive(&self) -> [bool; 4];
    pub fn level(&self) -> HealthLevel;
}
```

物理：悬停附近角动量方程 `J·ω̇ = Σᵢ e_torque,ᵢ · T_actual,ᵢ`。如果电机 `k` 死，`T_actual,k → 0` 而 `T_cmd,k` 仍然是 allocator 命令的值 → 残差

```
r = ω̇_obs − J⁻¹ · Σᵢ e_torque,ᵢ · T_cmd,ᵢ  ≈  −J⁻¹ · e_torque,k · T_cmd,k
```

方向上和 `−J⁻¹ · e_k` 平行。detector 对每个存活电机计算这个 **反向对齐度** × `max(T_cmd,ᵢ, 0)` 作为 score，最高分 wins。超过阈值 `residual_threshold_rad_s2` 的 tick 上累计 persistence；同一电机 `n_ticks_to_declare` ticks 持续胜出 → 标死，`HealthLevel` 跳 Emergency。

默认值：`threshold = 2.0 rad/s²`（半信号），`n_ticks = 50`（1 kHz 下 50 ms，容得下单样本噪声）。

**6 新 test**（含 4 种 dead-motor symmetry + 持续性 + 一旦标死不 un-declare）。

### M20b — `rate_loop_step` 挂上 detector（commit `1426980`）

```rust
RateLoopConfig {
    effectiveness: Matrix4<f32>,   // 新：cached 正向 E
    inertia_inv: Matrix3<f32>,      // 新：cached J⁻¹
    // ...
}
FlightState {
    motor_fault_detector: MotorFaultDetector,  // 新
    last_motor_cmd: SVector<f32, 4>,            // 新：上一 tick cmd
    // ...
}
RateLoopOutput {
    omega_dot_filtered: Vector3<f32>,           // 新：暴露给 instrumentation
    // ...
}
```

rate_loop_step 末尾 armed 时：

```rust
flight.motor_fault_detector.observe(
    &flight.last_motor_cmd,     // 上一 tick 的 cmd 配...
    omega_dot_filtered,          // ...这一 tick 的 ω̇
    &cfg.effectiveness,
    &cfg.inertia_inv,
);
let det_alive = flight.motor_fault_detector.alive();
for (slot, &d) in flight.motor_alive.iter_mut().zip(det_alive.iter()) {
    *slot = *slot && d;          // AND-monotone: 只能 alive→dead, 永不反向
}
flight.last_motor_cmd = final_thrusts;
```

**AND-monotone** 是关键设计：detector 的 `alive()` 能**新标死**一个电机（flip true → false），但永远不会把已标死（test oracle 或 future 更高层 FDIR 手写的）电机 flip 回 true。这保留了 M19c 的 oracle 路径同时允许 detector 独立工作。

1 新 test 验证此合约：pre-declare motor 2 dead，连跑 20 ticks + zero cmd（detector 看不出异常），motor 2 必须**始终**保持 dead。

### M20c — SITL end-to-end（commit `0ecef90`）

在 M18/M19c 之外**新增**一个 `single_motor_failure_detected_end_to_end_without_oracle`：

- 起点 `flight.motor_alive = [true; 4]`，**不写**任何 oracle
- `t = 2 s` 时 `sim_cfg.motor_fault_mask[0] = 0.0`
- detector 必须自己从 ω̇ 残差中找出 motor 0
- 预期检测延迟 < 250 ms（含 motor-lag 30 ms + LPF group delay 5 ms + persistence 50 ms，理论 ~85 ms，3× headroom）
- altitude err < 5 m（比 M19c oracle 的 3 m 松，因为 detection latency 期间 allocation 仍错）
- tilt < 75°（比 M19c 的 60° 松，理由同上）

M19c 作为**单元级**检查（zero 延迟，纯 failover 路径），M20c 是**集成级**检查（detector + failover + rate loop 合作）。两个都留作 regression guard。

## Why / 为什么这么做

### 为什么用 torque-direction alignment 而不是 RPM 反馈

最直接的 FDIR 思路是 **bidirectional DShot** + RPM 反馈 → 每个电机期望 / 实际 RPM 对比。问题：

1. **需要硬件**：bidirectional DShot 要求 ESC 支持 telemetry 协议，SITL 没这个
2. **采样率低**：典型 RPM 反馈 ~200 Hz vs 控制环 1 kHz，延迟另加 5 ms
3. **噪声**：低 RPM 时信号质量差，零 RPM 时彻底没数

使用 **ω̇ 残差对齐** 则：
- 只需 IMU（飞控必备）
- 1 kHz 控制环原生支持
- 对**突发完全失效**（motor_fault_mask 0.0 = runaway / lock-up / ESC 崩）效果最好，对**渐变性能衰减**（mask 0.5 = 功率下降）效果次之但仍可用

硬件做出来之后 M30+ 再加 RPM 交叉校验作为**独立冗余信号**，和 IMU-based detector 共同投票。

### 为什么选 persistence-based 阈值而不是 Kalman filter

也可以写一个正式的**电机状态 Kalman filter**（每个电机 1 维状态"alive-ness"），观测是残差，过程模型是马尔可夫链。但：

- 调 process/measurement noise ≈ 调 threshold + persistence，技术上等价但**黑箱 10×**
- 证明 Kalman 1D 的收敛性（一阶 Lyapunov）能写，但比 "count 50 consecutive hits" 复杂 100×
- 硬件同行看到 persistence-based 一眼懂，PX4 / ArduPilot 的 critical FDIR 都用这种
- Kani 形式化不了连续 Kalman，但能证 persistence-based detector 的"一旦 declared, 永不撤回"（M20d 拟计划）

### 为什么 `last_motor_cmd` 放在 `FlightState` 而不是 `RateLoopConfig`

`FlightState` = **每 tick 更新的运行态**。`RateLoopConfig` = **固定 boot-time 参数**。last_motor_cmd 每 tick 都变 → 属于运行态。

严格意义上 last_motor_cmd 是 detector 内部状态，我也想过把它藏在 detector 里。但这样 `observe()` 签名就变成 `&mut self, ω̇, E, J⁻¹`，少一个参数 → 调用方无法**显式看到**输入配对。显式 out-of-band pairing 在我看来更清楚，面 review 时也能一眼检查。

### 为什么 AND-monotone 而不是直接替换

```rust
// ❌ Overwrite:
flight.motor_alive = flight.motor_fault_detector.alive();

// ✅ AND-monotone:
flight.motor_alive[i] = flight.motor_alive[i] && detector.alive()[i];
```

问题：detector 默认 `alive = [true; 4]`。M19c 的 oracle test 预先写 `motor_alive[0] = false` → rate_loop_step 调用 → detector.observe 看不到异常（因为 last_cmd 是正常的）→ detector.alive() = [true; 4] → overwrite 就把 oracle 的 false **resurrect** 掉。

AND-monotone 保证：detector 永远只能**declare 新故障**，不能 un-declare 已 declared 的。语义上也呼应 `HealthLevel` 的"飞行中不回滚"契约。

### 为什么 detector 只 armed 时跑

Disarmed 地面状态：motor cmd = 0（被 arm gate 截断），IMU 可能被人手动摇晃（模型没模 ground handling）→ 残差可以随便大。不 gate 的话一次搬运台架就会误报 motor fault。

Armed 意味着飞行员（或 offboard）已经说"这玩意儿要飞了，给它通电开始控制"。之前的所有 ground handling 误差都不应该污染飞行时的 detector 状态。

### 为什么 default threshold = 2.0 rad/s²

手算 + sim 数据综合：

- **完全失电**单电机在 hover 位置（cmd 仍 0.6125 N）给出残差 ≈ 4.3 rad/s²（实测我 M18 analysis 给出的数字；M20a 测试里 `synthetic_omega_dot` 可重现）
- **50% 功率衰减**给出一半 ≈ 2.2 rad/s²，**刚过阈值**
- 合理噪声（ICM-42688 gyro noise ~0.01 rad/s RMS，α ≈ 1 rad/s² 短周期抖）要求 threshold ≥ ~1.0

所以 2.0 是"完全失电一定抓得到，noise 看不到" 的中间点。

### 为什么 `n_ticks_to_declare = 50` ms at 1 kHz

典型单-tick IMU glitch ~1 tick。20 ms motor-lag 期间 actual thrust 还在 ramp-down，residual 还没到最大。LPF group delay ~5 ms。50 ms ≈ "两个时间常数 + 一点 margin"，既过滤掉单样本噪声又控制检测延迟不过长。

M30+ 硬件实测后 tune 到真实噪声分布，目前 50 是工程经验估计。

### 为什么 altitude err 从 3 m 松到 5 m（vs M19c）

M19c 的 oracle：故障和切换**同一 tick 完成**，lift 不丢，altitude 几乎不动。
M20c 的 detector：故障 → 检测 ~ 80 ms 的 **allocation 错误窗口**。在此期间 3 个存活电机 × 0.6125 N = 1.84 N < hover 2.45 N → 下跌 0.61 N / 0.25 kg ≈ 2.45 m/s² × 80 ms → **0.78 cm** position drop + 0.2 m/s velocity carry。

理论最大高度掉 ~1 m（包括恢复期 overshoot）。但：
- Sim 有噪声（IMU、GPS ~测量更新）
- LPF group delay 让 detector 比手算更慢
- Position controller 见 altitude err 后加 thrust → 有可能跑过 saturation limit → 下跌加剧

给 5 m 是 5× 理论上限，留给 CI 变动 + 未来 sensor noise tune。

## How it's verified / 怎么验证的

本地 cargo 不在 PATH，依赖 CI：

```bash
$ cargo test -p algo-fdir
# 原 19 + 新 6 = 25 tests

$ cargo test -p app-copter --lib
# 原 29 + 新 1 = 30 tests

$ cargo test -p sim-hil --release
# 原 42 + 新 1 = 43 tests

$ cargo test --workspace
# ~255+ tests

$ cargo kani -p algo-fdir
# 原 5 个 HealthLevel + 1 个 SensorRejectionCounter，M20 没加（M20d 计划）
```

## Follow-ups / 遗留

- **M20d — Kani on MotorFaultDetector**
    - 证 "once alive[i] = false, 后续 observe() 永不让 alive[i] = true"（latch monotonicity）
    - 证 "persistence saturates, no overflow"
    - 证 "dead_count() ≤ 4"
- **M20e — 多电机失效 detector**
    - 现版本假设 "一次只挂一个"。两个电机同时挂时，残差方向**混合**，attribution score 最大者只能是其中之一，另一个可能被漏
    - Fix：多遍 detection，第一个标死后再 run 一遍，找下一个
- **M21 — Mueller-style 3-motor attitude controller**
    - 位置控制器目前不知道 yaw 在 spin，body-frame 指令会漂
    - 独立 milestone，不在 M20 scope
- **M22 — 渐变性能衰减**（热失控 / ESC 降额 / 桨叶裂）
    - 当前测试仅 `motor_fault_mask = 0.0` 完全失电。软失效需要 detector 在 threshold 附近更灵敏（动态阈值？）
    - 和 MAVLink `MOTOR_INFO` telemetry 挂钩，让 GCS 看到每个 motor 的 "health score"
- **硬件集成 M30+**：bidirectional DShot RPM 作为独立冗余信号
