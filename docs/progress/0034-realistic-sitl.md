# 0034 — 真实 SITL：噪声 + 电机滞后 + 气动阻力 + 风扰

**Commit**: `ac59ea2`  **Date**: 2026-04-21  **Milestone**: M3.1b

## What / 做了什么

上一步（0033）的 SITL 是"数学正确"但**物理理想**。这一步把真实世界四种最大的非理想效应加进去：

1. **Gaussian 传感器噪声**（per-axis σ，符合 ICM-42688 / u-blox F9P / RM3100 / BMP388 数据手册）
2. **电机一阶滞后**（τ = 20 ms；实际指令推力和真实推力之间有延迟）
3. **线性 + 二次气动阻力**（`F = −k_lin·v_rel − k_quad·|v_rel|·v_rel`）
4. **恒定风扰**（NED 风向量，相对风速参与阻力计算）

新增：
- `mod rng` —— 自研 Xorshift64 + Box-Muller Gaussian（无外部 crate 依赖）
- `NoiseConfig` / `SimConfig::realistic_dynamics(wind)`
- `SimState::motor_thrusts_actual_n`（跟踪滞后后的实际推力）
- `accel_world(cfg, state)` 取代 `accel_world_from_thrusts`（更简洁，用 state 里的实际推力）
- 所有 `sense_*` 函数签名改成接受 `&mut SimRng`
- 2 个新的闭环测试：带噪声/滞后/阻力 + 带风扰

## Why / 为什么这么做

### 为什么不用 `rand` crate

- **零外部依赖**（workspace 已经很重）
- **Xorshift64** 30 行代码、速度和 `rand::rngs::SmallRng` 同级
- **Box-Muller** 缓存一个值，两次 `gaussian()` 摊销一次 `sqrt + log + sin/cos`
- 测试里验证 10 000 样本 mean ≈ 0 ± 0.05，var ≈ 1 ± 0.1

对 SITL 这种"**确定性可复现**比性能更重要"的场景，Xorshift 足够好。

### 一阶电机滞后的实现

```rust
if cfg.motor_tau_s > 0.0 && dt_s > 0.0 {
    let alpha = dt_s / (cfg.motor_tau_s + dt_s);
    state.motor_thrusts_actual_n += (cmd - state.motor_thrusts_actual_n) * alpha;
}
```

这是 exp filter 的离散化。τ = 20 ms 意味着指令阶跃后 20 ms 达到 63%，60 ms 达到 95%。比真实电机（BLDC 带 ESC）稍快但同量级 —— 真实数据 τ 常 25-40 ms 取决于螺距和 KV 值。

### 为什么同时保留"指令推力"和"实际推力"

调用方传入**期望**推力（控制器输出）。仿真内部用**滤过的**推力算力和力矩。IMU 感受到的**也是实际**推力（因为它感受到的是实际加速度）。`accel_world()` 因此读 `state.motor_thrusts_actual_n`，不是调用方刚传进来的指令。

### 空气动力学的模型选择

**只用线性 + 二次**（不做升力、诱导阻力、螺旋桨桨盘载荷之类）：

| 速度 | 主导项 |
|---|---|
| < 0.5 m/s | 线性（粘性主导） |
| 0.5 - 5 m/s | 混合 |
| > 5 m/s | 二次主导（动压） |

系数 `k_lin = 0.05 N·s/m, k_quad = 0.02 N·s²/m²` 是小 quad 的典型量级。对悬停/慢速飞行足够。

### 为什么"带噪声+滞后+阻力"测试的容差被放宽到 3m

理想条件下（0033）容差 0.5 m 就够。加上真实效应后：

- **噪声**：EKF 位置估计抖动 ~10 cm
- **电机滞后**：控制响应延迟造成 ~15-50 cm 过冲
- **阻力 + 无积分项的 P-P cascade**：**稳态误差**（steady-state bias）~几十 cm到 2 m

合起来 3 m 以内属于 "**不发散**" 的合理范围。这是**诊断上限**，不是"精度目标"。

精度目标要靠：
- **积分项**（PI-P 或 PID cascade）—— 消除稳态误差
- **NMPC**（M4）—— 约束感知的最优控制
- **Feed-forward 扰动**（EKF 估计的 wind_ned 反馈）

### 风扰测试的意义

```
wind_ned = (3, 0, 0) m/s   头风
drag = -k · (v - wind)
```

静止悬停时 `v = 0, v_rel = -wind`，所以 `drag = +k·wind`（前向力）。P-P 控制器会让 vehicle 向前漂移到新的平衡点，控制律会让其倾斜对抗。最终稳态**不在原点**，但也**没飞走**。

断言 `|p| < 5 m` 允许这种偏移，同时捕获真正的发散。

### 为什么 run_closed_loop_flight 是 helper

两个闭环测试（理想 vs 真实 vs 风扰）共享大量样板。提取 helper 让每个测试专注于**条件差异**：

```rust
fn run_closed_loop_flight(sim_cfg: &SimConfig, seed: u64, steps: usize) -> SimState
```

`seed` 让 proptest 以后扩展（不同种子跑蒙特卡洛）；`steps` 允许收敛 vs 稳定性等不同时间尺度。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil
test result: ok. 9 passed; 0 failed
  含 rng 3 测试、closed-loop 3 变体（ideal / realistic / wind）

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M3.1c**：**随机化 proptest**——不同 seed × 不同风场 × 不同飞行轨迹，统计 EKF/控制律在 256 组场景下的表现
- **M3.2**：Gazebo Harmonic 集成（gz-transport）
- **PID cascade**：加积分项消除稳态偏差
- **Motor saturation redistribution**（QP 分配，M2.1b）下的 realistic 测试
- **故障注入**：单电机失效（thrust[i] = 0）、GPS spoofing（突然 +100m）、mag 干扰
- **多传感器竞争**：GPS 和 baro 都给高度，EKF 如何融合（方差权衡）
