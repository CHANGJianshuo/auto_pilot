# 0028 — app-copter 组装 rate loop（EKF → INDI → 分配）

**Commit**: `5f4e812`  **Date**: 2026-04-21  **Milestone**: M2.3

## What / 做了什么

**第一次把 M1 和 M2 做出来的所有算法模块缝在一起**。`app-copter` 从"只打印 task graph 的 placeholder"升级到真正的 rate loop 库 + 演示 binary。

新增：
- `app-copter` 现在是 **dual-target**（library + binary）
- `pub struct RateLoopConfig` —— 所有启动时的调参
- `pub struct FlightState` —— 跨步保持的飞行状态
- `pub struct RateLoopOutput` —— 单步输出
- `pub fn rate_loop_step(cfg, flight, imu, dt, rate_cmd) -> RateLoopOutput`
- `pub fn default_config_250g()` —— 250 g X-quad 预设
- 3 集成测试
- `main.rs` 简化成：拉起 `default_config_250g()`、跑 100 步、打印采样值

## Why / 为什么这么做

### 管道结构

```
ImuSample (from driver or mock)
    │
    ▼
predict_step(state, cov, imu, noise, dt)     ← EKF M1.9c
    │
    ▼
gyro_lpf.update(imu.gyro)                    ← LPF M2.0
    │
    ▼
α = (gyro_filtered − last_gyro) / dt         ← finite diff
    │
    ▼
alpha_lpf.update(α)                          ← second LPF
    │
    ▼
compute_torque_increment(indi_input)         ← INDI M2.0
    │
    ▼
allocate(e_inv, virtual_cmd)                 ← allocation M2.1
    │
    ▼
saturate(thrusts, min, max)                  ← per-motor clamp
    │
    ▼
motor_thrusts_n: SVector<f32, 4>             ← ESC drive signal
```

每一步都用前面 M1/M2 已证/已测的函数。**没有新算法**，只是把它们组合。

### 为什么 dual-target

把 `rate_loop_step` 放在 `lib.rs` 里让它：
- **可外部测试**（`tests/` 目录或别的 crate 能 import）
- **可重用**：固件、SITL、HITL 三种场景都共用同一个函数
- **文档化**：rustdoc 能显示整个管道

`main.rs` 只是一个**演示 harness**。真实 firmware 会另写 `app-copter/src/bin/firmware.rs`（M3+）用 embassy 起 ISR 驱动。

### 为什么用 `&mut FlightState` 和 `&mut RateLoopConfig`

`FlightState` 必须 mut：EKF state + covariance + last_gyro_filtered 都跨步累积。

`RateLoopConfig` 看起来是 const，但 `LowPassFilterVec3` 有内部状态（`output` + `initialized`）。所以 config 也得 mut。

**权衡**：
- 把 LPF state 挪到 `FlightState`：更纯净，但 `cfg` 名不副实
- 把 LPF 做成纯函数 + 调用者自管 state：代码更啰嗦
- **当前选择**：LPF 是"配置时构造、rate loop 里 mut"的对象

飞控 1 kHz 调用 LPF 是默认路径，让 LPF 对象自带 state 让代码最短。

### α 的有限差分：不怕不怕

IMU 只给 ω 不给 α。要做 INDI 就要 α = dω/dt。直接有限差分：

```rust
α_raw = (gyro_filtered_now - gyro_filtered_prev) / dt
α_filtered = alpha_lpf.update(α_raw)
```

两级滤波：先 80 Hz 低通压高频噪声（IMU MEMS 自身 bandwidth 限制）、再有限差、再 30 Hz 低通压差分放大后的残留高频。这是 indiflight、Agilicious 用的方法。

为什么两个 cutoff 不一样？**先低通再差分**会放大 cutoff 附近的噪声（除以 dt 效果），所以第二级 cutoff 比第一级低。30 Hz 覆盖所有 "有控制意义的" 姿态动态（人眼辨别 10 Hz 已经是抖动）。

### 测试数字的意义

`rate_loop_stationary_stays_stable_for_one_second`：
- 1000 次 1 ms 迭代 → 1 秒真实时间
- 每次都做完整 predict + 4 次分配 + 2 次 LPF + 1 次 INDI
- **所有** EKF 和 IndiInput 的 proptest 都已经通过了，这里只是验证组合不炸
- 用 debug build 跑 1 秒真实时间在容器里 0.66 s，release 估计 < 0.1 s

`rate_loop_commands_positive_roll_shifts_motors_asymmetrically`：
- 10 步静态预热（让 LPF 初始化）
- 然后一步 `rate_cmd = (2 rad/s, 0, 0)` 
- 断言 y<0 motors (M2, M3) 拿更多推力

**对称差分**是最敏感的 bug 探测器 —— 任何一个地方（INDI 符号、分配矩阵、LPF 状态）错了，这种断言就会翻。

### demo 输出的含义

```
step 0:  motors=[0.613 0.613 0.613 0.613] N, q=(1.000, 0, 0, 0)
step 49: motors=[0.613 0.613 0.613 0.613] N, q=(1.000, 0, 0, 0)
step 99: motors=[0.613 0.613 0.613 0.613] N, q=(1.000, 0, 0, 0)
```

- `0.613 N × 4 ≈ 2.45 N = hover_thrust` ✓
- q 保持单位 ✓（100 ms 没偏离）
- 对称（所有 motors 相等）✓（零扰动零偏差）

这是**最 boring 的好输出** —— 就是我要的。任何偏差就说明 bug。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib
test result: ok. 3 passed; 0 failed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished

$ cargo run -p app-copter --release
auto_pilot — task graph: ...
Rate-loop demo (100 ms of stationary IMU):
  step   0: motors=[0.613 0.613 0.613 0.613] N, q=(1.000, 0, 0, 0)
  step  99: motors=[0.613 0.613 0.613 0.613] N, q=(1.000, 0, 0, 0)
```

## Follow-ups / 遗留

- **attitude loop**（外层）：把期望姿态 → rate_cmd
- **position loop**（外层）：把期望位置 → 期望姿态
- **SITL 桥**（`sim-hil`）：把 rate_loop_step 接进 Gazebo 物理引擎，看真的飞起来
- **NMPC 外环**：替换 position loop 的简单 PID
- **嵌入式绑定**：在 `thumbv7em-none-eabihf` 下用 embassy-stm32 做 ISR-driven rate loop
- **WCET 测量**：在 Cortex-M7 上实测 rate_loop_step 的最坏执行时间（目前在 x86 上 ~2 µs/step）
