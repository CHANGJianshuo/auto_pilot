# 0032 — app-copter 接入 GPS/mag/baro + 每传感器 FDIR 计数器

**Commit**: `870f146`  **Date**: 2026-04-21  **Milestone**: M3.0

## What / 做了什么

把 `algo-ekf` 的三个 measurement update（GPS / mag / baro）+ `algo-fdir` 的 `SensorRejectionCounter` 一次性接进 `app-copter::FlightState`。这之后 `app-copter` 里的 EKF 是**完整的双环**（predict + update），并且每个测量源的健康由 FDIR 计数器监控。

新增：
- `FlightState` 字段 `gps_health` / `mag_health` / `baro_health`（各自 `SensorRejectionCounter`）
- `FlightState::overall_health() -> HealthLevel`（3 个计数器里取最坏）
- 3 个测量应用函数：
  - `apply_gps_measurement(&mut FlightState, &GpsMeasurement) -> f32 (nis)`
  - `apply_mag_measurement(...)`
  - `apply_baro_measurement(...)`
- 4 单元测试（共 9）

## Why / 为什么这么做

### 为什么 `FlightState` 而不是 `RateLoopConfig` 里

健康状态是**动态**的（随测量流变化），配置是**静态**的（启动时定）。放在 FlightState 里语义对。

### `apply_*_measurement` 返回 NIS 的用途

NIS 本身是**可观测性的诊断信号**：
- 平时 < 3 说明 EKF 模型和传感器匹配
- 突然变 > 11.345 被门拒，counter 加 1
- 持续偏高但 < gate 可能是模型有偏（比如 mag_bias 没校准好）

把 NIS 返给调用方让**遥测层**（MAVLink / 日志）记录历史。未来做飞后分析可以直接看 NIS 曲线判断是 EKF 调不对还是传感器真有问题。

### 为什么 `overall_health` 取**最坏**（max）

多源 FDIR 的标准组合规则：任何一个源 Degraded 都让系统整体 Degraded。这样：
- GPS Degraded + mag Healthy + baro Healthy → 整体 Degraded（GPS 可疑，系统应该小心）
- 三个都 Healthy → Healthy
- GPS Emergency + baro Failed → Failed

这种 **worst-case roll-up** 是 ARINC 653 风格 FDIR 的标准（也是 PX4 ekf2 的健康语义）。

### 为什么接受 / 拒绝**都**过计数器

在 M1.14 的 counter 设计里：
- `observe(true)`：streak 清零，level 不变（单向 FDIR）
- `observe(false)`：streak 加一，可能触发升级

这意味着我们必须**无条件**调用 `flight.*_health.observe(r.applied)`，即使测量被接受了。不然 streak 不会清零，过去的拒绝会永远黏着。

`apply_*_measurement` 确保这一点：

```rust
if r.applied {
    flight.state = r.state;       // 接受才更新状态
    flight.covariance = r.covariance;
}
flight.gps_health.observe(r.applied);  // 无论接受与否都 observe
r.nis
```

### 单元测试 `repeated_gps_outliers_degrade_health` 的意义

15 次 10 km 离谱测量 → 全部被 χ² 门拒 → counter streak 达到 10 → 升级到 Degraded。**这就是** SensorRejectionCounter 在实际 FDIR 系统里的价值。

`overall_health()` 立刻反映这个 Degraded（其他两源仍 Healthy）。控制层可以读到这个信号决策（比如"GPS 不稳 → 切 baro-only 定高"）。

### 为什么 4 个测试都是**单方向**（GPS / mag / baro / overall）

每个 measurement update 有独立的数学（GPS 是线性 H，mag 是非线性 H，baro 是 1D），独立的失效模式。**单独测试**比端到端更容易定位 bug。端到端的长序列 proptest 放到 M3.1 做（SITL 跑会更真实）。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib
test result: ok. 9 passed; 0 failed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **SITL 桥**（M3.1）：Gazebo / AerialGym 喂 IMU + GPS + mag + baro 给 `FlightState`，跑真动力学
- **端到端 EKF 收敛 proptest**：drive FlightState with noisy measurements, verify converges to truth. 这个在 M3.1 SITL 上做更有意义
- **MAVLink HEALTH_STATUS 下行**：`overall_health()` → `MAV_STATE`
- **NMPC 集成时用 `overall_health`**：比如飞控进入 Degraded → NMPC 减速、收紧加速度约束
- **Scheduled rates**：现在 apply_*_measurement 是按 driver 节奏调用（GPS 5 Hz, mag 25 Hz, baro 50 Hz），调用方决定何时调。这里**不强制** schedule，保持灵活
