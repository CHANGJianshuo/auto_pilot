# 0041 — GPS 故障注入 + FDIR 端到端验证

**Commit**: `c53099e`  **Date**: 2026-04-21  **Milestone**: M5.0

## What / 做了什么

首个**系统级 FDIR 端到端测试**：SITL 注入 GPS 恶意偏移，验证整条 FDIR 链路（χ² 门 → SensorRejectionCounter → HealthLevel → overall_health）完整工作。

新增：
- `pub enum GpsFault { None, Offset(Vector3), Stuck(Vector3) }`
- `pub fn sense_gps_with_fault(cfg, state, rng, fault) -> GpsMeasurement`
- `sense_gps` 现在委托到新函数 with `GpsFault::None`
- 1 SITL 测试：`gps_outliers_degrade_sensor_health`（共 11）

## Why / 为什么这么做

### 之前各层分别测过，但没端到端验证

| 组件 | 测试位置 | 测过什么 |
|---|---|---|
| χ² gate | algo-ekf 各 update 测试 | "单次 outlier 被拒" |
| SensorRejectionCounter | algo-fdir | "10 连拒 → Degraded" |
| apply_gps_measurement | app-copter | "applied → counter observe(true)" |
| overall_health | app-copter | "15 连拒 GPS → Degraded" |

本 commit 把**飞行环境下**这四层串起来：真飞机 + 真 GPS 传感器模型 + 真 EKF measurement update。任何一层有 bug 都会在这里暴露。

### 测试的具体脚本

```
0-2 s:   clean GPS → EKF 接受正常更新，gps_health = Healthy
2-6 s:   每次 GPS 读数 +500 m x 方向 offset → χ² 约 6×10⁶ >>> 11.345
         → 每个 sample 被拒 → streak 递增
         → 连拒 >10 → level 升到 Degraded
         → overall_health 跟着升
         → 同时 vehicle 要保持稳定（baro + mag 还能 anchor）
```

6 s 内 20 个 GPS sample 以 200 ms 间隔发出，`i >= 2000` (t >= 2s) 时 fault=Offset。整个后半段 20 个连续拒绝。默认阈值 `DEFAULT_N_DEGRADE = 10`，所以第 10 个拒绝时 level 升到 Degraded。

### 为什么 500 m 偏移

目标：让 NIS 远远超过门值。
- 500 m 位置残差 + σ≈0.3m → residual²/σ² = 500²/0.09 ≈ 2.8 × 10⁶
- χ² 门值 11.345
- 比值 2.5 × 10⁵ 倍，绝对不可能通过

用这个确定性故障避免 proptest 样本太小时偶尔被接受的风险。

### 为什么 vehicle 不会失控

Baro 仍然正常（间隔 20 ms 高频率）、mag 正常、IMU 正常。EKF 仍然正确维护 attitude 和 altitude。GPS 被拒意味着水平位置的协方差会慢慢增长，EKF 逐渐"不相信"自己水平位置估计。但：
- 水平 drift 很慢（gaussian wind 已经 cfg 成默认 0）
- 测试 4 s 内 drift < 50 m（断言 `pos.norm() < 50.0`）

这正是 FDIR 的**价值**：坏 GPS 不影响正常传感器，vehicle 仍能控制，只是水平定位逐步恶化。操作员看到 `overall_health >= Degraded` 就该采取行动（切回家、切手动、切 visual-inertial 模式）。

### 设计细节：为什么 GpsFault 是 enum 不是 struct

Enum 让每个 fault 模式**语义清晰**：
- `None` —— no fault
- `Offset(v)` —— 加偏移（模拟 multipath / spoofing，偏移但还有噪声）
- `Stuck(p)` —— 固定位置（模拟驱动 hang 或 GPS 失锁）

将来加 `Dropout`（完全没输出）、`JitterBurst`（短暂高噪）等扩展只要加 variant，不破坏 API。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil gps_outliers_degrade_sensor_health
test result: ok. 1 passed; 0 failed

$ cargo test -p sim-hil
test result: ok. 11 passed; 0 failed  (含 M3.1 / M3.1b / M4.1 场景)

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **mag 故障**：`MagFault { Offset, Stuck, Interference }` — 比 GPS 更复杂因为 h() 非线性
- **IMU 故障**：gyro bias 跳变、accel 饱和
- **Motor 故障**：单电机死机 / 卡滞 —— 不走 FDIR counter（不是传感器），但可以让 INDI + 分配 做 reconfiguration
- **Fault recovery**：GPS 清洁后自动清零 streak（counter 已经有，但 level 单向不降）→ 地面 reset 流程
- **端到端 RTL（Return To Launch）**：`overall_health >= Emergency` 自动切换 setpoint 到起飞点
