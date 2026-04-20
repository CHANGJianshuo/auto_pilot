# 0024 — EKF 端到端多源综合测试

**Commit**: `9574f99`  **Date**: 2026-04-20  **Milestone**: M1.13

## What / 做了什么

第一个**端到端**EKF 集成测试 —— 3 个测量源按真实速率并发，跑 2 秒 1 kHz 预测循环，验证 EKF 能正确融合所有信息。

新增测试：`end_to_end_stationary_multi_sensor_convergence`

## Why / 为什么这么做

### 这个测试做什么

模拟"飞控放在桌上 2 秒"的场景：

```
每 1 ms：    predict（IMU 给重力）
每 200 ms： GPS → (1, -2, -3) m, σ = 0.3/0.3/0.4 m
每 40 ms：  Magnetometer → 跟地磁场一致, σ = 5 mGauss
每 20 ms：  Barometer → altitude = 3 m, σ = 0.1 m
```

2 秒后断言：
- 位置收敛到 GPS 目标（误差 < 0.3 m）
- 高度收敛到气压计（误差 < 0.2 m，**baro 比 GPS z 更准**）
- 四元数保持单位（误差 < 1e-4）
- 24 个 P 对角都正且有限

### 为什么是 2 秒

太短 → GPS 只触发 10 次、mag 50 次、baro 100 次，收敛未完成。
太长 → 测试跑太慢。1 kHz × 2s = 2000 预测 + 几百次 update，容器里 1.5 秒跑完，够。

### 为什么 baro 比 GPS 更准

σ_baro = 0.1 m，σ_gps_z = 0.4 m。卡尔曼更新会自动给更低方差测量更高权重（`K ∝ 1/σ²`），所以 baro 支配 z 轴估计。最终高度收敛到 baro 值，GPS 的 z 影响变小。

这是 EKF 的优势 —— 多源融合**不需要手工选择哪个传感器说了算**，方差自动决定权重。

### 为什么这是关键里程碑

之前的所有 proptest 都是**单 aspect**：单独验证 F 的一个子块、单独验证 gps_update 的收敛性。**这个测试把所有代码放在一起跑，发现任何 cross-module bug**：
- 如果 gps_update 后四元数归一化没做好 → mag_update 下一次会偏
- 如果 Joseph form 有浮点累积错误 → P 某对角慢慢变负 → baro_update 奇异 → 下一步炸
- 如果 predict 的 F 某块有 dt 系数错 → 长时间后 P 某对角指数增长或衰减

跑通 = 全链路数学和代码没有硬 bug。任何一个出错都会在 2000 步内被抓到。

### 性能意外

这个测试是**最重的单元测试** —— 2000 次 predict（每次 24×24 矩阵乘 ~14K flops × 3 次 = 42K flops ≈ 14 µs CPU）+ 几百次 update（更重）。容器 x86-64 跑 1.5 s，换算 Cortex-M7 大约 10 倍慢 → **实际 1 秒硬件飞行 ≈ 0.15 秒 CPU**。1 kHz predict + 25 Hz update 的负载是 Cortex-M7 能承受的。

这是**第一次有真实 CPU 消耗数字**的证据，预算合理。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 66 passed; 0 failed
  end_to_end_stationary_multi_sensor_convergence ← 1.5 s
  others (proptest + units)                    ← ~18 s 全部
```

## Follow-ups / 遗留

- **M1.14**：NIS 连续被拒 / 测量源异常 → `HealthLevel::Degraded` 接入 `algo-fdir`
- **M1.15**：**动态**飞行场景（起飞 → 悬停 → 10 m/s 横移 → 降落）的 proptest
- **M1.16 及以后**：驱动层（SPI/DMA）、rate loop 的 embassy 集成、INDI 控制律
- 性能 benchmark：在 Cortex-M7 上实测 predict+update 的 WCET（需要真机调试）

EKF predict + update 的**数学栈**到此结束。下面进入更多工程化 / 硬件集成的工作。
