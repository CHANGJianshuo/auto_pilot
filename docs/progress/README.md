# Progress log / 进度日志

每完成一个里程碑或一次有意义的提交，在这里加一份简短记录。格式：`NNNN-slug.md`，按时间顺序递增编号。这样未来任何人（包括你自己、下一个对话里的 Claude）扫一遍就知道项目到哪了、为什么这么做、下一步该干啥。

| # | 日期 | 标题 | Commit | 摘要 |
|---|------|------|--------|------|
| [0001](./0001-m0-scaffold.md) | 2026-04-20 | M0 工作区脚手架 | `7ed3421` | 12-crate Cargo workspace、Apache-2.0、CLAUDE.md 规约、toolchain 固定到 1.88、docs/plan.md 凝练路线图 |
| [0002](./0002-docker-container.md) | 2026-04-20 | Docker 开发容器 | `0bfef4c` + `e83b695` | Ubuntu 24.04 + Rust 1.88 + ROS 2 Jazzy + probe-rs + gh CLI；docker-compose 三 profile（dev/gpu/sim），校园网下 apt/gh 各种兜底 |
| [0003](./0003-m0.5-verification.md) | 2026-04-20 | M0.5 验证全绿 | `4c23282` | 在容器里跑通 cargo check / clippy -D warnings / test；调教 workspace lints（去掉 pedantic+cargo 组，保留 safety-critical deny） |
| [0004](./0004-quaternion-invariant.md) | 2026-04-20 | M1.0 四元数归一化不变量 | `2a74d09` | `algo-ekf::State::normalize_attitude` + proptest（256 样本） + 3 个边界用例。**项目第一个真正的测试**。 |
| [0005](./0005-kani-harness.md) | 2026-04-20 | M1.1 Kani 形式化证明 | `e421dc2` | 重构 normalize_attitude 让退化检查先于 sqrt；Kani 0.67.0 装好，证明零四元数 100% 被拒绝，1.8 s 通过。**项目第一个形式化证明**。 |
| [0006](./0006-priority-proofs.md) | 2026-04-20 | M1.2 Priority 排序证明 | `01c8c26` | `Priority::{rank, from_rank, ALL}` + 5 个 Kani 证明（全序/传递/反对称/round-trip/越界无 panic）。49 ms 证完。 |
| [0007](./0007-kani-in-image.md) | 2026-04-20 | M1.3 Kani 装进 Dockerfile | `f371d3c` | 镜像自带 kani-verifier + CBMC + nightly toolchain；`docker run --rm` 即可跑所有证明。 |
| [0008](./0008-healthlevel-proofs.md) | 2026-04-20 | M1.4 HealthLevel 状态机 | `94f1c24` | 单向 FDIR 不变量（monotone transition + ground-only reset）+ 5 个 Kani 证明 21 ms 通过。 |
| [0009](./0009-state-24d-vector.md) | 2026-04-20 | M1.5 State ↔ 24-D 向量 | `33fdc0d` | 24 维序列化 + `idx` 模块、2 个 256-sample proptest round-trip、EKF 协方差矩阵的数据基础。 |
| [0010](./0010-imu-buffer.md) | 2026-04-20 | M1.6 IMU 环形缓冲 + mock | `68fa83d` | `heapless::spsc` SPSC ImuSource + MockImuSource 重放、FIFO proptest。生产/SITL 同一套 trait。 |
| [0011](./0011-ekf-predict.md) | 2026-04-20 | M1.7 EKF predict 步 | `d5ca634` | Strapdown IMU 积分（四元数指数 + NED 重力 + 速度/位置） + 5 个 proptest（自由落体、水平静止、零角速度、四元数归一化、bad-dt）。 |
| [0012](./0012-covariance-scaffold.md) | 2026-04-20 | M1.8 协方差矩阵骨架 | `cf8196a` | `Covariance = SMatrix<24,24>`、`initial_sigma` 常量、`initial_covariance()` 对角 P₀、`enforce_symmetry()` + proptest。为 M1.9 的 F·P·Fᵀ+Q 铺路。 |
| [0013](./0013-process-noise-placeholder-f.md) | 2026-04-20 | M1.9a 过程噪声 Q + F=I 占位 | `81f1a25` | `ProcessNoise` 每秒强度 + `build_process_noise(dt)` + `predict_covariance(P,F,Q)` 完整数据流。F=I 先占位，M1.9b 替换真实 Jacobian。 |
| [0014](./0014-kinematic-f.md) | 2026-04-20 | M1.9b-0 动学 F（dp/dv = I·dt） | `f9c204c` | F 矩阵填入唯一纯线性子块；2 个 proptest 验证 cross-block 正确生成（P[p, v] ≈ dt·σ_v²）。 |
| [0015](./0015-dq-dq-jacobian.md) | 2026-04-20 | M1.9b-1a ∂q/∂q Jacobian | `c228afc` | 四元数自耦合 = 右乘矩阵 R(δq)；Hamilton 积数值验证 + determinant=1 proptest。F 的最重要一块。 |
| [0016](./0016-dq-dbg-jacobian.md) | 2026-04-20 | M1.9b-1b ∂q/∂b_g Jacobian | `10d6d41` | 陀螺偏差对姿态的 4×3 块 `-(dt/2)·L(q)[:, 1:4]`；有限差分 proptest 256 样本匹配闭式公式到 1e-4。 |
| [0017](./0017-dv-dq-dv-dba.md) | 2026-04-20 | M1.9b-1c ∂v/∂q + ∂v/∂b_a | `06c32b7` | 旋转矩阵对 q 的偏导 3×4、加速度偏差的 `−R(q)·dt` 3×3；两个有限差分 proptest 同时通过。 |
| [0018](./0018-dp-dq-dp-dba.md) | 2026-04-20 | M1.9b-1d ∂p/∂q + ∂p/∂b_a | `704d544` | 位置的二次项 `½·R(q)·f·dt²` 对 q 和 b_a 的偏导；**F 矩阵所有非平凡块全部闭合**。EKF predict 半段数学收尾。 |
| [0019](./0019-predict-step-end-to-end.md) | 2026-04-20 | M1.9c predict_step 端到端 | `4993ce4` | 单入口函数 `predict_step` + 1000 步悬停不变量 + 50 000 步随机序列 proptest。**EKF predict 半段完全收官**。 |
| [0020](./0020-gps-innovation.md) | 2026-04-20 | M1.10a GPS innovation | `edcc0e8` | `GpsMeasurement`/`GpsInnovation` 类型 + NIS（χ² gating）+ block 抽取 S = P_pos + R（200× 快于实体化 H）。EKF **measurement** 半段开始。 |
| [0021](./0021-gps-update-joseph.md) | 2026-04-20 | M1.10b GPS Kalman + Joseph | `ed3114f` | `gps_update()` 完整卡尔曼增益 + Joseph 协方差 + χ²=11.345 外值门；20 步 proptest 256 个随机目标全都在 0.5m 内收敛。 |
| [0022](./0022-magnetometer-update.md) | 2026-04-20 | M1.11 Magnetometer update | `5c1231f` | 非线性 `h(x) = R(q)ᵀ·mag_ned + mag_bias`；3×24 的 H 矩阵，转置 Jacobian 通过 q 共轭识等式实现；有限差分 proptest 验证 H 到 1e-4。 |
| [0023](./0023-barometer-update.md) | 2026-04-20 | M1.12 Barometer update | `ab3e2b3` | 1 维高度观测，标量 S / 标量 NIS / 纯数字求逆；Joseph form 退化到列操作；30 步 proptest 256 个随机高度目标收敛到 30cm 内。 |
| [0024](./0024-end-to-end-simulation.md) | 2026-04-20 | M1.13 端到端多源融合 | `9574f99` | 2 秒 1 kHz 预测 + GPS 5Hz + Mag 25Hz + Baro 50Hz 并发；EKF 收敛位置 < 0.3m, 高度 < 0.2m。**EKF 数学栈的最终集成验证**。 |
| [0025](./0025-nis-fdir-integration.md) | 2026-04-21 | M1.14 SensorRejectionCounter | `c362e59` | NIS 拒绝流 → HealthLevel；10 连拒→Degraded、50 连拒→Emergency；`observe` 单步单调性 Kani 证（多步 CBMC 卡 15min → 退到单步+归纳论证）。 |
| [0026](./0026-indi-inner-loop.md) | 2026-04-21 | M2.0 INDI 内环 + LPF | `25a8e3f` | 生产级 INDI：`Δτ = J·(k·(ω_cmd − ω) − ω̇)`；LowPassFilterVec3 首阶 IIR；10 测试 含 2 proptest。**算法栈第二块（控制律）启动**。 |

## 写新文档时遵守的模板

```markdown
# NNNN — 标题

**Commit**: `xxxxxxx`  **Date**: YYYY-MM-DD  **Milestone**: MX.Y

## What / 做了什么
一段话 + 代码/文件清单。

## Why / 为什么这么做
决策背景、取舍、拒绝的方案及理由。

## How it's verified / 怎么验证的
具体命令 + 期望输出。

## Follow-ups / 遗留
未完成但有必要标出的事项。
```

简短优先，不要写成小说。三行能说完的就不要写三段。
