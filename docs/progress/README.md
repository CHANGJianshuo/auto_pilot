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
| [0027](./0027-control-allocation.md) | 2026-04-21 | M2.1a 控制分配 X-quad | `747b0a3` | 效率矩阵 `E = [−y, +x, k_yaw, 1]`；预计算 `E⁻¹`；`saturate` 夹取；256 样本 round-trip proptest。N=4 直接逆。 |
| [0028](./0028-rate-loop-assembly.md) | 2026-04-21 | M2.3 rate loop 组装 | `5f4e812` | **第一次端到端组合**：IMU → EKF → LPF → INDI → 分配 → 电机。`app-copter` dual-target lib+bin，3 集成测试，demo 跑 100 ms 悬停稳定。 |
| [0029](./0029-attitude-loop.md) | 2026-04-21 | M2.4 姿态环（四元数误差）| `7e4ddec` | `attitude_to_rate(q_current, q_desired, k_att)`；最短旋转判定；退化 q 返回 0；small-angle proptest 256 样本验证 ω ≈ k⊙r。 |
| [0030](./0030-position-loop-baseline.md) | 2026-04-21 | M2.5 位置环基线 | `851a79c` | P-P 级联：`p_sp → v → a → (q_desired, thrust)`；推力向量法避开姿态奇异；256 proptest 所有输入下 q 单位、thrust 有限非负。M4 NMPC 将透明替换。 |
| [0031](./0031-outer-step-stitching.md) | 2026-04-21 | M2.6 outer_step 缝合 | `43f3226` | `app-copter::outer_step(setpoint) → motor_thrusts`：position → attitude → rate → allocation 全栈调用。悬停 1000 ticks + 高度目标 thrust 增长测试。**控制栈整链首次可驱动**。 |
| [0032](./0032-measurement-hooks-fdir.md) | 2026-04-21 | M3.0 GPS/mag/baro + FDIR | `870f146` | `FlightState` 三传感器 `SensorRejectionCounter` + `apply_*_measurement` + `overall_health`；15 连拒 GPS → Degraded；EKF predict+update 闭环 + FDIR 在 app 层打通。 |
| [0033](./0033-sitl-first-hover.md) | 2026-04-21 | 🚁 M3.1a SITL 首次闭环悬停 | `6cb7c96` | 6-DoF 欧拉 rigid-body 仿真器（Newton-Euler 含 `ω×Jω` 项）+ 无噪声传感器；3 秒 1 kHz 闭环悬停测试通过，altitude/水平误差 < 0.5 m。**控制栈首次系统级验证**。 |
| [0034](./0034-realistic-sitl.md) | 2026-04-21 | M3.1b 真实 SITL（噪声+滞后+阻力+风）| `ac59ea2` | Xorshift + Box-Muller SimRng；`NoiseConfig::realistic()`；电机 τ=20 ms 滞后；线性+二次阻力；恒风扰；3 闭环场景（ideal/realistic/wind）全通过。 |
| [0035](./0035-pi-cascade.md) | 2026-04-21 | M3.2 PI cascade（积分项）| `0004831` | velocity-loop 加 I + conditional anti-windup + integrator clamp；realistic SITL 稳态误差 **2m → 0.6m**；向后兼容旧函数。 |
| [0036](./0036-wind-feedforward.md) | 2026-04-21 | M3.3 风扰前馈 | `9eeaaa9` | `outer_step` 用 EKF.wind_ne 做 accel FF；`wind_ff_gain = k_drag/mass`；I 项处理慢变，FF 处理已知风；未来加 EKF 风观测时零改动。 |
| [0037](./0037-drag-aware-predict.md) | 2026-04-21 | M3.4 drag-aware predict | `a39e2ad` | `State::predict_with_drag` + `predict_step_with_drag`；outer loop 的 wind FF (M3.3) 现在有了估计器配对 —— predict 出的速度会反映风拖动。Jacobian 更新（wind observability）queue 到 M4。 |
| [0038](./0038-app-drag-wiring.md) | 2026-04-21 | M3.5 app 接 drag predict（opt-in）| `51708f9` | `RateLoopConfig::drag_over_mass_hz` 默认 0；`rate_loop_step` 调 `predict_step_with_drag`；机械件就位，M4 加 Jacobian 后再默认开启。 |
| [0039](./0039-wind-observable-jacobian.md) | 2026-04-21 | M4.0 风可观测 F Jacobian | `b875322` | `build_transition_jacobian_with_drag`：∂v/∂v + ∂v/∂wind_ne + ∂p/∂v + ∂p/∂wind_ne 修正/新块；2 个有限差分 proptest；EKF 现在**能从测量学 wind**。 |
| [0040](./0040-wind-identification-sitl.md) | 2026-04-21 | M4.1 端到端 wind 识别 SITL | `52421fe` | 开环 SITL 注入 (+2, -1) 风，15 s 后 EKF wind_ne 方向正确（cos > 0.5）；闭环下 controller FF 干扰识别 → 开环隔离；默认仍 opt-in 避免回归 PI。 |
| [0041](./0041-gps-fault-injection.md) | 2026-04-21 | M5.0 GPS 故障注入 + FDIR 端到端 | `c53099e` | `GpsFault::{None, Offset, Stuck}` 故障模式；SITL 注入 500m 偏移 4 s → gps_health ≥ Degraded；vehicle 靠 baro+mag 存活。**FDIR 全链路首次系统级验证**。 |
| [0042](./0042-mavlink-basic-encoding.md) | 2026-04-21 | M5.1 MAVLink 基础编码 | `fbcbbe5` | `mavlink` v0.14 接入 workspace；`encode_heartbeat / encode_attitude / encode_global_position_int`；no_std；clamp_f32_to_i* 防 panic；5 测试。 |
| [0043](./0043-mavlink-udp-transport.md) | 2026-04-21 | M5.2 MAVLink UDP 传输 | `4acaba7` | `MavlinkUdpSink` tokio `UdpSocket` 封装；AtomicU8 序号；2 集成测试（bind+send 和 seq 递增）；QGC 可直接监听 14550。 |
| [0044](./0044-sitl-mavlink-demo.md) | 2026-04-21 | M5.3 `sitl_mavlink` demo | `849c9f3` | `cargo run -p sim-hil --example sitl_mavlink` 一键启动实时 SITL + MAVLink telemetry；spawn_blocking sim + tokio fan-out + wall-clock pacing；HEARTBEAT/ATTITUDE/GLOBAL_POSITION_INT 分速率。 |
| [0045](./0045-mavlink-parse.md) | 2026-04-21 | M5.4 MAVLink 解析器 | `fcb2b09` | `parse_frame(&[u8]) -> (MavHeader, MavMessage)`；&[u8] 直接实现 embedded_io::Read 保持 no_std；4 round-trip/边界/CRC-corruption 测试。 |
| [0046](./0046-mavlink-bidirectional.md) | 2026-04-21 | M5.5 MAVLink 双向 + setpoint | `7570dcb` | `try_recv()` + `setpoint_from_mav_message`；demo `Arc<Mutex<Setpoint>>` 共享，QGC "Go To Location" 能真驱动 SITL 飞过去；4 新测试。 |
| [0047](./0047-arm-disarm.md) | 2026-04-21 | M5.6 ARM/DISARM via MAVLink | `498ba23` | `ArmState { Disarmed(Default), Armed }` + `rate_loop_step` 末尾硬性短路；`arm_change_from_mav_message` 解 COMMAND_LONG(400)；demo `Arc<Mutex<ArmState>>` 共享；5 新测试，183 单元测试全绿。 |
| [0048](./0048-land-autodisarm.md) | 2026-04-21 | M5.7 LAND + 触地 auto-disarm | `4191611` | `LandingState { Idle(Default), Landing }` + `TouchdownDetector`（三条件 AND + 1s 持续）；Landing 态 outer_step 覆盖 setpoint 原地下降 0.5 m/s 并在触地时自动 disarm；`land_request_from_mav_message` 解 COMMAND_LONG(21)；demo 双向同步；5 新测试，188 单元测试全绿。 |
| [0049](./0049-command-ack.md) | 2026-04-21 | M5.8 COMMAND_ACK | `a7e24c6` | `encode_command_ack` + `send_command_ack`；demo 对每个 COMMAND_LONG 回 ACCEPTED/UNSUPPORTED，消除 QGC 按钮 spinner；round-trip 测试验证 encode/parse 双向兼容。 |
| [0050](./0050-fmt-cleanup.md) | 2026-04-22 | M6.0 Workspace fmt 统一 | `9d986af` | `cargo fmt` 一把梭，11 files / 472 lines insertion，0 行为改动；`cargo fmt --check` 现在全绿，下一步进 CI。 |
| [0051](./0051-ci-workflow.md) | 2026-04-22 | M6.1 CI workflow pinned | `47b1283` | `.github/workflows/ci.yml` fmt/clippy/test/build-fw/geiger 五 job，toolchain 硬 pin 1.88.0 和 `rust-toolchain.toml` 对齐；防 "本地绿 CI 红" 漂移。 |
| [0052](./0052-takeoff-home.md) | 2026-04-22 | M6.2 TAKEOFF + home | `d2d1469` | `TakeoffState { Idle, TakingOff { target_z_ned } }` + `AltitudeReachedDetector` + 首次 arm 记 home；`outer_step` 爬升到 target（1 m/s）再回 Idle；`takeoff_request_from_mav_message` 解 COMMAND_LONG(22)；demo auto-arm；4 新测试。 |
| [0053](./0053-rtl.md) | 2026-04-22 | M6.3 RTL（返航） | `8a06910` | `RtlPhase { Idle, Climbing, Returning }` 3 阶段：爬到 `rtl_safe_alt_m`（默认 10m）→ 飞向 home xy → hand-off 给 LandingState::Landing 自动降落+disarm；`rtl_request_from_mav_message` 解 COMMAND_LONG(20)；4 新测试，起降完整闭合。 |
| [0054](./0054-preflight.md) | 2026-04-22 | M6.4 Preflight check | `358592b` | `preflight_check(&FlightState) -> Result<(), PreflightReject>` 查 GPS/baro/mag 健康 + EKF 协方差；demo 的 ARM/TAKEOFF 被 gate，失败发 `MAV_RESULT_TEMPORARILY_REJECTED`；DISARM 永不 gate；4 新测试。 |
| [0055](./0055-thumbv7em-build.md) | 2026-04-22 | M7.0 app-copter thumbv7em build | `7a708f8` | `#![cfg_attr(not(test), no_std)]` + 两处 `libm::sqrtf`；workspace 11 个可嵌入 crate 全部编译到 thumbv7em-none-eabihf；CI build-firmware job 从占位 echo 换成真实 per-crate 编译循环。 |
| [0056](./0056-firmware-link.md) | 2026-04-22 | M7.1 firmware ELF 链出 | `336ca0a` | `crates/app-copter/memory.x`（STM32H753/Pixhawk 6X）+ `build.rs` + `src/bin/firmware.rs`（`#![no_main]`, cortex-m-rt entry, cfg-gated host stub）→ **132 KiB ELF**；CI 现在真链 firmware binary。 |
| [0057](./0057-embassy-runtime.md) | 2026-04-22 | M7.2 embassy async runtime | `1064645` | `#[embassy_executor::main]` 替代 cortex-m-rt entry；TIM2 time driver；rate_loop @ 1kHz + heartbeat @ 1Hz 两个 task；`critical-section-single-core` 特性解 link 错误；**298 KiB release ELF**。 |
| [0058](./0058-defmt-probe-rs.md) | 2026-04-22 | M7.3 defmt-rtt + probe-rs runner | `498f320` | `.cargo/config.toml` 加 `runner = probe-rs run --chip STM32H753ZITx` + `-Tdefmt.x`；firmware 用 defmt_rtt/panic_probe；heartbeat_task 每秒 `defmt::info!` 出 RTT；修 `+fp-armv8d16sp` 错指令（M7 用 VFPv5 非 Armv8-M），硬件 FPU 恢复后 release ELF **298 KiB → 91 KiB**。 |
| [0059](./0059-icm42688-driver.md) | 2026-04-22 | M7.4 ICM-42688 IMU 驱动 | `f371158` | `core-hal::imu::icm42688` 纯 `embedded-hal::SpiDevice` 泛型驱动：`new` 查 WHO_AM_I、`configure` 设 1kHz/±16g/±2000dps、`read_sample` 14-byte burst + SI 标度；6 新测试用自制 MockSpi，workspace 189 全绿；thumbv7em 编译过。 |
| [0060](./0060-lqr-position.md) | 2026-04-22 | M9.0 LQR 位置环 | `1738c0a` | `compute_lqr_gains` 迭代 DARE 求解 2×2 双积分器最优反馈 + `lqr_position_gains` helper 包三轴；**plan.md 四大创新点之一 NMPC 的起点**（无约束 + 无限 horizon）；SITL 闭环 3s 内悬停误差 < 25 cm；workspace 196 tests 全绿。 |
| [0061](./0061-mpc-constrained.md) | 2026-04-22 | M9.1 带约束的有限 horizon MPC | `43d2af6` | `Mpc1d<const H>` — 单轴 receding-horizon MPC：展开动力学到 dense QP、LQR Riccati P 作终端代价保稳定、projected gradient + Gershgorin 步长 + warm start；5 新测试验证 unconstrained = LQR / 紧约束被遵守 / 紧 box 下闭环收敛；workspace 201 tests 全绿。 |
| [0062](./0062-mpc-sitl.md) | 2026-04-22 | M9.2 三轴 MPC + SITL | `1b29518` | `Mpc3dPositionController<const H>` 把 xy+z 两个 Mpc1d 打包、.step 返回 `(q_desired, thrust)`；与 PI 路径共享 `accel_to_attitude_thrust` last-mile；**SITL 闭环首飞**：ideal sim 悬停误差 <25cm、u_max=2m/s² 紧约束下 8 秒内到达 1m 设定点 < 50cm；206 tests 全绿。 |
| [0063](./0063-lqi.md) | 2026-04-22 | M9.3 LQI：integrator 清稳态偏差 | `0e7d1c7` | `LqiWeights { q_pos, q_vel, q_i, r }` + 3×3 DARE → `LqiAxisGains`；`Lqi3dPositionController` 带 3 积分器 + anti-windup；核心 sanity `q_i=0 ⇔ LQR`；**realistic sim + 2m/s 风 + drag**，15 秒 altitude<0.3m、horiz<1m，LQR 在同条件有持续偏差；213 tests 全绿。 |
| [0064](./0064-controller-shootout.md) | 2026-04-22 | M9.4 `PositionController` enum + shootout | `4585489` | PI/LQR/MPC/LQI 一个 enum `PositionController<H>` 统一 `.step()` / `.reset()` / `.kind()`；**shootout SITL**: ideal sim 4 路 < 25cm 悬停，realistic sim 积分器类 < 0.6m、LQR 松界 < 2m（regime 分层记录）；218 tests 全绿。 |
| [0065](./0065-mpc-i.md) | 2026-04-23 | M9.5 MPC-I：约束 + 积分器合体 | `b150a06` | 把 M9.1 MPC（box 约束）和 M9.3 LQI（integrator）augmented 成 3-state：`z = [e_p, e_v, i]`、`dare_3x3` 终端代价、`Mpc1dI<H>` + `MpcI3dPositionController<H>` + `PositionController::MpcI`；核心 sanity `q_i=0 ⇒ MPC-I ≡ M9.1 Mpc1d`；realistic shootout 3-way（PI+I / LQI / MPC-I）全 < 0.6m；**NMPC 创新点收尾**；225 tests 全绿。 |
| [0066](./0066-zenoh-bus.md) | 2026-04-23 | M10 Zenoh 原生中间件（a+b） | `ee3d94c` + `c943142` | **plan.md 创新点 4 第一次落地**。M10a: core-bus 7 个 typed message（IMU/Attitude/Vel/Pos/Setpoint/ActuatorCmd/Health）+ postcard codec + `no_std` round-trip tests。M10b: `sim-hil::zenoh_bus` 两个 peer session 端到端 pub/sub 通过 Zenoh，feature-gated (`zenoh-host`) 不污染默认 build；233 tests 全绿，CI 新 `test-zenoh` job。 |
| [0067](./0067-nn-runtime.md) | 2026-04-23 | M11a nn-runtime residual policy | `44b6ce5` | **plan.md 创新点 3 第一次落地**。`FeatureVector` [f32;9] + `Residual(Vector3)`、`InferenceBackend` trait + 参考 `AffineBackend`（pure Rust，no tract dep 避 dev-version 锁定）、`SafetyEnvelope` + typed `EnvelopeReject`、`ResidualPolicy<B>` 统一 policy harness + reject count；11 新测试；workspace 244 全绿；plan.md 四大创新点**全部开始**。 |
| [0068](./0068-residual-mpc-sitl.md) | 2026-04-23 | M11c MPC + residual 接进 SITL | `88c13bd` | `Mpc3dPositionController` 拆出 `solve_accel` + `accel_to_attitude_thrust` pub；`sim-hil::residual_mpc::MpcResidualController<H, B>` 把 MPC 和 ResidualPolicy 绑一起、envelope reject 静默 fallback；**realistic sim（1.5 m/s 风 + drag）下手调 affine residual 比裸 MPC 横向误差降 > 20 cm**；247 tests 全绿。 |
| [0069](./0069-three-tier-shootout.md) | 2026-04-23 | M11d 三档 shootout | `0f680c8` | 同 realistic sim 三档对比：bare MPC（漂 > 0.5 m）/ MPC+residual (0.22 m) / LQI (0.24 m)；意外发现手调 PD 残差和 MPC 预测合力追平 LQI；断言**档位**而非 ordering，允许 tuning 摇摆；`residual × 3 < bare`、`max/min < 3`；248 tests 全绿。 |
| [0070](./0070-kani-core-bus.md) | 2026-04-23 | M12 Kani 扩到 core-bus + CI | `c32b90e` | 5 个新 Kani 证明（HealthLevel u8 序保持 + round-trip / SensorFaultBit 位互斥 + union 保留 / ACTUATOR_MAX_CHANNELS 限 u8）；workspace 证明 13→18；CI 新 `kani` job 4 crate 并行证，总耗时 < 15 秒；plan.md 创新点 1 的"形式化"从声明落到 CI 合同。 |
| [0071](./0071-full-telemetry-zenoh.md) | 2026-04-23 | M13 全遥测 Zenoh 端到端 | `cc9965c` | `TelemetryPublisher` 把 core-bus 7 条 topic 打包成 typed `.publish_*()`；helper `healthy_msg` + `with_fault`；SITL 测试 2 peer session、7 并行 subscriber、byte-exact round-trip；M10 Zenoh 从"1 条消息"升级到"整套 schema"。 |
| [0072](./0072-sitl-zenoh-runner.md) | 2026-04-23 | M14 SITL runner 广播 Zenoh | `648b68a` | `run_closed_loop_with_zenoh_telemetry` 把 MPC 闭环 + TelemetryPublisher 合体：每 tick 按 docs/topics.md 速率（IMU 1 kHz / att·vel·pos 250 Hz / setpoint 50 Hz / health 10 Hz）广播；测试 counting subscriber 断言 ≥ 90% 速率 + 飞机保持 < 30 cm 悬停；plan.md 创新点 4 从"能 pub/sub"升级到"闭环里真的实时流"。 |
| [0073](./0073-shootout-executable.md) | 2026-04-23 | M15 residual+Zenoh / shootout 可执行 | `7deab80` + `206b9bb` | **M15a**：新 runner 让 MpcResidual + TelemetryPublisher 同时跑，policy reject 转 HealthMsg::EKF flag；SITL 风扰 < 1 m。**M15b**：`cargo run -p sim-hil --example controller_shootout --release` 输出六路 markdown 表（PI 0.015 / residual 0.221 / MPC-I 0.226 / LQI 0.239 / MPC 2.74 / LQR 5.18 m），`sim_hil::sitl` 新 public 模块、app-copter 升主 dep。|
| [0074](./0074-rtl-kani.md) | 2026-04-23 | M16a RtlPhase pure fn + Kani | `493a643` | RTL transition 从 outer_step 糅合代码重构成 pure `RtlPhase::advance → RtlTransition`，用 `dx²+dy² < tol²` 替代 `sqrt`；4 新 Kani 证（Idle 吸收、no-home 取消、Climbing 不跳 Handoff、Returning 不回退）；CI Kani job 加 app-copter；workspace 证明数 18→22。 |
| [0075](./0075-landing-takeoff-kani.md) | 2026-04-23 | M16b+c Landing + Takeoff pure fn | `a8be3e6` + `91200fb` | 对称完成：`LandingState::advance → LandingTransition {Stay, Complete}`、`TakeoffState::advance → TakeoffTransition {Stay, Reached}`；TouchdownDetector 的 sqrt 换 squared compare；6 新 Kani 证（Idle 吸收 / 只能从指定出口退出 / Idle 不碰 detector）× 2；workspace 证明数 22→28；app-copter 里 3 条飞行 mode 状态机全部形式化。|

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
