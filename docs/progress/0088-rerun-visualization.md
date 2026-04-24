# 0088 — rerun.io 3D 可视化

**Commit**: `c24f940`
**Date**: 2026-04-24  **Milestone**: 可视化展示

## What / 做了什么

新 `sim-hil/examples/sitl_rerun.rs`：用 [rerun.io](https://rerun.io) 的 native viewer 看 SITL 飞行。启动命令：

```bash
cargo run -p sim-hil --example sitl_rerun --features rerun-viz --release
```

跑 20 秒 figure-8（复用 M17 基准的 2 m 半径、10 s 周期、1 m 高 lemniscate），MPC-I 控制。viewer 里：

**3D 场景**:
- 世界系三轴（red/green/blue = N/E/Down）
- 机体 30×30×6 cm 方块
- 4 motor 点阵在 X-quad 臂尖，颜色按推力 magnitude 插值（绿=idle → 红=saturated）
- body -z 方向的 thrust 箭头
- 青色 setpoint 球
- 飞行轨迹 trail（1 kHz sim，50 Hz 显示抽样，上限 2000 点）

**时序面板**（底部时间轴可拖）:
- altitude_m
- xy_err_m（与 setpoint 的水平距离）
- body rate × 3（roll/pitch/yaw）
- thrust_n 总推力
- motor_0..3 各自推力
- wind_n / wind_e 的 EKF 估计

`rerun-viz` feature gated 掉默认 CI build（rerun 拖 ~300 crates）。

## Why / 为什么选 rerun 不选 Gazebo

user 要 "3D 模拟 / 仿真展示"。选型决定：

### 评估过的方案

| 方案 | 工程量 | 物理精度 | 视觉质量 | Rust 原生 | 结论 |
|---|---|---|---|---|---|
| **rerun.io** | ~2h 集成 | 用我们自己的 | 3D + 时序，粗糙 mesh | ✓ | **选** |
| Gazebo / gz-sim | 1-2 周 | 自带 ODE（和我们冲突）| 真实渲染、有影子 | ✗ (C++) | 拒 |
| Foxglove Studio | ~1 day | 用我们自己的 | 很好 | ✗ (TypeScript bridge) | 备选 |
| Bevy 自研 | 1-2 周 | 用我们自己的 | 完全可定制 | ✓ | 过度工程 |
| matplotlib 后处理 | 1h | 用我们自己的 | 静态 2D 图 | ✗ (Python) | 不是 3D |

### Gazebo 为什么被拒

Gazebo 是**物理 + 渲染**一体机。两种用法：

1. **取代我们的 sim-hil 物理**: 飞控通过 MAVLink 或 gRPC 和 Gazebo 对接，Gazebo 跑刚体动力学 + 给我们回 IMU 读数
   - 坏处: sim-hil 的 4 态 motor-lag + quaternion exp-map 积分 + 精细 drag 模型被扔掉。Gazebo 的 ODE 在 1 kHz 飞控内环时间尺度下精度**不够好**（PX4 用 lockstep timestep scaling 才搞得动）
   - 更坏: workspace 大改，所有 M3-M22 的 SITL 测试 regression 要重做
   - 最坏: Gazebo SDF 世界文件 + 飞机 URDF + 插件 C++ 代码 = 独立的另一个 project

2. **Gazebo 作纯 visualizer**: sim-hil 照常跑，把 pose 通过 Zenoh / MAVLink 推给 Gazebo，写 plugin 驱动 `Visual` model
   - 问题是 Gazebo plugin API 学习曲线 + C++ 编译 friction
   - 就为了 "3D 飞机 mesh 绕圈" 这一个需求付那么多代价不值

**关键判断**: 我们需要的是 **可视化**，不是 **另一个物理引擎**。Gazebo 把两件事 bundle，但我们只要一件。

### rerun 为什么合适

- **Rust native** —— `cargo add rerun` 加一条就完成，workspace 零摩擦
- **专为机器人/ML 设计** —— Transform3D + Point cloud + Arrow + LineStrip + Scalar 时序图都是 primitive
- **时间轴滚动** —— 点底部时间线任意 t，3D + 所有 plot 全部跳到那刻的状态。Gazebo / Foxglove 都要额外配置
- **native viewer 自包含** —— 不需要开额外 web server 或配 ROS bridge。`cargo run` 就弹窗
- **我们要的**: 一个飞机在 3D 世界里绕 8 字，箭头指推力方向，几条 time series 滚动。rerun 全部 out-of-the-box 支持

### 重要取舍: 不做 mesh 纹理

rerun 支持导入 GLTF / OBJ mesh，能渲染真实无人机模型（螺旋桨、碳架、相机等）。我**没用**，理由：
- 加 mesh 加文件 + Cargo.toml 配置 + 可能的 license 问题
- 示范的重点是**控制栈行为**，不是视觉质感。方块 + 点 已经够表达
- 后续有人想要漂亮视觉，加 `rerun::Asset3D::from_file("drone.gltf")` 一行就搞定，留给 followup

## How it's verified / 怎么验证的

```bash
$ cargo build -p sim-hil --example sitl_rerun --features rerun-viz
# expected: compiles (可能要试几次 rerun API 版本；我用的 0.21)

$ cargo run -p sim-hil --example sitl_rerun --features rerun-viz --release
# expected: 弹窗 → 看飞机绕 8 字
```

CI **不跑这个** —— `required-features = ["rerun-viz"]` + `default = []` 意味着 `cargo test --workspace` 不编 rerun 代码。保持 CI 快。

本地跑需要 GUI 环境（X11 / Wayland / WSLg）。WSL2 用户用 WSLg（Windows 11 自带）or 装 VcXsrv。

## Follow-ups / 遗留

- **真实 mesh**: `rerun::Asset3D::from_file()` 加 GLTF 飞机模型。网上有 free quadrotor GLTF，CC-0 的装包里
- **screenshot capture**: rerun 支持录 mp4；跑一次 figure-8 录下来放 README.md 动图。Demo 价值大
- **监听模式**: 现在 example spawn viewer + 跑完 loop forever。改成 spawn sim 作为独立 process + rerun 以 TCP listener 模式起，便于"跑 SITL 同时开 viewer 看"而不是必须一起启动
- **故障注入 UI**: 在 viewer 里加 button "kill motor 0"？rerun 不支持双向（只能看不能交互）。真要交互留 Foxglove 或自写 Bevy
- **reload 功能**: 跑完保存 `.rrd` 文件，下次直接 `rerun file.rrd` 打开。长期 benchmark 数据可以存档对比
- **M13 Zenoh 集成路径**: 也可写一个 `zenoh_to_rerun` bridge，让**任何** Zenoh 订阅者都能拿到 rerun 视角。属于 pillar 4 的自然扩展
