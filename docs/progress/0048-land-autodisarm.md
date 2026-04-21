# 0048 — LAND 命令 + 触地自动 disarm

**Commit**: `4191611`  **Date**: 2026-04-21  **Milestone**: M5.7

## What / 做了什么

完成"起降生命周期"闭环的最后一块：**自动降落**。之前 M5.6 只解决了"开机→飞"，现在 M5.7 解决"飞完→落→关电机"。

- `app_copter::LandingState { Idle, Landing }`：状态机，Default = Idle。
- `app_copter::TouchdownDetector`：
  - 触发条件（同时满足 1 秒）：
    - `|v_z| < 0.15 m/s` （停止下降）
    - `‖v_xy‖ < 0.5 m/s` （停止平移）
    - `p_z_ned > -0.30 m` （离地面近）
  - 任意一个破坏 → 计时器清零重来。
- `outer_step` 新行为（Landing 态）：
  1. 用 EKF 位置估计生成"在原地下降"的 setpoint 覆盖调用者传入值；
  2. 每步 `touchdown_detector.observe(...)`；
  3. 触地确认 → `arm_state = Disarmed` + `landing_state = Idle` + detector 复位。
- `RateLoopConfig::landing_descent_mps`：默认 0.5 m/s（保守，给 EKF 时间识别触地）。
- `sim_hil::mavlink_udp::land_request_from_mav_message(&MavMessage) -> bool`：`COMMAND_LONG` + `MAV_CMD_NAV_LAND` (id 21)。
- `sitl_mavlink` demo：`Arc<Mutex<LandingState>>` 双向同步——GCS 发 LAND → flight；flight 自动清除 → 回推 shared；同样机制套用到 auto-disarm。

**测试**：4 新 app-copter + 1 新 sim-hil，总 188 单元测试（17 app-copter + 21 sim-hil + 其他 150）全绿。

## Why / 为什么这么做

### 为什么自动 disarm 要门槛严格

飞机"看起来停了"≠ 真的触地了：
- GPS 抖动 / baro 漂移 → 速度估计瞬间可能 < 0.15 m/s 即使飞机在飞；
- 悬停在 -0.1 m 上方（EKF 原点未校准）→ 不应 disarm；
- 降落瞬间反弹（地面反作用力让 v_z 瞬间 > 0）→ 必须 observed 持续 1 s。

**三个条件 AND + 1 秒持续** 是保守但鲁棒的默认。PX4 用类似条件（`MPC_LAND_SPEED` + `LNDMC_Z_VEL_MAX`）。

### 为什么在 outer_step 里做，而不是做成独立函数

考虑过把 landing 管理做成独立模块（`fn apply_landing(flight) -> Setpoint`, `fn check_touchdown(flight)` 分开）。但：
- 触地后要改 `flight.arm_state`，这是 outer_step 已有的状态通路；
- landing 用的速度/位置估计就在 `flight.state` 里，外部函数得重新拿一遍；
- **用户**只调 outer_step 一次，不希望记得另外调 `check_landing`。

内聚到一个函数反而简洁、不易漏。代价是 outer_step 多 15 行；值。

### 为什么 setpoint 覆盖要同时改 position + velocity

只改位置：`position.z += descent_rate * dt` → 位置环会算出"需要 -0.1 m/s velocity"但不是确切 `descent_rate`——环路跟踪误差导致下降慢或震荡。

同时给 velocity_ned.z = descent_rate → **velocity feed-forward**，让 PI 速度环直接有个好起点，位置环只需纠小误差。收尾平滑，不会在触地前加速度冲击。

### 为什么 xy 位置从 `flight.state.position_ned` 抓，不是 setpoint

GCS 发 `MAV_CMD_NAV_LAND` 的 xy 参数（lat/lon）通常是 NaN 或 0：指令本意是 "就地降落"。如果用指令里的 xy，飞机会边降边水平飞走——反直觉且危险。从 EKF 估计抓当前位置最安全。

### 为什么 landing_state 双向同步

- **GCS → sim**：用户点"Land"按钮，demo 要立即反应；
- **sim → GCS**：触地自动 disarm 后，若不把状态回推到共享变量，下次 GCS 再点 Land 时会发现"还是 Idle"好像 LAND 没生效——其实已经生效并完成。

工业飞控回 `COMMAND_ACK` + 心跳里的 `system_status` 字段通知 GCS。M5.8 会做 ACK。

### 为什么 Idle 是 Default 而 Landing 不能 arm

考虑过让飞机每次 arm 就自动进入 Landing 的**相反**态（Takeoff mode）。但：
- 手动/acro 飞行（未来 M6）不需要自动 takeoff；
- 多旋翼 takeoff 比 land 简单得多（给个 z=-2 setpoint 即可），不值得单独状态；
- Landing 是"有明确终点"的模式；Takeoff 不是。

所以只做 Landing，不做对称的 Takeoff。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib
17 passed, 包含:
 * touchdown_detector_fires_after_settle_time
 * touchdown_detector_resets_on_disturbance
 * touchdown_detector_does_not_fire_while_airborne
 * landing_mode_overrides_setpoint_and_auto_disarms

$ cargo test -p sim-hil
21 passed, 包含 land_request_matches_only_nav_land_command_long

$ cargo test --workspace
所有 crate 绿

$ cargo clippy --workspace --all-targets -- -D warnings
Finished  (修掉一个 doc_overindented_list_items)

$ cargo build -p sim-hil --example sitl_mavlink
Finished
```

**预期真机联调流程**（M5.8 出 COMMAND_ACK 后可测）：
1. 启 SITL demo + QGC；
2. QGC Arm → motor 转；飞机悬停在 -1 m；
3. 点 "Land" → console 看到 `LAND command received — starting autoland`；
4. 飞机以 0.5 m/s 下降，约 2 s 到地面；
5. 触地 → 1 s 后 console 看到 arm state change (→ Disarmed)；
6. 飞机躺在地上，motor 零转，不会被"go to location" 再飞起来（需要重新 arm）。

## Follow-ups / 遗留

- **M5.8 COMMAND_ACK**：GCS 发 LAND / ARM 后要回 ACK，否则 UI 转圈几秒才变状态。
- **LAND 精度**：现在是"原地降落"，忽略 GCS 传的 lat/lon。生产固件要支持按目标点降落（降落伞模式需要把 xy 也带进 landing setpoint）。
- **Landing wind compensation**：大风天飞机会在降落过程被吹偏，应该在 landing setpoint 里考虑 wind_ff。目前关了因为降落速度低 → 风的影响相对大，但短时间问题不大。
- **紧急 stop**: 触地检测不是 kill-switch。真正的硬 disarm（MAV_CMD_COMPONENT_ARM_DISARM + param2=21196）需要独立路径，M5.6 没做，M5.8/9 会加。
- **RTL (Return-To-Launch)**：`MAV_CMD_NAV_RETURN_TO_LAUNCH` —— 先回起飞点，再降落。需要"home"概念。M5.9 候选。
- **TAKEOFF 命令**：对称的 `MAV_CMD_NAV_TAKEOFF`。目前通过 arm + setpoint 隐式做。显式 takeoff 对任务计划更清楚。
- **Preflight check on arm**：GPS lock、baro 健康、battery 阈值，arm 前拒绝。
- **Kani 证 LandingState 状态转移单调**：Idle → Landing 可以，Landing → Idle 只在触地后；防止误配置把 Idle → Idle bypass。
