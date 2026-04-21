# 0047 — ARM/DISARM：GCS 真能当"红色开关"

**Commit**: `498ba23`  **Date**: 2026-04-21  **Milestone**: M5.6

## What / 做了什么

整个飞控现在多了一个 **"武装(arm) / 解除武装(disarm)"** 状态。Disarmed 时控制器照常算最优电机推力，但出端被**硬性短路为零**——电机不转。Armed 时恢复正常。

具体：
- `app_copter::ArmState { Disarmed, Armed }` —— `Default = Disarmed`，所以**每次冷启动都必须显式 arm**。
- `app_copter::FlightState::arm_state` 新字段；`rate_loop_step` 末尾 `match flight.arm_state` 决定是否零化 `clipped`。
- `sim_hil::mavlink_udp::arm_change_from_mav_message(&MavMessage) -> Option<bool>` —— 解析 `COMMAND_LONG` + `MAV_CMD_COMPONENT_ARM_DISARM` (command id 400)，`param1 ≥ 0.5 → arm`。
- `examples/sitl_mavlink.rs` 用 `Arc<Mutex<ArmState>>` 把 UDP 主任务和 spawn_blocking sim 任务桥起来（和 M5.6 之前 setpoint 的做法同构）。

**测试**：app-copter 新增 2 条（disarmed 即使剧烈指令也 0 推力 / Armed 恢复），sim-hil 新增 3 条（heartbeat → None / 错指令 → None / arm+disarm round-trip），既有 SITL 闭环测试全部显式加 `arm_state: ArmState::Armed` 才能飞。

workspace：183 单元测试全绿（171 → 183）。

## Why / 为什么这么做

### 为什么 "Disarmed = Default"

所有工业飞控（PX4、ArduPilot、Betaflight、Auterion）的第一条安全守则：**上电不能转**。原因：

- 操作者 arm 时意味着"我看着飞机、我负责"——电气上和心理上都是一道明显的门槛。
- 开发/调试场景里，电机随便转容易**砸手 / 烧 ESC / 炸锂电**。
- 没 arm 的飞机不响应任何控制指令——即使控制器算法写崩了也不会转。

把 Disarmed 设为 `Default` 的深层含义：**任何忘记 arm 的代码路径都安全**。这是 Rust 风格的 fail-safe（类型系统上 "没设置 = 最保守"），比 C 飞控里"默认 enum 0 = DISARMED 但文档靠 review" 鲁棒得多。

### 为什么用 Arc<Mutex<ArmState>>，而不是 atomic

和 M5.6 setpoint 同理：
- 跨线程共享（Arc）
- ArmState 是枚举，不能直接 atomic 化；
- 写少读多（arm 事件稀疏、sim 1 kHz 读）——Mutex < 100 ns，足够。

用 `AtomicU8`（Armed=1 / Disarmed=0）可行但**加一层枚举↔u8 转换**，代码噪音大于性能收益。保持类型一致更干净。

### 为什么把 rate_loop_step 末尾短路，而不是早退

考虑过把 ARM 检查放在函数开头"disarmed 直接返回零"。但：
- 外环（`outer_step`）仍然**计算**每步 → 保持 EKF / setpoint 数据通路热；
- INDI 内部的 LPF 状态仍然**更新** → arm 一瞬间不会因为 α 突变炸电机；
- 虚拟指令、allocation 也正常算——日志里看得到"本应飞成什么样"。

相当于 arm 是一把"在输出阀门上加锁"，阀后面的水流一直在算。这样 arm 后切换**瞬态小**、**调试友好**。

### 为什么 `param1 ≥ 0.5` 而不是 `param1 == 1.0`

MAVLink 规范说 `param1 = 1.0` = arm，`0.0` = disarm。但 QGC / MP / MAVROS 偶尔会发 `NaN` / `0.999999`（f32 舍入 / 用户自定义脚本）——直接等值比较会把合法 arm 命令过滤掉。阈值 `0.5` 居中、鲁棒、工业惯例。

### 为什么不校验 target_system / target_component

当前 SITL 接受所有（参见函数 doc 注释）。生产固件必须校验，因为多机场景会收到发给别人的 `COMMAND_LONG`。M5.7 多机 + FDIR 阶段会加，M5.6 先打通 loop。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib
13 passed (包含 disarmed_vehicle_outputs_zero_thrust_regardless_of_command
           + arming_enables_motor_output)

$ cargo test -p sim-hil
20 passed (包含 arm_change_from_non_command_message_is_none
           + arm_change_from_wrong_command_is_none
           + arm_change_arm_and_disarm_round_trip)

$ cargo test --workspace
所有 crate 绿

$ cargo clippy --workspace --all-targets -- -D warnings
Finished  (含 ArmState derive(Default) + #[default] 惯用风格)
```

**真机联调步骤**（下次开 QGC 时测）：
1. `cargo run -p sim-hil --example sitl_mavlink -- 172.17.0.1:14550`
2. QGC 自动连上 → vehicle 显示在 Zurich 附近
3. 此时点 "Go To Location" **不会飞**（Disarmed，电机零推力，高度会从 -1 m 往下掉）
4. QGC 工具栏 "Arm" 按钮 → console 里看到 `arm state changed by new=Armed`
5. vehicle 开始维持高度；点 Go To → 飞过去
6. "Disarm" → 电机立刻停、飞机坠落（SITL 无护盾）

## Follow-ups / 遗留

- **自动 disarm on landing**：触地检测（z velocity < 0.05 m/s 且 z > -0.3 m，持续 2 s）→ 软件自动 disarm。安全关键：防止飞机在停机坪上意外飞起来。
- **Arm preflight checks**：工业飞控拒绝在以下情况 arm：GPS lock < 3D、baro/mag 健康 ≠ Healthy、battery < 20%、IMU 初始化未完成……目前 SITL 无视所有前提。
- **LAND 命令**：`MAV_CMD_NAV_LAND` → 下降到地面 + 自动 disarm。M5.7 候选。
- **RTL 命令**：`MAV_CMD_NAV_RETURN_TO_LAUNCH` → 返航到起飞点 + 降落。需要 "home" 概念。
- **target_system / target_component 过滤**：多机场景必须。
- **COMMAND_ACK**：GCS 发 COMMAND_LONG 后按规范需要回 `COMMAND_ACK` 告诉它"已接受 / 拒绝 / 完成"。当前不回，QGC UI 上 arm 按钮会转圈几秒才变状态。
- **紧急 disarm**（kill switch）：任何状态下，`param2 = 21196` (magic number) → 立即 disarm 不管飞机在哪。MAVLink 规范支持；我们没实现。
