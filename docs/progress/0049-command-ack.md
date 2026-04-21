# 0049 — COMMAND_ACK：让 QGC 的按钮不再转圈

**Commit**: `a7e24c6`  **Date**: 2026-04-21  **Milestone**: M5.8

## What / 做了什么

MAVLink 规范规定：收到任何 `COMMAND_LONG` 都必须回一个 `COMMAND_ACK`（echoed command id + `MavResult`）。之前 M5.6/5.7 把 arm/land 的**语义**做通了，但每次 GCS 发命令后 QGC UI 的按钮会**转 5 秒** 才变状态——因为它在等我们没发的 ACK。

这一步补上：

- `comms_mavlink::encode_command_ack(system_id, component_id, sequence, command, result) -> FrameBuffer`
- `sim_hil::mavlink_udp::MavlinkUdpSink::send_command_ack(command, result) -> io::Result<()>`
- `sitl_mavlink` demo 收到 `COMMAND_LONG` 后：
  - `ARM_DISARM` / `NAV_LAND` → `MAV_RESULT_ACCEPTED`
  - 任何其他 command id → `MAV_RESULT_UNSUPPORTED`

加 1 测试（sink → UDP → receiver 里 parse 回来验证 command/result 匹配）。workspace 22 sim-hil 测试、9 comms-mavlink 测试全绿。

## Why / 为什么这么做

### 为什么每个 COMMAND_LONG 都必须 ACK

MAVLink 是 UDP-based，**没有传输层重传**。`COMMAND_LONG` 规范要求双向确认：
1. 发送方（GCS）发 `COMMAND_LONG`，开个 timeout（QGC 默认 3-5 s）；
2. 接收方（vehicle）处理 → 回 `COMMAND_ACK` 带 echoed command id；
3. 发送方确认 = 关闭 UI spinner。如果 timeout 还没 ACK，重发 + 告警。

不 ACK 的后果：
- **UI 卡顿**：操作员以为自己点错了，反复按；
- **重发风暴**：GCS 会发 `confirmation=1, 2, 3...` 的重试版本，占用带宽；
- **高级任务系统（mission planner）炸**：任务里每个步骤可能是一个 `COMMAND_LONG`，没 ACK 就卡住整个任务流。

### 为什么 Unsupported 命令也要 ACK

直觉："我不懂这个命令就不回"——错。规范要求**任何** COMMAND_LONG 都要 ACK，未知的回 `UNSUPPORTED`（或 `DENIED` / `FAILED`）。这样 GCS 知道"vehicle 听到了但做不到"，和"vehicle 完全没听到"是不同状态——前者不需要重发，后者需要。

### 为什么不在 comms-mavlink 里做命令分发

考虑过把 "收到 COMMAND_LONG 应该 ACK" 这条规约打包到 `comms-mavlink` 里，提供一个 "dispatch" API 类似 `on_command_long(|cmd| -> result)`。但：
- comms-mavlink 是 **`no_std`**，tokio 的 `async send_command_ack` 不能直接放进去（tokio 需要 std）；
- 不同 transport（UART / USB / CAN bridge）有不同的发送 API；
- 强制 dispatch 模式会侵入 application 的事件循环。

所以保持 comms-mavlink 是纯 encoder，具体何时 ACK 由 application 决定。sim-hil demo 给了一个参考实现，实机应用抄一样的 while-let-match 就行。

### 为什么 ACK 用的 sequence 独立递增

每个 MAVLink 帧都带 sequence byte，接收方能检测丢包。ACK 是一个**新的独立帧**，不是 "response 帧"——它和 telemetry（heartbeat/attitude）共用同一套 sequence counter（`MavlinkUdpSink::sequence` AtomicU8）。规范允许这么做，因为 ACK 的 "回应 谁" 靠 `command` 字段判断，不靠 sequence。

### 为什么 `MavResult::MAV_RESULT_UNSUPPORTED` 而不是 `FAILED`

- `UNSUPPORTED`：vehicle 软件层不认这个命令——"这个固件版本没实现"；
- `DENIED`：vehicle 认得，但当前状态下拒绝（例如 preflight check 失败）；
- `FAILED`：认得、允许，但执行时出错（例如要 disarm 但电机锁死）；
- `TEMPORARILY_REJECTED`：稍后重试就能过（比如当前 GPS 没 lock）；

现阶段 demo 对未知命令都是 "没实现"，用 `UNSUPPORTED` 最准确。将来加 preflight 检查时，arm 在 GPS 无 lock 时应当返 `TEMPORARILY_REJECTED`（QGC 会告诉用户为啥）。

### 测试用 round-trip 不是 encode-only

encode-only 测试只能验证字节数 ≥ 15（帧结构）和前几位魔数。但 `COMMAND_ACK` 的 `MavResult` 字段是一个 enum，序列化时它的 u8 repr 必须和 decoder 协商。最可靠的是 encode → 真的发 UDP → recv → parse_frame → 匹配枚举。这复用了 M5.4 的 parser，顺带证 encode/parse 路径对这条消息双向兼容。

## How it's verified / 怎么验证的

```bash
$ cargo test -p comms-mavlink
9 passed

$ cargo test -p sim-hil
22 passed, 包含 command_ack_round_trips_to_receiver

$ cargo test --workspace
所有 crate 绿

$ cargo clippy --workspace --all-targets -- -D warnings
Finished

$ cargo build -p sim-hil --example sitl_mavlink
Finished
```

**QGC 实测预期**：
1. demo 启动，QGC 连上；
2. 点 "Arm" → 按钮**立刻**从"转圈"变成"绿色 Armed"（之前要 3-5 s）；
3. 点 "Land" → vehicle 开始降落 + 按钮立刻切 "Landing"；
4. QGC log 面板里看到 "Command accepted"。

（还没真机测过 QGC UI 时序，等有时间开桌面 host 上跑。代码层面 round-trip 已验证。）

## Follow-ups / 遗留

- **Preflight checks before Arm**：arm 时检查 GPS lock / IMU init / battery，失败返 `TEMPORARILY_REJECTED`，QGC 会在警告区显示具体 reason（need to extend ACK with `progress`/`result_param2`）。
- **Progress ACK for long-running**：TAKEOFF / LAND 完成前应每几秒发 `MAV_RESULT_IN_PROGRESS` + `progress: 0-100`。QGC 的进度条依赖它。
- **Ack confirmation==N messages**：规范要求：如果看到 `confirmation = 0` 之后又看到 `confirmation = 1` 的同一条 `COMMAND_LONG`，只执行一次但回两次 ACK。现在每次都执行——多次 arm 有幂等性问题（电机瞬间抖）。
- **target_system / target_component 过滤**：多机场景必须，今天还是"收到就响应"。
- **COMMAND_INT**：MAVLink v2 鼓励用 `COMMAND_INT`（带 lat/lon）代替 `COMMAND_LONG`。我们现在只处理 LONG。
- **M6.0 fmt cleanup**：stashed 的 rustfmt churn 还没合。下一步应该先把它 pop + 提交，统一 workspace 风格。
