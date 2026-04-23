# 0066 — Zenoh 原生中间件：M10a 消息 + M10b host session

**Commits**: `ee3d94c`（M10a）、`c943142`（M10b）
**Date**: 2026-04-23  **Milestone**: M10

## What / 做了什么

plan.md 四大创新点第 4 条"**ROS 2 / Zenoh 原生中间件**"首次从类型占位变成可用代码。

### M10a — core-bus 消息 schema

`core-bus` 从"只有 topic 常量"扩成一整套**typed message + codec**：

```rust
use core_bus::{encode, decode, ImuMsg, SetpointPositionMsg, topics};

let msg = SetpointPositionMsg {
    timestamp_us: 42,
    position_ned_m: [0.0, 0.0, -1.0],
    velocity_ned_m_s: [0.0, 0.0, 0.0],
    accel_ned_m_s2: [0.0, 0.0, 0.0],
    yaw_rad: 0.785,
};
let wire: Payload = encode(&msg)?;       // heapless::Vec<u8, 256>
let back: SetpointPositionMsg = decode(&wire)?;  // 同一 schema 解码
assert_eq!(msg, back);
```

7 条消息：
- `ImuMsg` — IMU 原始
- `AttitudeMsg` / `VelocityNedMsg` / `PositionNedMsg` — EKF 估计（带 diag 协方差）
- `SetpointPositionMsg` — 位置 setpoint（pos + vel + accel + yaw）
- `ActuatorCmdMsg` — 8 通道电机推力 + n 计数
- `HealthMsg` — `HealthLevel` + 每传感器 + `fault_flags: u32`（`SensorFaultBit::GPS` 等位常量）

所有消息：`no_std` + `Copy` + serde derives + postcard 编码 → `Vec<u8, 256>` 有界有栈。**8 新测试** 覆盖每条消息 round-trip + 坏 payload 解码错误 + 最大 encoded size 不超界 + topic 名符合 ROS 2 命名规则。

### M10b — sim-hil 上接 Zenoh 真 session

```rust
let pub_session = ZenohSession::open_peer().await?;
let sub_session = ZenohSession::open_peer().await?;
let publisher = pub_session.publisher::<SetpointPositionMsg>(topics::SETPOINT_POSITION).await?;
let mut subscriber = sub_session.subscriber::<SetpointPositionMsg>(topics::SETPOINT_POSITION).await?;
publisher.put(&msg).await?;
let received: SetpointPositionMsg = subscriber.recv().await?;
```

- `ZenohSession`：Arc<Session> 的 thin wrapper，`open_peer()` 零配置 loopback/LAN
- `Publisher<M: Serialize>` / `Subscriber<M: DeserializeOwned>` 泛型 trait bound
- **通过 `zenoh-host` feature 门控**（zenoh 1.9 拉 ~80 crate，不污染默认 build）
- 端到端 SITL test：两个 peer session，publisher put → subscriber recv，byte-exact 回收

CI 新 job **`test-zenoh`**：专门跑 `--features zenoh-host` 的测试，默认 matrix 不被拖慢。

Workspace **233 tests** 全绿（core-bus +8），zenoh-host feature 下 +1 round-trip，thumbv7em 编译过，clippy + fmt 全绿。

## Why / 为什么这么做

### 为什么 M10 要分 a + b

两步分开：
- **M10a**：message 定义 + codec —— 纯 `no_std`，编进 firmware，和 Zenoh 无关
- **M10b**：host 侧 Zenoh session wrapper —— std-only，靠 M10a 的 codec

分开的好处：
- 各自 commit 可独立 review（a 是 schema 设计，b 是 transport plumbing）
- 假设 Zenoh 将来换成 DDS / NATS / MQTT，只需重写 M10b，M10a 不变
- firmware 端先有 M10a 可用（给 UART / CAN 用），M10b 专门面向 host

### 为什么用 postcard 而不是 CBOR / protobuf / flatbuffers

比较：
| 格式 | size | no_std | zero-copy | schema evolution |
|---|---|---|---|---|
| protobuf | 中 | 需 alloc | 不 | ✅ tag-based |
| CBOR | 中 | 需 alloc 大概率 | 不 | ✅ self-describing |
| flatbuffers | 大 | 复杂 | ✅ | ✅ |
| **postcard** | **小** | ✅ native | 不 | 🟡 struct 变化要两端一起更新 |
| JSON | 极大 | 不 | 不 | ✅ |

postcard 专为 `no_std` Rust serde 场景设计：`heapless::Vec` 输入 → 紧凑 varint 编码，比 JSON 小 5-10×，比 protobuf 小 ~30%。**tagged schema evolution 是缺失的特性** —— 我们接受它，因为 firmware 和 host 从同一 repo 编译，schema 总是匹配。

将来需要跨版本兼容（比如 firmware 升级了但 ground station 还是旧版），可以在消息里加 `version: u8` 字段手动 dispatch，或者切到 CBOR。现在过度设计会浪费。

### 为什么 message 是 plain data（不用 nalgebra）

`AttitudeMsg::quaternion: [f32; 4]` 而不是 `nalgebra::Quaternion<f32>`。原因：

- **schema 稳定**：如果 nalgebra 升级改了 layout，serde 输出字节变；我们的 wire format 不受这影响
- **dep-free consumer**：Python / C++ ROS 2 订阅者不需要懂 nalgebra
- **zero-copy 便利**：`[f32; 4]` 布局清晰，直接 `memcpy`

代价：算法层用 nalgebra 时要做一次 `msg.quaternion → Quaternion::new(w, x, y, z)` 转换。轻，一行代码。

### 为什么 zenoh-host 要 feature-gate，不直接加

zenoh 1.9 的依赖图：
```
zenoh → ~80 transitive crates
  包括：stabby、flume、tokio-tungstenite、rustls、... 
  first cold build: ~4 minutes on our dev container
```

对**所有人都需要 Zenoh** 的场景（生产飞控最终形态），这没问题。但 M10b 只是**第一次落地** —— workspace 里大多数工作（算法、SITL、MAVLink、firmware link）都不该被拖慢。

Feature gate 的设计意图：
- 默认 `cargo test --workspace` 不拉 zenoh → 4 min build 只收一次
- CI 有专门 `test-zenoh` job 覆盖 zenoh-host 路径
- 集成到实际 demo（替换 MAVLink sitl）的时候再打开

### 为什么 Zenoh，不是 DDS / uORB

项目愿景（plan.md 创新点 4）选的 Zenoh。Evidence 在代码里：

- **PX4 的方法**：uORB (2014 设计) + uxrce-dds (2020 补丁) → 双层翻译，有历史债
- **Zenoh**（2023 Eclipse Foundation）：设计目标就是 **uORB 和 DDS 的统一后继**，单一协议覆盖 embedded（zenoh-pico）+ LAN（zenoh）+ WAN（peering）
- **ROS 2 直接兼容**：`ros2topic echo auto_pilot/estimator/attitude` 直接工作，因为 Zenoh 有 ROS 2 适配插件

day 1 上 Zenoh = 避开 PX4 花了 10 年补上的"从单体消息总线升到网络原生"的痛。

### 为什么 peer mode 不是 client mode

Zenoh 有两种部署：
- **Client + Router**：所有 session 连到中心 router 进程
- **Peer mode**：session 之间直接 UDP multicast 发现彼此（loopback 和 LAN 均可）

SITL / 单机开发用 peer mode：**零配置 run-and-go**，不需要先启动 router daemon。生产部署会放 router（需要流量控制、授权、跨 subnet）。

`open_peer()` 是**开发默认**。`open_with_config()` 让生产部署传自己的配置。

### 为什么 Zenoh 强制 `multi_thread` tokio

```
error: Zenoh runtime doesn't support Tokio's current thread scheduler.
```

Zenoh 内部起自己的 runtime 线程（用于 message forwarding、session maintenance），要求 host tokio 是 multi-thread 变体。单线程调度会**死锁**：session spawn 的 task 等 runtime drive，但 runtime 只能在一个线程上 drive。

我们的测试用 `#[tokio::test(flavor = "multi_thread", worker_threads = 2)]`。生产 app 如果整个 main 都是 tokio multi-thread，这个限制无感。

### 为什么消息 Payload 大小固定 256 B

设计期选的：
- 最大消息（`ActuatorCmdMsg`，8×f32 + u64 + u8）encoded ~45 B
- 256 B 给未来扩展（16+ actuator、新 cov 字段、等）留余量
- 256 = 2^8，栈分配友好

如果未来加**地图 / 栅格 / point cloud 这种大消息**（明显大于 256 B），schema 本身有问题 —— 该拆小消息 + 订阅 + zero-copy buffer pool。而不是把 `MAX_PAYLOAD_LEN` 无脑加大。

`encoded_payload_fits_max_len` 测试 **每次加消息自动检查**，防止悄悄超界。

## How it's verified / 怎么验证的

```bash
$ cargo test -p core-bus
8 passed (每消息 round-trip + corrupt payload + size budget + topic 命名)

$ cargo test -p sim-hil --features zenoh-host
1 passed: setpoint_round_trips_through_two_peers
  （两个 peer session、150 ms discovery、put → recv 2s 超时、byte-exact）

$ cargo test --workspace
233 passed

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿

$ cargo clippy -p sim-hil --features zenoh-host --all-targets -- -D warnings
全绿

$ cargo build -p core-bus --lib --target thumbv7em-none-eabihf
Finished（messages 仍 no_std）
```

## Follow-ups / 遗留

- **M10c zenoh-pico firmware 集成**：把 Zenoh 搬到 STM32H7 上 (`zenoh-pico` C 库 + Rust FFI)。**真机 onboard** 运行 Zenoh，不只 host。
- **ROS 2 bridge smoke test**：docker-compose 起 ROS 2 Jazzy node 订阅 `auto_pilot/estimator/attitude`，SITL 端 publish，校对帧。
- **Topic rate control**：添加 `publish_periodically` helper，按每 topic 的目标频率（见 `docs/topics.md`）自动限流。
- **SITL demo 替换**：sitl_mavlink 的 MAVLink UDP 可以并行加 Zenoh publisher，同步发 telemetry 两条路径。
- **Zenoh queryable**：除了 pub/sub，加 `.query()` API 做 RPC-like 拉取（比如 `get vehicle/config`）。
- **QoS / reliability**：当前 Zenoh put 是 best-effort；actuator_cmd 应该 reliable。加 `put_reliable()` / `put_best_effort()` 区分。
- **消息版本字段**：加 `version: u8` 字段到每条消息，给未来的 breaking schema evolution 留路径。
- **PR 文档**：`docs/topics.md` 更新 IDL 状态从 "TBD" 到 "done"，列出消息字段。
