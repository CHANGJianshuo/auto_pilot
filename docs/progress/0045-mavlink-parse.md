# 0045 — MAVLink 2 解析器（字节 → 消息）

**Commit**: `fcb2b09`  **Date**: 2026-04-21  **Milestone**: M5.4

## What / 做了什么

补上 MAVLink 的**接收侧**。之前只能发（HEARTBEAT / ATTITUDE / GLOBAL_POSITION_INT），现在也能从字节 slice 解出消息。下一步 QGC → 无人机的命令通路（setpoint, ARM/DISARM）就是把 `parse_frame` 串进 UDP recv。

新增：
- `pub enum ParseError { Incomplete, Corrupt, Unknown }`
- `pub fn parse_frame(bytes: &[u8]) -> Result<(MavHeader, MavMessage), ParseError>`
- 4 个新测试（共 9）

## Why / 为什么这么做

### 为什么 `&[u8]` 能直接喂

`mavlink` crate v0.14 的 `read_v2_msg` 用 `embedded_io::Read` trait，而**原生 `&[u8]` 已经实现**了 embedded_io::Read。所以没有 std / Cursor 折腾：

```rust
let mut reader = PeekReader::<&[u8], 280>::new(bytes);
mavlink::read_v2_msg::<MavMessage, _>(&mut reader)
```

一次性解出一帧。**保持 no_std** —— 固件也能用，串口驱动收完字节直接调就行。

### 错误分类

```rust
pub enum ParseError {
    Incomplete,   // 还没收完整
    Corrupt,      // CRC 错或消息 ID 未知
    Unknown,      // 保留给未来扩展
}
```

这三类对**接收策略**是有意义的区分：
- `Incomplete`：等下次 `recv_from`，把新字节拼上重试
- `Corrupt`：可能是 glitch，丢弃重新等帧同步字节
- `Unknown`（未使用）：以后支持更大的 dialect 时，识别"本 dialect 不认识但 CRC 合法"的帧

### 为什么不返回 `consumed` 字节数

当前 `parse_frame` 每次只处理一个 datagram（UDP 一帧 = 一个消息，典型）。如果转 serial/tcp 流式场景，需要 `parse_frame_at(slice, start) -> (result, new_start)`。这个等 M5.5 有接收需要时再做。

### 为什么 CRC 翻转要能检测

MAVLink 2 每帧末尾两字节 CRC16/X25 校验全部字段。**flipping 一个 byte 必导致 CRC mismatch**，返回 `ParseError::Corrupt`。

这是帧同步的基础 —— 接收方用 CRC 把"找到了真帧"和"字节流里刚好出现了 0xFD"分开。

### 测试覆盖

| 测试 | 验证 |
|---|---|
| heartbeat_round_trip_through_parser | 基本编码-解码对等 |
| attitude_round_trip_preserves_euler | 浮点字段（yaw、body rate、time）数值保真 |
| parse_rejects_short_buffer | 不完整帧返回 Incomplete |
| parse_rejects_corrupted_frame | CRC 错误被检出 |

前两个保证 **"我们编码的帧能被标准 parser 解回去"**。第三、四保证**边界情况不 crash**。

## How it's verified / 怎么验证的

```bash
$ cargo test -p comms-mavlink
test result: ok. 9 passed; 0 failed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished

$ cargo test --workspace
  所有 crate 都绿
```

## Follow-ups / 遗留

- **M5.5**：`MavlinkUdpSink` 扩成 `MavlinkUdpNode` + `recv_frame()`，把进来的帧 dispatch
  - `SET_POSITION_TARGET_LOCAL_NED` → 更新 Setpoint
  - `COMMAND_LONG(MAV_CMD_COMPONENT_ARM_DISARM)` → arm/disarm 状态机
  - `COMMAND_LONG(MAV_CMD_NAV_LAND)` → 触发降落模式
  - 忽略未知消息类型（`ParseError::Unknown`）
- **流式解析**：`parse_frame_at(slice, start) -> (result, consumed)`，一次 UDP 帧可能包含多条（某些 bridge 会 batch）
- **Signed packets**：`read_v2_msg_signed` 带签名校验版本，生产环境防欺骗
- **Dialect 扩展**：目前只用 `common`；`ardupilotmega` 有额外自定义消息，按需加 features
