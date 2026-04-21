# 0043 — MAVLink UDP 传输（单向遥测）

**Commit**: `4acaba7`  **Date**: 2026-04-21  **Milestone**: M5.2

## What / 做了什么

在 `sim-hil` 加 UDP 传输层，让 M5.1 写好的 MAVLink 编码函数能**真的发给 QGroundControl**。

新增：
- `sim-hil` 依赖 `comms-mavlink`
- `pub mod mavlink_udp::MavlinkUdpSink`：tokio `UdpSocket` 封装
  - `bind(local, remote)`
  - `send_heartbeat()`
  - `send_attitude(t_ms, q, omega)`
  - `send_global_position_int(...)`
  - AtomicU8 seq counter（线程安全）
- 2 个集成测试

## Why / 为什么这么做

### 为什么 UDP 不是 TCP

MAVLink 过去 20 年都用 UDP：
1. **低延迟**：没 TCP 三次握手、重传、队头阻塞
2. **无连接**：GCS 中途掉线不影响无人机发送
3. **广播友好**：可以同时向多个 GCS 多播
4. **MAVLink 2 有自己的序列号/CRC**，不需要 TCP 的可靠性

QGroundControl 默认监听 UDP 14550 端口，MissionPlanner 一样。

### 为什么放在 `sim-hil` 不放在 `comms-mavlink`

- `comms-mavlink` 是 `no_std`，嵌入式固件也能用。UDP 是 std/tokio only
- 未来固件上 transport 会是 UART（via `core-hal`），不是 UDP
- 所以"**协议打包 + 无依赖**"归 comms-mavlink，"**具体 socket**"归 sim-hil

这保持了 "**算法/协议 crate 不碰平台**" 的架构原则。

### 为什么 `AtomicU8` 做 sequence

`&self` 方法发送 → sequence 不能用 `&mut self`（多任务共享 sink）。`AtomicU8::fetch_add` 是 lock-free 的 wrapping increment，正合适：
- Drone 一般一秒几百条消息，wrap 后旋转 256 次是常见的
- MAVLink 2 sequence 是 u8，wrap 是协议规定的

### `send_to` 而非 `connect + send`

`UdpSocket::connect` 后 `send` 没法切换远端。MAVLink 场景里未来可能支持**多 GCS**（pilot 一个、observer 一个），要用 `send_to` 灵活些。当前 `MavlinkUdpSink` 固定一个 remote，但预留升级路径。

### Timeout 1 秒

测试里 `tokio::time::timeout(1 s, recv)` 足够发送 + 接收 —— localhost UDP 延迟 < 1 ms。1 秒对"容器里偶尔抽风"的情况留了余量。

### 为什么两个 tests 包含了足够的覆盖

1. `sink_bind_and_send_round_trip` —— 验证"打得出 + 收得到"（基本存活）
2. `sequence_increments` —— 验证状态管理正确（序号递增）

再深入（attitude 测试验证 byte 内容）会和 M5.1 的 encoder 测试重复。这里只测传输层。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil
test result: ok. 13 passed; 0 failed
  + mavlink_udp::tests::sink_bind_and_send_round_trip
  + mavlink_udp::tests::sequence_increments

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## 实际使用示例

```rust
use sim_hil::mavlink_udp::MavlinkUdpSink;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sink = MavlinkUdpSink::bind("0.0.0.0:14551", "127.0.0.1:14550").await?;

    let mut interval_hb = tokio::time::interval(Duration::from_millis(1000));
    let mut interval_att = tokio::time::interval(Duration::from_millis(50));

    loop {
        tokio::select! {
            _ = interval_hb.tick() => { sink.send_heartbeat().await?; }
            _ = interval_att.tick() => {
                sink.send_attitude(
                    now_boot_ms(),
                    flight.state.attitude,
                    flight.last_gyro_filtered,
                ).await?;
            }
        }
    }
}
```

QGC 打开 → Vehicle connection → UDP → 14550 → 自动 detect 并显示姿态条。

## Follow-ups / 遗留

- **M5.3 双向**：parse SET_POSITION_TARGET_LOCAL_NED、COMMAND_LONG (ARM/DISARM / LAND)，从 `recv_from` 拿字节 → 用 `mavlink` crate 解帧 → 喂 setpoint
- **MAVLink router 模式**：一个 sink 广播到多个 GCS
- **遥测 scheduler**：HEARTBEAT 1 Hz、ATTITUDE 50 Hz、GLOBAL_POSITION_INT 5 Hz 等的时间表；目前是调用者自己 tick
- **嵌入式 UART transport**：`core-hal` 里加 `MavlinkUartSink`，跟这个 UDP sink 同一个协议层，方便 firmware 复用
- **demo binary**：把 rate_loop_step + MavlinkUdpSink 串起来的 `examples/sitl_with_mavlink.rs`，一键启动
