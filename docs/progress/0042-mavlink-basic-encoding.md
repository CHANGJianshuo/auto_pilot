# 0042 — MAVLink 2 基础编码（HEARTBEAT / ATTITUDE / GLOBAL_POSITION_INT）

**Commit**: `fbcbbe5`  **Date**: 2026-04-21  **Milestone**: M5.1

## What / 做了什么

把 `mavlink` crate v0.14 接进 workspace，给 `comms-mavlink` 提供 3 个最常用的遥测编码函数（HEARTBEAT、ATTITUDE、GLOBAL_POSITION_INT），还没做 transport（UDP/serial）。

新增：
- workspace Cargo.toml 新增 `mavlink = { version = "0.14", default-features = false, features = ["common", "embedded"] }`
- `FrameBuffer = heapless::Vec<u8, 280>`
- `encode(&MavHeader, &MavMessage) -> FrameBuffer`
- `encode_heartbeat / encode_attitude / encode_global_position_int`
- `quaternion_to_euler`（3-2-1 Hamilton，gimbal-lock-safe）
- 三个 clamp 辅助防止 f64/f32 → 整型溢出
- 5 unit tests

## Why / 为什么这么做

### 为什么先编码不做 transport

MAVLink 分三层：
1. **消息定义**（common dialect：HEARTBEAT 等）
2. **打包/解包**（v2 帧头 + payload + CRC）
3. **传输**（UDP / serial / TCP / 桥接）

编码层独立、可测试、确定性 —— 这是**最值得先做**的。transport 层要选网络库（tokio / embassy）+ 硬件抽象（UART / UDP socket），耦合多。分两步做避免纠缠。

### 为什么 `mavlink` crate 的 `common + embedded`

- `common` dialect 覆盖 QGroundControl / MissionPlanner 认的所有核心消息（HEARTBEAT、ATTITUDE、LOCAL_POSITION_NED 等）
- `embedded` feature 切 `no_std` 友好模式，适合最终固件
- `default-features = false` 不拖 `std`、`tokio` 等主机依赖

### 输入类型用原始 `Quaternion/Vector3`，不拉进 `algo_ekf::State`

保持 `comms-mavlink` 和 filter 解耦 —— app 层做数据整形：

```rust
let q = flight.state.attitude;
let omega = flight.last_gyro_filtered;  // 或 state 的估计
let frame = encode_attitude(1, 1, seq, t_ms, q, omega);
send_over_udp(&frame);
```

### 为什么 flat-earth NED → lat/lon

GLOBAL_POSITION_INT 要地球中心坐标。严格做需 WGS-84 Vincenty solver（100+ 行代码）。近距离（< 10 km）用平面近似：

```
delta_lat_rad = x_ned / R_earth
delta_lon_rad = y_ned / (R_earth · cos(lat_origin))
```

误差 ~1 m at 10 km 半径。对本项目第一阶段 SITL 和试飞完全够用。未来长航程切 Vincenty。

### Clamp 而不是 unwrap

MAVLink 用 `i32 / i16 / u16` 紧凑整型。如果浮点输入是 NaN 或 ±∞ 或越界：

```rust
fn clamp_f32_to_i16(v: f32) -> i16 {
    if !v.is_finite() { return 0; }
    let clamped = v.clamp(f32::from(i16::MIN), f32::from(i16::MAX));
    clamped as i16
}
```

返回 0 而非 panic 符合 CLAUDE.md "firmware never panic"规约。

### MAVLink 2 帧布局细节

踩了一个坑：第一次测试我写 `frame[5]==0 sequence` —— 但 v2 帧头是：

```
[0] STX (0xFD)
[1] payload length
[2] incompat flags
[3] compat flags
[4] sequence         ← 这里
[5] system ID
[6] component ID
[7..10] message ID (3 bytes)
[10..] payload
[...] CRC (2 bytes)
```

MAVLink v1 和 v2 sequence 位置都是第 2 或 3 或 4——版本不一样。v2 是 idx 4。已经在 test assertion 里修对了。

### GLOBAL_POSITION_INT payload 长度变化

v2 做了 "**truncated zero tail**" 优化：尾部连续的 0 字节不发送，解码方补零。意思是：payload 长度是**变量**（≤ 28 字节）。我第一次 assert `frame[1] == 28` 是错的，改成了"≤ 12 overhead + 28 payload"。

## How it's verified / 怎么验证的

```bash
$ cargo test -p comms-mavlink
test result: ok. 5 passed; 0 failed
  heartbeat_round_trips
  attitude_encodes_expected_euler
  global_position_int_wraps_heading
  global_position_int_handles_negative_heading
  clamp_helpers_reject_non_finite

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M5.2 transport 层**：UDP socket for SITL（host 上用 tokio），后续 UART for firmware
- **Streaming parser**：从字节流提取完整帧 —— `mavlink` crate 的 API 还是 `io::Read` 绑定，需要 slice 适配
- **命令接收**：SET_POSITION_TARGET_LOCAL_NED、COMMAND_LONG (ARM/DISARM / LAND) 等
- **状态类消息**：SYS_STATUS（电池、FDIR overall_health → MAV_STATE 映射）、VFR_HUD（airspeed / heading）
- **GPS_RAW_INT**：从 EKF 的 state 无信息可下发，直接从真 GPS 驱动生成
- **RC 输入**：RC_CHANNELS 接 SBUS/ELRS 驱动
- **长航程**：替换 flat-earth → WGS-84 Vincenty 精确度 < cm
