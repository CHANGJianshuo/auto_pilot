# 0083 — MAVLink STATUSTEXT encoder

**Commit**: `b4a4b8f`
**Date**: 2026-04-24  **Milestone**: M23a（先头部队）

## What / 做了什么

在 `comms-mavlink` 里加了 `encode_statustext(sys, comp, seq, severity, text) -> FrameBuffer`，同时 `pub use MavSeverity`。让飞控能把 pilot-facing 警报（"MOTOR 0 FAILED"、"EKF DEGRADED"、"PREFLIGHT REJECTED: GPS unhealthy" 等）通过 MAVLink STATUSTEXT 推给 GCS —— QGC 之类的地面站会 toast-notification 显示。

API：

```rust
encode_statustext(
    system_id: u8, component_id: u8, sequence: u8,
    severity: MavSeverity,
    text: &str,
) -> FrameBuffer
```

语义：
- MAVLink STATUSTEXT 是 fixed 50-byte UTF-8 + severity 枚举 (EMERGENCY..DEBUG)
- 输入文本 > 49 bytes 截断到 49 + null terminator
- 输入文本 < 49 bytes null-pad
- 不做 multi-chunk 分段（现行 firmware 的警报都一个 message 内能装下）

2 个新 test（round-trip + 截断）。

## Why / 为什么这么做

### 为什么先做 encoder 不做 SITL 集成

Single-responsibility commit：encoder 本身是 comms-mavlink 里 150 行的 self-contained delivery，就地测试。SITL 集成（"detector 一声明 → emit STATUSTEXT"）涉及 `sim-hil::sitl_mavlink` runner 的改造 + tokio task 交互 + UDP 接收方 —— 另一次 commit 的工作量。

把两者分开 ⇒
- M23a 现在 mergeable（CI 过 = 编解码正确）
- M23b 再做 SITL 连线（CI 过 = 端到端 detector → STATUSTEXT → UDP）
- 两次各 < 300 行 diff，审查每次都小

### 为什么 STATUSTEXT 而不是自定义 MAVLink message

可选路径：
1. **STATUSTEXT**（选这个）：MAVLink common，任何 GCS 都显示；代价是 50-byte 文本（没结构化数据）
2. **ESC_STATUS**（MAVLink #291）：per-ESC voltage / current / RPM / errors；结构化但 **需要 ESC telemetry 支持**（bidirectional DShot / UAVCAN）—— SITL 没有
3. **ESC_INFO**（#290）：static ESC info，同上
4. **Custom message**：自定 MAVLink dialect，需要每个 GCS 自己加载 XML 定义 → 生态成本高

STATUSTEXT 是最**不需要定制**的路线，覆盖 ~95% 的 "告诉 pilot 出事了" 需求。Structured ESC_STATUS 是 Phase II 真机上有 bidirectional DShot 之后做（M30+）。

### 为什么 `pub use MavSeverity` 放在 comms-mavlink

严格按 Rust 封装原则，caller 可以 `use mavlink::common::MavSeverity` 直接取。但：
- comms-mavlink 已经把 `MavCmd`、`MavResult`、`MavMessage` 等打包，`MavSeverity` 不 re-export 会让调用方的 use 列表分两行
- 未来如果换 MAVLink 实现（比如 `mavspec` 或自研），通过 comms-mavlink 的 re-export 可以 **一次替换** 不改调用方

后向兼容 / 一致性考量 > 严格最小封装。

### 为什么 null-pad 尾部（而不是随便填）

MAVLink STATUSTEXT 文档说 text 是 "MAV_SEVERITY level + ASCII string"，不强制 null termination，但**每个 GCS 实现都假设 null termination**。不 null-pad 会导致 QGC 显示乱码（读到前一次 buffer 的残留）。

truncation 规则是 49 chars + 1 null = 50 bytes 满员。一个 "MOTOR 0 FAILED at t=12.345s\n reason=ESC_OVERCURRENT" 之类的长告警会被砍到 49 字节截短版。

## How it's verified / 怎么验证的

```bash
$ cargo test -p comms-mavlink
# 原 N tests + 2 新（statustext_round_trips_short_message, _truncates_overlong_text）

$ cargo test --workspace
# expect: ~259 tests pass
```

CI 依赖。

## Follow-ups / 遗留

- **M23b — SITL 集成**：`sim-hil::sitl_mavlink` 里：
    - 每个 tick 比较 `flight.motor_fault_detector.level()` 和上次记录值
    - 如果从 非-Emergency → Emergency，emit STATUSTEXT "MOTOR N FAILED"（N 从 `motor_fault_detector.alive()` 推出，找 false 的 idx）
    - 通过 `MavlinkUdpSink` 发出去
    - 端到端测试：spawn UDP listener，断言收到至少一个 STATUSTEXT，body 包含 "MOTOR"
- **其他 pilot alerts**：
    - `preflight_check` 拒绝时 emit STATUSTEXT "PREFLIGHT REJECTED: <reason>"
    - HealthLevel 升级到 Emergency 时 emit "<subsystem> EMERGENCY"
    - Landing 触地 emit "LANDED"
    - RTL 激活 emit "RETURN TO LAUNCH"
- **Severity 分配约定**：建议 emergency 退 → `MAV_SEVERITY_EMERGENCY`；degraded 次 → `CRITICAL`；info 态事件（arm / land） → `INFO`。收敛成一份 `docs/alerts.md` reference。
- **Rate limiting**：detector 每 tick 都可能见 Emergency 状态；不 rate-limit 就会每 tick 发一条 STATUSTEXT → 刷爆 MAVLink 链路。应该 "edge triggered"（level 变化时才发）or "N/second rate limit"。SITL 集成时要做对。
