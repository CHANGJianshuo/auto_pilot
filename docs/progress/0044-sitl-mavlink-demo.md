# 0044 — `sitl_mavlink` demo 二进制

**Commit**: `849c9f3`  **Date**: 2026-04-21  **Milestone**: M5.3

## What / 做了什么

一个可以**一键跑** 的 cargo example，把 M3.1 SITL + M5.1 MAVLink 编码 + M5.2 UDP sink 串成可运行的 demo。用户：

```bash
cargo run -p sim-hil --example sitl_mavlink
# 打开 QGroundControl → 自动连上 → 看到悬停的四旋翼
```

核心线路：
- 后台 `spawn_blocking` 线程跑 1 kHz 闭环仿真（predict + outer_step + step）
- 每 10 tick 把 Snapshot（attitude / body_rate / pos / vel / heading）发到 mpsc channel
- Tokio 主 task 按 GCS 期望的速率 drain 并打成 MAVLink：
  - HEARTBEAT 1 Hz
  - ATTITUDE 50 Hz
  - GLOBAL_POSITION_INT 5 Hz
- 仿真线程 wall-clock paced —— 保证 QGC 看到的数据率自然

Zurich (47.397742°, 8.545594°) 地理原点（Agilicious 的家）。

## Why / 为什么这么做

### 为什么 spawn_blocking 跑 sim

Tokio 的 cooperative scheduling 对"**紧 CPU 循环**" 不友好 —— 没有 .await 的地方会饿死其他 task。Sim 是 1 kHz 纯计算，放在 blocking 线程里隔离：
- `spawn_blocking(fn)` 用专门的 thread pool
- 主 task 照常 process UDP 和 timer tick
- mpsc channel 做跨线程通信（lock-free for unbounded）

### 为什么用 mpsc 不用 `Arc<Mutex<Snapshot>>`

Mutex 每 tick 要获锁 → sim 线程速率受 tokio runtime scheduling 影响。mpsc 的 `try_recv` + `send` 是 lock-free 原子操作，sim 不会被打断。

代价：消息堆积风险。用 `unbounded` + `try_recv` 排干 → 只保留 latest snapshot。这是 "**watchtower pattern**"—— 你不关心历史，只要最新值。

### MAVLink 速率选择

| 消息 | 速率 | 为什么 |
|---|---|---|
| HEARTBEAT | 1 Hz | QGC 用它做"存活判定"，必须 ≥ 0.5 Hz 不然报 "Vehicle Lost" |
| ATTITUDE | 50 Hz | QGC 姿态条流畅度 —— 低于 20 Hz 会看到卡顿 |
| GLOBAL_POSITION_INT | 5 Hz | 地图刷新不需要太快，GPS 本身 5 Hz |

这些是 PX4/ArduPilot 的默认值。

### Wall-clock pacing

```rust
next_wake += step_duration;
if next_wake > now { sleep(next_wake - now); }
else { next_wake = now; }
```

关键是**累积目标时间**而不是每次 `sleep(1ms)` —— 后者会累积计算开销变慢。catch-up 逻辑（`else next_wake = now`）避免系统 stall 后的雪崩重试。

### 为什么是 example 不是 binary

`cargo run --example` 是 cargo 对 "**演示性代码**" 的标准位置。比起 `src/bin/sitl_mavlink.rs`：
- 不影响 `cargo build -p sim-hil` 的默认输出
- 和 `tests/` 一样只在显式调用时编译
- examples/* 的 code standards 稍微宽松（比如 `main -> Result`）

## How it's verified / 怎么验证的

```bash
$ cargo build -p sim-hil --example sitl_mavlink
   Finished `dev` profile

$ cargo clippy --workspace --all-targets -- -D warnings
Finished

$ cargo test --workspace   (全部 200+ 测试仍过)
```

**真机验证步骤**（容器里跑 demo + 宿主开 QGC）：
1. `make -C docker shell`
2. container 内：`cargo run -p sim-hil --example sitl_mavlink -- 172.17.0.1:14550`
   （Docker bridge gateway 是 172.17.0.1，主机 QGC 应该在那监听）
3. 宿主启动 QGC → 应该看到 vehicle 出现在 Zurich

## Follow-ups / 遗留

- **M5.4**：双向 —— parse SET_POSITION_TARGET_LOCAL_NED、ARM/DISARM command 从 QGC 进来，推给 Setpoint
- **参数化 setpoint**：demo 里硬编码 hover。加 CLI 参数支持位置 / 圆周 / 方形 mission
- **Multi-vehicle**：同一 sink 广播 sys_id 1..N 做 swarm 演示
- **容器 → 宿主网络**：Docker network 下 GCS 看不到 container，需要 host network 或 port-forward
- **录制/回放**：保存 sim state 轨迹到 `.bin` 文件，离线可视化
