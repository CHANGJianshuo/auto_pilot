# 0054 — Preflight check：地面还没 ready 就不让起飞

**Commit**: `358592b`  **Date**: 2026-04-22  **Milestone**: M6.4

## What / 做了什么

在 arm / takeoff 之前**主动验证"飞机确实 ready"**。核心 API 只有一个函数：

```rust
pub fn preflight_check(flight: &FlightState) -> Result<(), PreflightReject>
```

按顺序 short-circuit 检查（任一失败即返回）：
1. `gps_health == Healthy`
2. `baro_health == Healthy`
3. `mag_health == Healthy`
4. EKF 位置协方差 trace `< 5 m²`（刚初始化时是 300 m²，落稳 GPS 后 < 1 m²）

Reject 用 `#[repr(u8)]` 枚举：`GpsUnhealthy | BaroUnhealthy | MagUnhealthy | EkfNotConverged`，带 `reason_str() -> &'static str` 方便 log / STATUSTEXT。

Demo 层：
- 新 `Arc<Mutex<Option<PreflightReject>>>` 快照，sim 线程每 tick 写一次；
- UDP 线程收到 ARM/TAKEOFF 时读取：失败就 log warn + `COMMAND_ACK::TEMPORARILY_REJECTED`；
- **DISARM 永远不 gate** —— "关不了电机" 比 "开不了电机" 严重 10 倍。

4 新 app-copter 测试（默认 vehicle 过不了 / 收敛后过 / GPS 健康恶化后拒绝 / 枚举 reason_str 完备性），182 单元测试全绿。

## Why / 为什么这么做

### 为什么要 preflight check

M5.6–M6.3 做的是"会响应指令"——但**响应**不等于**可以安全响应**。想象一下场景：

- 室内刚开机 → GPS 还没 fix → 操作员手滑按 ARM → 电机转起来 → 没有位置反馈的飞机飘走撞墙；
- 寒天 IMU 启动温漂 → EKF 协方差没收敛 → TAKEOFF → 飞机以为自己在原地，其实在漂 → 撞人；
- baro 进水读零 → 高度永远说 "0 m"→ LAND 永远不触发。

工业飞控的通用守则："**arm 前必须证明你已经 ready**"。PX4 甚至不让 GCS 发 ARM，除非 preflight 自检全绿。

### 为什么 GPS/baro/mag 都是"必须 Healthy"

看起来严苛，但在当前架构里：
- 位置环依赖 GPS；
- 高度环依赖 baro；
- 姿态 heading 依赖 mag；

**少任何一个 = 部分维度 open-loop**。工业上可以退化（比如 GPS 失效后切 "optical flow only" 或 "dead reckoning"），但我们现在还没 fallback 模式——所以全部必须 Healthy。

扩展路径：加 `PreflightStrict` vs `PreflightRelaxed`，后者在 degraded 下仍允许起飞（适合 mission 已知 GPS 稀疏的地下 / 室内场景）。

### 为什么位置协方差阈值 5 m²

初始协方差 `initial_sigma::POSITION_NED = 10.0 m` 每轴 σ，对应 `σ² = 100 m²`。trace = 300 m²。

接了几次 GPS 之后，典型收敛值：
- GPS σ = 0.3 m → σ² = 0.09 m²/axis → trace ≈ 0.27 m²；
- 加上 baro noise: trace ≈ 0.3–1 m²；
- 悬停 10 秒后：< 0.5 m² 很常见。

**5 m² = 中间值**，既远高于正常收敛态（避免假阳性），又远低于初始态（避免假阴性）。等价于"每轴 σ ≈ 1.3 m"。

实际部署应该按机型调参：大型机可能起飞前 GPS 差 2-3 m 都算 ready，小竞速机要求 < 0.5 m。

### 为什么 DISARM 不 gate

安全**永远可退**。想象这个场景：

- Arm 成功，起飞；
- 飞到空中 GPS 突然挂（gps_health → Degraded）；
- 操作员按 DISARM；

如果 preflight check 去 gate DISARM → DISARM 会被拒 → 飞机掉天上——灾难。

相反：arm 是 "从静态进动态"（增加熵），disarm 是 "从动态回静态"（减少熵）。增加熵要严格，减少熵要宽松。

具体实现：检查 `data.param1 >= 0.5`（arm 意图），只有 arm 意图才过 preflight；disarm 意图 param1 = 0 直接放行。

### 为什么 preflight 是 read-only 函数

**尽量无副作用**：
- 可以在任何线程 / 任何时机调用，不破坏状态；
- 不修改 FlightState → 调用者随便 probe 不担心污染；
- 单元测试写起来干净（输入 FlightState → 输出 Result）；
- Kani 验证也更容易（将来可以证"health = Healthy AND cov < 5 ⟺ Ok"）。

### 为什么 `Arc<Mutex<Option<PreflightReject>>>`，不用 channel

方案对比：
| 方案 | 优点 | 缺点 |
|---|---|---|
| Arc<Mutex> | 简单、和其他 shared states 同构 | 跨线程锁开销（<100ns，1kHz 下无感） |
| mpsc channel | 无锁 | 需要单独任务管理，复杂 |
| atomic | 最快 | Option<enum> 不能直接 atomic 化 |

对 1 kHz 控制循环读、1 Hz 命令写的场景，**Mutex 够用**。和 setpoint / arm / landing / takeoff / rtl 5 个 shared states 一致，维护性好。

### 为什么 demo 里单独检查 arm_request vs disarm

COMMAND_ACK 发生在 command routing 之后，要复用 preflight 结果：

```rust
let arm_request = matches!(cmd, ARM_DISARM) && data.param1 >= 0.5;
```

这样 ACK 逻辑 mirror 了 arm routing 的 gate 逻辑——`TEMPORARILY_REJECTED` 只在真的 reject 时返回。否则 disarm 命令（param1=0）在 preflight 失败时也会错误地 reject，而实际行为是 accept。

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib preflight
4 passed

$ cargo test --workspace
182 passed, 0 failed

$ cargo fmt --check && cargo clippy --workspace --all-targets -- -D warnings
全绿

$ cargo build -p sim-hil --example sitl_mavlink
Finished
```

**QGC 预期交互**（对比 M6.2/6.3 新行为）：
1. demo 启 + QGC 连；
2. **立刻点 "Arm"** → QGC UI 看到 "Temporarily rejected"，console log `ARM rejected by preflight reason="EKF not converged"`；
3. 等 5-10 秒，GPS/baro/mag 各来几次更新，EKF 协方差收敛到 < 5 m²；
4. 再点 "Arm" → 成功；console log `arm state changed by new=Armed`；
5. TAKEOFF 也一样受 gate。

## Follow-ups / 遗留

- **STATUSTEXT 推到 GCS**：`COMMAND_ACK` 的 result 是枚举，QGC 显示的是 "Rejected" 不是具体 reason。需要发 `STATUSTEXT` 消息把 `reason_str()` 推给 GCS。
- **`result_param2` 附加原因**：MAVLink 2 允许 ACK 附加一个 `u8` 的原因编码，把 `PreflightReject as u8` 塞进去。GCS 能 programmatically 读。
- **Battery check（桩）**：没真实电池传感器，目前永远 "pass"。有 BMS 后按 `voltage / cell < 3.6V` 拒绝。
- **IMU temperature check**：冷启动后 IMU 温度漂移大，应该等温度 > 0°C + 稳定 30s。
- **Takeoff altitude sanity check**：param7 > 100 m 或 < 0 m 应该 `DENIED` 而不是 `TEMPORARILY_REJECTED`（这不是暂时问题，是请求本身无效）。
- **GeoFence**：起飞位置超出预设地理围栏 → `DENIED`。
- **Calibration required**：IMU/mag 没校准时拒绝，引导操作员进 calibration flow。
- **Manual override by GCS**：紧急情况需要 bypass preflight（比如测试场地明知 GPS 弱）。PX4 用 `param2 = 21196`（magic）；我们可以加。
- **M7.0 embedded main.rs**：当前全部在 host 跑，下一步应该把 app-copter 拆成 `no_std` firmware entry。
