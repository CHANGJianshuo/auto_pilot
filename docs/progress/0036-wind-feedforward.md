# 0036 — 风扰前馈（从 EKF.wind_ne 到 outer loop）

**Commit**: `9eeaaa9`  **Date**: 2026-04-21  **Milestone**: M3.3

## What / 做了什么

把 EKF 状态里的 `wind_ne` 引到 `outer_step`，作为**前馈**加到 `setpoint.accel_ned` 上。当 EKF 识别出风扰时（未来工作）或外部风速传感器给出风速时，控制器可以**零延迟**反抗风。

新增：
- `RateLoopConfig::wind_ff_gain: f32`（默认 0.2）
- `outer_step` 在调用 PI 控制器前调整 `setpoint.accel_ned += k · wind_ne · (x,y,0)`
- 1 个新测试（共 10）

## Why / 为什么这么做

### 两种互补的扰动补偿路径

| 机制 | 响应速度 | 适用 |
|---|---|---|
| **I 项**（M3.2）| ~ 秒级（积分累积）| 稳态未建模扰动（drag 偏差、misalignment） |
| **前馈**（本步）| 零延迟 | 已知 / 可估计的扰动（风、外部推力） |

实际系统**两者都要**：I 吸收慢变低频扰动，FF 抵消快变但可观测扰动。PX4 ekf2 + `MulticopterPositionControl` 也是这样（它们用 `wind_feedforward_enabled` 参数）。

### 为什么 `k_wind = 0.2`

物理意义：如果风速 = wind_ne（m/s），drag 力 = k_drag · wind_ne。这个力让 vehicle 跟着风漂，要抵消它需要反向 thrust。

换成 accel：`a_compensate = F_drag / m = k_drag / m · wind_ne`

对 250 g quad + `k_drag = 0.05 N·s/m`：`0.05 / 0.25 = 0.2 s⁻¹`。这就是 `wind_ff_gain`。

### 为什么只补偿水平 (x, y)，不补 z

`wind_ne` 是 EKF 里的 **horizontal** 2D 状态（W 和 N 两个分量，不含 vertical）。多旋翼垂直方向对风扰敏感度低（翼面投影小），垂直风扰用气压计和 z 位置环自然处理。

### 为什么要单独的 `wind_ff_gain` 而不是塞进 PositionGains

概念上 wind FF 是"**飞机具体参数**"而非"**控制器调参**"：
- 不同飞机 `k_drag / mass` 不同，但控制律参数（k_p / k_v / k_i）可以复用
- 把它放在 `RateLoopConfig` 而非 `PositionGains`，让 `PositionGains` 保持**飞机无关**

### 为什么测试要比较 **with FF** vs **without FF**

绝对精度很难断言（motor 值依赖很多因素）。相对比较 = 确定 FF 信号真的流到了 motor。零 FF 时 wind_ne 被忽略；正 FF 时 motors 被重新分配。diff > 1 mN 足够证明信号路径工作。

### 现在测不到"end-to-end"效益，因为 wind_ne 不会自动更新

没写 EKF 的 wind-observable 模型（drag Jacobian 要加到 predict step 的 F 矩阵），所以 `wind_ne` 永远停在初值 (0,0)。本步**先把机械件架好**，用 manual set 的方式证明 FF 链路通。未来 M4 加风观测（飞行数据里 `v_measured − v_predicted_no_drag` 的残差归因到 wind_ne）时，wind FF 自动就开始做真工作，不需要改控制律代码。

### 所以这一步的价值是什么

- **设计正确性**：把 wind FF 接口和 EKF wind_ne 状态正确挂上
- **未来零改动**：加风观测时只改 EKF predict，控制器不动
- **手动覆盖能力**：今天就能通过外部风速计（如 pitot-static）直接设 wind_ne，控制律立即受益

## How it's verified / 怎么验证的

```bash
$ cargo test -p app-copter --lib
test result: ok. 10 passed; 0 failed
  outer_step_wind_feedforward_tilts_against_wind ← 新

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **EKF 风观测模型**：在 predict step 加 drag 项，让 wind_ne 从 IMU residual 可观测
- **wind FF gain 自动校准**：飞行中观察 `v_ned_response` vs `thrust` 自动识别 k_drag
- **3D 风扰**（含垂直分量）：升级 wind state 为 Vector3，给垂直 thrust 做贡献
- **风梯度**：大型飞行器需要考虑高度变化引起的风变；本步只做局部常数
- **SITL 验证**：给 run_closed_loop_flight 加一个"oracle wind injected into flight.state.wind_ne"开关，验证端到端收敛速度提升
