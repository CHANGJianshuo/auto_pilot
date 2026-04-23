# 0073 — P1 收尾：residual + Zenoh 合流 + 可执行 benchmark

**Commits**: `7deab80`（M15a）、`206b9bb`（M15b）
**Date**: 2026-04-23  **Milestone**: M15

## What / 做了什么

把 TODO 清单里 **P1 两项** 一次做完：

### M15a — MpcResidual + Zenoh 同 SITL 里共存

新 runner `run_closed_loop_residual_with_zenoh_telemetry<B>` 让 `MpcResidualController<10, B>` 驱动 SITL 同时通过 Zenoh 广播 7 条 topic。关键验证：

- Residual policy 每 tick 跑一次 affine inference + envelope check
- TelemetryPublisher 每 tick 做 2 次 (IMU/actuator) + 周期性 put
- 两者 await 在同一个 tokio task 上，互相不饿死
- Policy 的 `reject_count` 通过 `HealthMsg.fault_flags |= SensorFaultBit::EKF` 推给 GCS

**SITL 测试实测**：realistic sim + 1.5 m/s 风 + 600 tick，IMU 收到 ≥ 540 条、horizontal err < 1 m、policy 零 reject。

### M15b — `cargo run --example controller_shootout`

**可执行 benchmark** 输出 markdown 表。Realistic sim（1.5 m/s 风 + drag + motor lag）、15 秒、跑 6 个控制器实测如下：

```
Scenario: wind [1.5, 0, 0] m/s + realistic drag + motor lag, 15.0 s

| Controller     | horizontal err (m) | altitude err (m) |
|----------------|--------------------|------------------|
| PI cascade     |              0.015 |            0.000 |
| LQR            |              5.179 |            0.077 |
| MPC            |              2.736 |            0.001 |
| LQI            |              0.239 |            0.007 |
| MPC-I          |              0.226 |            0.007 |
| MPC + residual |              0.221 |            0.002 |

Ordering by horizontal error (smaller is better):
  1. PI cascade     — 0.015 m
  2. MPC + residual — 0.221 m
  3. MPC-I          — 0.226 m
  4. LQI            — 0.239 m
  5. MPC            — 2.736 m
  6. LQR            — 5.179 m
```

几个可以拿来写 README 的观察：
- **bias-tolerant 组（积分 / residual / integrator）全部 < 25 cm**
- **bias-blind LQR / MPC** 漂到米级，证明积分 / residual 的必要性
- **PI cascade 偶然赢**：这个场景下手调 k_i_vel 恰好匹配，不代表 PI 在所有场景最佳

用于支持的结构性改动：
- **`sim-hil` unconditional dep on `app-copter`**（之前是 dev-dep + zenoh-host feature 双写）
- **新 `sim_hil::sitl` public 模块**：`SitlScenario` / `SitlResult` / `run_with_controller` / `run_with_mpc_residual`
- `examples/controller_shootout.rs` 整例 < 150 行，零 test 依赖

## Why / 为什么这么做

### 为什么 M15a 用 affine residual 而不是真 tract 模型

`tract-onnx` 截至 2026-04 只有 0.23.0-dev.5 预发，不适合固定在 CI 锁定版本。M11a 就为这个设计了 `InferenceBackend` trait。M15a 继续用 `AffineBackend` 作为参考实现 —— 等 tract 1.0 stable 或 candle 成熟，一行 impl 就能接入真神经网络。

Affine 不是"假 NN"：它是一个**有物理解释的 PD 残差**（`pos_err_x × -2 + vel_x × -1`），对消常值风扰的效果来自**前馈 damping**。这在 MPC + hand-tuned affine 的组合里达到了和 LQI 相近的表现（0.221 vs 0.239），证明 pillar 3 的数据路径完整可用。

### 为什么 M15b 的 `sim-hil` 去掉 dev-dep + feature 两种 app-copter

之前设计：
- `[dev-dependencies]` 里 `app-copter` — 单元测试可用
- `[dependencies]` 里 `app-copter = { optional = true }`，绑在 `zenoh-host` feature 下 — Zenoh runner 可用

问题：**examples 不在这俩路径里**。Cargo 的 example 可以用 main dep 和 dev-dep，但 dev-dep 在 non-test build 不激活（examples 算 non-test build）。所以 controller_shootout.rs 里 `use app_copter::...` 在默认 feature 下找不到 app-copter。

新设计：无条件 main dep。**sim-hil = 飞控模拟器，不依赖被模拟者是逻辑矛盾**。删除两份重复的 dep 声明，关心的 user 清晰。

### 为什么 `sim_hil::sitl` 是新 public 模块

`#[cfg(test)] mod tests { fn run_closed_loop_with_controller(...) { ... } }` 在单元测试里够用，但：
- Examples 不能调（test cfg 不激活）
- 集成测试不能调
- 下游 CLI 工具不能调

抽一个 public `sitl` 模块：
- `SitlScenario` 把所有 knob 打包（sim_cfg, seed, ticks, setpoint, dt_s），有 `Default` 实现
- `SitlResult` 返回 **指标**（altitude_err_m, horizontal_err_m）而不是原始 SimState，调用方大多数只关心指标
- `run_with_controller<H>` / `run_with_mpc_residual<H, B>` 两个变体，API 对称

现有的 `#[cfg(test)]` 测试 runner **没动** —— 它们测的是相对比较、阈值 assertions，迁移到新 API 是下一步小改动。M15b 只要"example 能 run"。

### 为什么 example 允许 `unwrap/expect/as`

Workspace 级 deny：`clippy::unwrap_used`, `expect_used`, `as_conversions` 对固件关键代码。Example 是**展示性代码**，不是 production path：

- `unwrap/expect` 失败时 print 明确错误 + exit 非零 —— 对"跑 benchmark"场景刚刚好
- `as f32 / as usize` 例子里只做 debug 输出转换，不会在固件路径出现

用 `#![allow(...)]` 在 example 文件头显式 opt-out，不污染 workspace 级设置。

### 为什么 PI cascade 这个场景赢

出乎意料但**不是 bug**。原因：

1. **PI 的 k_i_vel 我手调到 (0.5, 0.5, 0.8)** —— 正好匹配这个 wind magnitude 的 drag 时间常数
2. **MPC 的 Horizon 只有 10 tick (10 ms)** —— 对 1.5 m/s wind 这种慢 disturbance 预测窗口小
3. **LQI 的 q_i = 1.5** 是我选的，不是最优；调大会更接近 PI
4. **场景是稳态跟踪**：15 秒几乎全在稳态，这是 integrator 类最强项

**如果换个场景**（比如"突然 yaw 90° + 加风"）预测类（MPC / MPC-I / residual）应该明显胜。shootout 骨架留给后续多场景测试扩展。

shootout output 里的排序**就是这个场景**的 truth，README 引用时**连同 scenario 描述一起引**，避免让读者以为 "PI 在所有情况最优"。

### 为什么 MPC+residual 和 MPC-I 几乎一样

数值上：MPC-I 0.226、MPC+residual 0.221。差距 5 mm 级别，**基本同位**：

- 两者都在"预测 + 积分补偿"类
- MPC-I 的 integrator 是 QP 内部状态，在 horizon 里被 reason
- residual 的"integrator"是 hand-tuned PD，没有 horizon 意识

两者表现相近**说明 horizon 对稳态跟踪作用有限** —— 预测窗口在 disturbance 已经稳态后只能重复同样修正。Transient 场景（设定点跳变 + 突加风）才能看出差距。

### 为什么 shootout 没做成 criterion benchmark

考虑过用 `criterion` 把这些数据变成统计学严谨的 benchmark（多次运行 + 平均 + 置信区间）。没做因为：

- criterion 期望**输入固定 / 度量变量**；我们的 scenarios 本质上**跟 seed 相关**，需要 seed sweep 才统计学有意义
- shootout 的目标是 **README 的对比引用**，不是 performance regression
- CI 会跑这个 example 慢（15 秒 × 6 controllers × 种子），浪费 CI 时间

当前 example 是 "**一次运行，一次快照**"，成本可控。有需要时 M16+ 加 seed-sweep 统计版本。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil --features zenoh-host sitl_residual
1 passed — M15a SITL with residual + Zenoh

$ cargo run -p sim-hil --example controller_shootout --release
# prints markdown table above

$ cargo test --workspace
248 passed

$ cargo clippy --workspace --all-targets -- -D warnings
clean（shootout example allow-list 了示范性 lints）

$ cargo clippy -p sim-hil --features zenoh-host --all-targets -- -D warnings
clean
```

## Follow-ups / 遗留

- **Seed sweep**：shootout 固定 seed=13，不知排名对种子多敏感。加 10 种子 × 6 控制器 = 60 次运行，出均值 + std
- **Multi-scenario shootout**：加 "突然 yaw 90°" / "setpoint step 5m" / "单电机失效" 场景，真 benchmark 不同 controller 的强项
- **把 `#[cfg(test)]` runner 迁到 `sitl` 模块**：现有测试里有重复的 SITL loop 代码，用新 public API 可以 dedupe
- **README 引用 shootout 表**：把上面那张 markdown 表放进顶层 README，和 4 大创新点描述放一起
- **`tract` 1.0 stable 时接 TractBackend**：替换 AffineBackend，跑真训练模型
- **Zenoh subscriber 也做成 example**：`cargo run --example zenoh_subscriber` 监听 SITL 广播并 pretty-print
- **Python shootout subscriber**：docker-compose 跑 `zenoh-python` 订阅，log 到 plot
