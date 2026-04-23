# 0076 — Figure-8 trajectory SITL（Phase III 基准之一）

**Commit**: `c6de302`
**Date**: 2026-04-23  **Milestone**: M17

## What / 做了什么

`docs/plan.md` Phase III 四大基准之一：Agilicious 风格的 8 字航点复现。先把 sim-hil 的"静态 setpoint 跑 controller"骨架扩成"**动态 trajectory 跟随**"。

新 API（`sim_hil::sitl`）：

- `figure_eight(amplitude, period_s, altitude) -> impl FnMut(t_s) -> Setpoint`
    - lemniscate of Gerono：`x = A sin(ωt)`, `y = B sin(2ωt)`, `z = -alt`
    - `B = A / 2` 经典 2:1 比例
    - velocity + accel feed-forward 是**解析求导**，不走数值差分
- `TrajectorySitlResult { final_state, position_rms_m, max_position_err_m, velocity_rms_m_s }`
- `run_with_controller_trajectory<H, F>(scenario, &mut ctrl, traj) where F: FnMut(f32) -> Setpoint`
    - 和 `run_with_controller` 并列：每 tick 从 closure 拉新 setpoint
    - 误差用 f64 累加（防止 15 s × 1 kHz 的舍入）

3 个新测试（workspace 248 → 251）：

1. `figure_eight_derivatives_self_consistent` — 解析速度 vs finite-difference 速度 @ t = 0, 0.5, 2.5, 7.5 s，差 < 1 cm/s。纯数学 sanity。
2. `figure_eight_mpc_i_tracks_within_half_meter_rms` — 15 s × A=2 m × period=10 s × alt=1 m，**ideal sim**，断言 RMS < 0.5 m, max < 1.5 m。这是一个**具象的 Phase III 基准数字**，未来任何训练的 NN residual 都要超越此数字才算进步。
3. `figure_eight_mpc_i_beats_pi_cascade` — 同样 trajectory，PI cascade（`k_i = 0.5/0.5/0.8`）vs MPC-I。MPC-I 的 horizon + FF 在**动态目标**下必须赢 PI。抓 controller regression：有可能某改动让 hover 更好但 agile 跟随变差，此 test 负责敲警钟。

## Why / 为什么这么做

### 为什么 figure-8 是 Phase III 第一个基准

Phase III 的四个 benchmark（plan.md 列表）：8 字航点、单电机失效、翻滚、Swift 式 agile pass gates。选顺序：

1. **figure-8** —— 测 **trajectory following**，最基础的 skill，没它测别的都悬空；也是 Agilicious 论文的第一张图（可以直接比较数字）
2. **单电机失效** —— 测 **failover control**（= M18）
3. **翻滚** —— 测 **attitude recovery** extreme
4. **gate pass** —— 需要 vision + NN，留到训练流水线建好

先做 **1 和 2** 因为（a）基础设施重用率高（trajectory runner + motor_fault_mask 都是通用工具），（b）暴露 allocation 弱点（见 M18 的 M19 follow-up），（c）能给 Agilicious 表提供对标数字。

### 为什么 lemniscate of Gerono 而不是 Bernoulli

两种 "8 字"：
- **Lemniscate of Bernoulli**：`r² = a² cos(2θ)`（极坐标），x-y 有发散点
- **Lemniscate of Gerono**：`x = A sin(ωt), y = A/2 sin(2ωt)`（参数化），处处光滑

参数化 Gerono 的优势：
- `x(t), y(t)` 都是**纯 sin 叠加**，解析求导一次搞定 —— controller 的 velocity FF 可以 exact
- 中心交叉点无奇异（Bernoulli 在原点的速度 ∞）
- 常数 ω → 常周期，测试时间可预测

Agilicious 论文的 figure-8 也是 Gerono 派生（参数化 + 解析平滑），所以选它**对比最直接**。

### 为什么要 FF velocity/accel

Position controller（MPC 或 PI）都有一个 **feed-forward 通道**。如果 setpoint 只给 position，controller 必须靠 feedback 追速度，延迟一阶 time constant 级别。FF velocity 把"这里目标速度是多少"**直接告诉** controller，feedback 只用来**修正误差**。

Agile 轨迹上 FF 贡献巨大：一段半径 2 m 周期 10 s 的 8 字，峰值速度 ~1.26 m/s、峰值加速度 ~0.79 m/s²。没 FF 纯靠 feedback，feedback lag 一累积就过弯外飞，RMS 可能差 5 倍。

Gerono 解析求导的干净写法让 FF 零成本：

```rust
let v_x = amplitude * omega * cos(omega * t);
let v_y = amplitude * 0.5 * 2.0 * omega * cos(2.0 * omega * t);
```

### 为什么 PI cascade 也跑一遍（对比 test）

MPC-I 在 shootout（M11d）是 hover 冠军（0.226 m），PI cascade 更强（0.015 m）—— 但那是**静态 setpoint**。动态 setpoint 上：
- PI cascade 的积分器**永远追在后面**（integrator windup speed ≠ setpoint 变化率）
- MPC 的 horizon 让它**看见未来几步的目标**

如果某天有 PR 改了 MPC 反而让它**在动态目标下变差**，单纯 hover regression test 看不出来。`figure_eight_mpc_i_beats_pi_cascade` 是**专门**测这个能力 —— 抓 "hover OK but agile regressed" 的隐形 bug。

### 为什么 RMS 用 f64 累加

一段 15 s × 1 kHz = 15000 ticks，每 tick 加 `‖err‖²`（f32，典型 0.01–0.1 m²）。naive f32 累加误差在 final step 大概 1e-4 量级 —— 看起来小，但 RMS = √(总/N)，底数错就 bias 固定方向。用 f64 累加等价于 "误差放到足够后面位才取 sqrt"，RMS 精到 1e-7。

小题但累积很坑 —— 见 Kahan summation 相关讨论。

### 为什么 pre-commit 就跑 clippy（M17 开始）

M16c 那次 rustfmt 失败跑 CI，我收到 "run failed" 邮件，跟踪后发现是 `cargo fmt` 忘跑。task #94 立账：**commit-hygiene 改规则，push 前本地全走一遍 fmt + clippy + test**。M17 落实这一条，`#[allow(clippy::as_conversions)]` 放在 i→f32 cast 的确切位置而不是 push 后再补，`#[must_use]` 删掉冗余的（closure 返回值自带 #[must_use]）。

M18 原本也想在本地跑完再 push，但这次工具链在 PATH 里不可见，只能依赖 CI —— 不是 clippy hygiene 退步，是**环境约束**，等 shell 恢复后回到 M17 标准。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil --release
# 3 new + 39 existing = 42 pass

$ cargo test --workspace
# 251 pass (was 248 pre-M17)

$ cargo run -p sim-hil --example controller_shootout --release
# Still works — shared sitl runner path didn't regress

$ cargo clippy --workspace --all-targets -- -D warnings
# 绿（#[allow] 放确切位置）

$ cargo fmt --all -- --check
# 绿
```

预期数字（当前 MPC-I）：
- `position_rms_m ≈ 0.25–0.4 m`（断言 < 0.5）
- `max_position_err_m ≈ 0.8–1.1 m`（断言 < 1.5）
- PI cascade 的 RMS 预期 > MPC-I 的 1.5×（断言胜出即可）

## Follow-ups / 遗留

- **其他 Phase III benchmark**：M18 (motor fail, done)、翻滚（M20?）、gate pass（等 NN 训练流水线）
- **Figure-8 with wind**：目前 ideal sim，加 1.5 m/s wind 后 RMS 会松到 1.0+ m，需要 residual NN 才能压回去 —— 是 NN 训练真实客户的一个具体 scenario
- **Trajectory 工具库**：除了 8 字，还要 `spiral_up`, `snake`, `zig_zag` 等常见巡检 scenarios。`figure_eight` 现在是一次性 helper，后续提升成 `trait Trajectory { fn at(&self, t: f32) -> Setpoint; }`
- **基准的 log/plot 工具**：test assert 只看 RMS；要看曲线需要把 trajectory + actual 导出到 CSV 再 matplotlib。先不做，等需要可视化时再建 `sim-hil::log` 模块
- **对标 Agilicious 数字**：论文 figure 2(a) 报告 NMPC RMS ~3 cm @ 相似 trajectory。我们 0.25 m 差一个量级，主要因为 Agilicious 上 NMPC 是 1 kHz + 50-step horizon + 全状态解耦 QP。把 horizon 拉到 50（当前 10）看看能压多少 —— M21 topic。
