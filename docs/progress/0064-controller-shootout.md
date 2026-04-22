# 0064 — PositionController 枚举统一 + 四路 shootout

**Commit**: `4585489`  **Date**: 2026-04-22  **Milestone**: M9.4

## What / 做了什么

M9.0-M9.3 的**四路位置控制器**有各自独立的 API，调用方要写四种不同的样板。M9.4 合成一个统一枚举，一个 `.step()` 换四种行为：

```rust
let mut ctrl: PositionController<10> = match mode {
    Mode::Pi   => PositionController::pi(gains),
    Mode::Lqr  => PositionController::lqr(w_xy, w_z, dt, max_accel)?,
    Mode::Mpc  => PositionController::mpc(cfg_xy, cfg_z, iter, max_accel)?,
    Mode::Lqi  => PositionController::lqi(w_xy, w_z, dt, max_accel, i_max)?,
};
let att = ctrl.step(&setpoint, pos, vel, mass, dt);  // 一个 API 覆盖全部
```

**Shootout SITL**：写一个通用 `run_closed_loop_with_controller<H>`，把四路都跑同一场景。跑两个对比：

1. `controller_shootout_ideal_sim_all_variants_hover` —— ideal sim 3 秒，4 路都悬停到 1 m，误差 < 25 cm
2. `controller_shootout_realistic_sim_bias_tolerant_variants_win` —— 实际 sim（2 m/s 风 + drag），15 秒：
   - **PI+I** 和 **LQI**（带积分器的）< 0.6 m altitude err
   - **LQR**（无积分器）给 2 m 宽上界（记录偏差，不是"通过"而是"不炸")

这第二个测试**就是回归屏障** —— 将来任何"让 LQI 变慢 / 让 PI 失去积分" 的 bug 会立刻弹。

**+5 tests**（3 enum unit + 2 SITL shootout），workspace **218 tests 全绿**。

## Why / 为什么这么做

### 为什么用枚举 + dispatch 而不是 trait object

两种统一方式：
| 方案 | 优点 | 缺点 |
|---|---|---|
| `dyn PositionController` | 动态分派、C++ 风格熟悉 | 有 vtable、`no_std` 下需要 `alloc::boxed::Box` |
| `enum` + `match` | 静态分派、零成本、`no_std` 友好 | 新 variant 要改 enum |

飞控固件用 `no_std`、不要 heap —— `enum` 是唯一工业级正确选择。`match` 在 4 个 variant 上 LLVM 会编译成单个跳转表，性能与直接调具体函数等价。

### 为什么 `H` const-generic 泄漏到 enum

`Mpc3dPositionController<H>` 的 warm buffer 是 `SVector<f32, H>`。`SVector` 是栈分配的 compile-time sized vector —— 要支持它作为 enum variant 的 payload，enum 本身必须看到 H。

非-MPC variants 里 H 是"死"的（不实例化任何大小-H 的东西）。应用层**挑一个 H** 用到底：
- H = 10（10 ms lookahead @ 1 kHz）—— 当前默认
- H = 20 / 50 —— 需要更强预测时

一个 codebase 里混用多个 H 的场景没有，`H` 泄漏代价可接受。

### 为什么 `reset()` API 统一

不同 variant 的状态清零操作不同：
- PI: integrator ← zero vec
- LQR: nothing
- MPC: warm buffers ← zero
- LQI: integrator ← zero

操作员在 mode 切换时（比如 MAVLink 发送新 waypoint，飞机要从"巡航"切"急停"）必须有一键清零。`ctrl.reset()` 打包四种逻辑。变量名 / 字段差异全在 impl 里，调用方看不见。

### 为什么 shootout 不直接失败 LQR

LQR 在 realistic sim 下**本来就该**有稳态偏差（无积分器 + 模型失配）。硬性失败 LQR 等于把"LQR 数学不完备"当 bug 测 —— 错误 framing。

正确做法：**用松阈值记录这个基线**。新 variant 出现后：
- **若 variant 是积分器类**：应该能 tighter（< 0.6 m）→ assert
- **若 variant 是预测类**：应该在 transient 期间 tighter → 加 transient-window assert
- **若 variant 是 LQR-like**：用松阈值（< 2 m）证明**至少稳定**不掉天上

这让 shootout 不是"通过/失败"二元，而是分**regime 分层**的正则矩阵。

### 为什么 LQI 用 `k_i_vel` 在 PI+I 里

`PositionGains::k_i_vel` 是 PI cascade 的 velocity-error 积分增益。在 shootout 的 realistic sim 里：

```rust
PositionController::pi(PositionGains {
    k_i_vel: Vector3::new(0.5, 0.5, 0.8),
    ..PositionGains::default()
})
```

0.5/0.8 是手调数值。与 LQI 的**代价最优推导**形成对比 —— 两个达到同样性能但一个靠经验、一个靠 DARE 求解。

### 为什么没把 PositionController 接进 `outer_step`

`outer_step` 里糅了 landing/takeoff/RTL override + wind FF + home latch + 触地自动 disarm + takeoff 到位判断 + PI 调用 —— **业务逻辑 + 控制律纠缠**。

要接 PositionController 进去：
1. `RateLoopConfig` 加 `pos_ctrl: PositionController<H>` 字段 → 整个 config 变成泛型，connected files 全得加 `<H>`
2. `FlightState::vel_integrator` 变成冗余（PI variant 内部已存）
3. outer_step 的 PI-specific path（wind FF、conditional integration）要分解到各 variant

这是**架构层 refactor**，不是 M9.4 scope。真正想运行时切换控制器的驱动力出现（比如 MAVLink `PARAM_SET` 让 GCS 调 controller）时再做。

当前 M9.4 解决了"**统一接口 + 头对头对比**"这两个可落地的具体需求。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-nmpc controller_enum
3 passed:
  controller_enum_constructs_every_variant
  controller_enum_each_variant_holds_hover_at_origin
  controller_enum_reset_clears_state

$ cargo test -p sim-hil controller_shootout
2 passed:
  controller_shootout_ideal_sim_all_variants_hover
    ← 4 路均 < 25 cm / 3 s
  controller_shootout_realistic_sim_bias_tolerant_variants_win
    ← PI+I / LQI < 0.6 m；LQR 给 2 m 松界（稳定性下界）

$ cargo test --workspace
218 passed

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿

$ cargo build -p algo-nmpc --lib --target thumbv7em-none-eabihf
Finished (no_std ok)
```

## Follow-ups / 遗留

- **M9.5 MPC-I**：把 LQI 的 integrator 状态加进 MPC 的 augmented state space，得到"约束 + 积分"的一体化控制器。用 `compute_lqi_gains` 的 3×3 Riccati P 作终端代价。
- **`outer_step` 集成**：等 MAVLink PARAM_SET 运行时 controller 切换需求出现再做。
- **`sim-hil/examples/controller_shootout.rs`**：非 test 的 executable，输出 `kind | alt_err | horiz_err | settling_time` 表格供 README 引用。
- **Property test**：对每个 variant 的 `.step()` 做 quaternion unit-norm + thrust-non-negative 不变量。
- **WCET benchmark**：`criterion` bench `.step()` 的 per-variant 时间。MPC 预期最慢（iter），LQI/LQR 最快（单纯 vec 运算），PI 中等（cascade + integrator）。
- **Runtime swap test**：从 PI 切到 LQI 的过程 + `reset()` 行为的专门测试。
- **Kani invariant**：`ctrl.step(sp=0, x=0, v=0)` 永远产生 hover thrust + 单位 quaternion。静态证明跨 variant 的 "do nothing" property。
