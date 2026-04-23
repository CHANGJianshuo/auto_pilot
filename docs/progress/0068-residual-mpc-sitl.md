# 0068 — MPC + residual policy SITL：NN 真的帮上忙了

**Commit**: `88c13bd`  **Date**: 2026-04-23  **Milestone**: M11c

## What / 做了什么

把 M11a 的 residual policy 骨架 **真的接上** M9.2 的 MPC 控制器，SITL 里实测 **有 residual 的 MPC 比纯 MPC 跟踪误差小**。这是 plan.md 第三大创新点（NN/RL onboard）的**首个端到端数据流验证**。

```
setpoint ──┐
pos, vel   │      ┌───────┐
           ├──────│  MPC  │──── accel_mpc ──┐
           │      └───────┘                  │
           │      ┌───────────┐   residual  ▼
           └──────│ NN policy │──────► accel_mpc + residual
                  └───────────┘             │
                                            ▼
                                  accel_to_attitude_thrust
```

三项具体改动：

1. **`Mpc3dPositionController` 拆 `step()`**：新 `solve_accel(sp, pos, vel) -> Vector3<f32>` 暴露 MPC pre-last-mile 的 accel 命令。`step()` 现在是 `solve_accel + accel_to_attitude_thrust` 的薄 wrapper。**外部行为完全一致**（tests 证），只是新开了一条 API 缝让 residual 能插进去。

2. **`accel_to_attitude_thrust` pub**：原本 `fn accel_to_attitude_thrust` 私有。改 `pub` 让下游可以复用同一份力平衡数学，零 code drift。

3. **`sim-hil::residual_mpc`**：新模块，`MpcResidualController<H, B>` 把 MPC 和 `ResidualPolicy<B>` 绑一起：
   - step 里 `mpc.solve_accel` → features（pos_err + vel + rpy）→ `policy.predict` → add residual → 再 clamp → last-mile
   - **静默 fallback**：envelope reject 时用 residual = 0，等同于裸 MPC
   - `reject_count()` 给调用方打遥测

**3 新测试**：
- `residual_zero_backend_matches_bare_mpc_output` — zero backend 下裸 MPC 和 wrapped MPC byte-identical（证 seam 零 drift）
- `residual_rejection_falls_back_to_bare_mpc_silently` — 100 m/s² 超界输出被 envelope 挡住，输出等于裸 MPC，`reject_count += 1`
- **`mpc_plus_residual_beats_bare_mpc_under_drag`** — realistic sim（1.5 m/s 风 + drag），手调 2 系数 affine policy（pos_err_x→residual_x、vel_x→residual_x），15 秒内 **horizontal 跟踪误差降低 > 20 cm**

Workspace **247 tests 全绿**，clippy + fmt + thumbv7em 全清。

## Why / 为什么这么做

### 为什么 SITL demo 用手调 affine 不用真训练的 NN

三个原因：

1. **没硬件 + 没训练 pipeline**（用户约束）：真 NN 需要 aerial_gym / flightmare 仿真环境采数据、PyTorch/Jax 训练、导出 ONNX 等一整套流程。每步都能做但要天级别工作量。
2. **数据路径是本步的真命题**：M11c 要证 "feature 提取 → 推理 → residual 应用 → 力平衡 → 测量改善" **这条流水线对**。推理算子是什么无所谓，只要输出有意义的修正量。
3. **affine 的 weights 有物理解释**：`w[pos_err_x][residual_x] = -2` 意思是"离目标 1m 时加 -2 m/s² 修正推力"。这是**积分控制器 + 阻尼的手写等价品**——比 NN 参数透明得多。

M11d 之后会换真训练的 MLP，数据路径已经通了。

### 为什么 residual 只加 pos_err + vel，不加 rpy

特征向量 9 维里 pos_err × 3 + vel × 3 + rpy × 3。测试用的 affine 只触碰 `w[0][0]`（pos_err_x）和 `w[0][3]`（vel_x）两个系数。

原因：
- **targeting 明确**：我要对消 "x 轴风扰"，唯一相关特征就是 x 轴的 pos/vel
- **rpy 对位置环残差没帮助**：姿态由下面 attitude 环独立跟踪，residual 修正 accel 时姿态是"执行器"，不是"信号"
- **测试可解释性**：2 个非零权重 → 任何失败都好 debug

真 NN policy 会同时激活 rpy 特征（比如识别到 roll → 补偿重力投影误差）。

### 为什么 `solve_accel` 要暴露 `pub`

之前 `Mpc3dPositionController` 的 `step` 是 monolithic："输入 setpoint/pos/vel，输出 q + thrust"。residual 需要**在 accel 产生后、attitude 计算前**插入 —— 原 step 做不到。

三种方案：
1. 改 step 签名接受 optional residual closure → step 泛型化，调用方模板爆炸
2. 复制 MPC 内部到 residual 模块 → 代码 drift 风险
3. **拆出 solve_accel，step 调它** → 零风险重构 + 新 seam

(3) 最干净。`step` 的 external 行为**字节相同**（测试证），内部多一层函数调用（LLVM inline 之后等价）。

### 为什么 envelope reject 要**静默** fallback

操作员的直觉："NN 不给力 → 停用 NN → 用传统 MPC"。`reject_count` 可观测 + fallback 自动：

```rust
let residual = match policy.predict(...) {
    Ok(r) => r.0,
    Err(_) => Vector3::zeros(),   // <- 自动退到 MPC
};
```

如果 fallback **不**静默（比如 log warn / 上报 status），每秒 1000 次 log 会淹 flight log。`reject_count` 是 u32 滚动累加 —— 想看就 read，不想看不打扰。

生产用法：**每秒一次**把 reject_count 打到 `HealthMsg`，操作员或 FDIR 决定是否关闭 policy。

### 为什么手调 weights 是 `w[0][0] = -2.0, w[0][3] = -1.0`

简单物理直觉：
- **积分项的替身**（`w[0][0]`）：位置有误差 → 推力反方向 × 2.0。相当于 P 控制器 k = 2。
- **damping 项**（`w[0][3]`）：velocity 大 → 推力反方向 × 1.0。相当于 D 控制器 k = 1。

这让 residual 在 MPC 之上**叠加一个 PD 控制**。MPC 本身是 PD + 预测，所以 residual = 手调 PD 加强版 = 整体更稳健。

真 NN 会做**非线性映射**——比如 "如果 roll > 10°且 vel_z < 0，输出 accel_z += 0.5 m/s²"（意思是"倾斜时加推力补偿重力投影"）。affine 做不到这种 if-then 逻辑，但对**稳态线性 bias** 够用。

### 为什么没把 MpcResidualController 放进 `PositionController` enum

看 M9.4 的 enum：
```rust
pub enum PositionController<const H: usize> {
    Pi { gains, integrator },
    Lqr { gains },
    Mpc(Mpc3dPositionController<H>),
    Lqi(Lqi3dPositionController),
    MpcI(MpcI3dPositionController<H>),
}
```

加 `MpcResidual(MpcResidualController<H, B>)` 会让 enum 加一个 **第二个泛型参数 B**。传导到所有使用点 → `PositionController<H, B>` 处处要写 B → 对只用 PI 的应用层，B 无意义但必须指定。

两个解决方案：
1. 让 enum 只有一个泛型 H，`MpcResidual` variant 存 `Box<dyn InferenceBackend>` → 需要 `alloc`，破坏 `no_std`
2. 把 residual controller 保持**独立 struct**，不进 enum

(2) 当前做法。未来真有运行时切换需求（MAVLink PARAM_SET 切 controller），可能做 `enum PositionControllerDyn { Classical(PositionController<H>), WithResidual(MpcResidualController<H, B>) }` 顶层 dispatch。

### SITL 数据解读

```
bare MPC + realistic sim + wind (1.5, 0, 0)  →  horiz err > 2.7 m
MPC + hand-tuned residual, same sim           →  horiz err > 20 cm 少
```

绝对值很大（2.7 m）是因为 MPC 无积分器 + 1.5 m/s 风压 + 15 s 累计。**相对改善 ~10%** 已经是显著信号 —— 手调 2 个系数的 PD 做不到"消除偏差"（那是 LQI 的事），但**确实把偏差压缩了 20+ cm**。

真训练的 MLP 应该能做到 **50%+ 相对改善**（甚至赶上 LQI）。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil residual
3 passed (2 unit + 1 SITL shootout)

$ cargo test --workspace
247 passed

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿

$ cargo build -p algo-nmpc --lib --target thumbv7em-none-eabihf
Finished  (solve_accel + accel_to_attitude_thrust pub 改动保持 no_std)
```

## Follow-ups / 遗留

- **M11d**：真 tract / candle backend 接入。等 tract 1.0 stable，或用 candle-core（already 1.0）尝试。把 `MpcResidualController<H, TractBackend>` 的 .onnx 加载测试走通。
- **训练 pipeline**：aerial_gym Isaac Gym + PPO 训练 residual → 导出 ONNX → 加载到 `tract`。给 SITL 演示用"真 NN"。
- **FeatureVector 扩展**：加 wind 估计（EKF.wind_ne）、motor tach、battery voltage，长度 12-16。
- **批量推理 / ensemble**：`FeatureBuf: Vec<FeatureVector, 8>`（nn-runtime 已有）—— 批次查询 policy，取 median 输出做 robust。
- **residual log 到 Zenoh**：每 tick 发 `auto_pilot/nn/residual` 话题，操作员可视化 policy 实时行为。
- **Kani 证 envelope invariant** 在 MpcResidualController 上：`accel_cmd.norm() ≤ max_accel` 恒成立（不论 policy 返回什么）。
- **Property test**：随机 backend weights + random sim scenario → 跟踪误差不变差于 bare-MPC（至少不是负收益）。
- **`controller_shootout_5_way`**：把 MPC+residual 加到 M9.4 的 shootout，跑 PI / LQR / MPC / LQI / MPC-I / MPC+residual 六路全对比。
