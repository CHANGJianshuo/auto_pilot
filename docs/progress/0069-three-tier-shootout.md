# 0069 — 三档 shootout：residual 和 LQI 谁更强？

**Commit**: `0f680c8`  **Date**: 2026-04-23  **Milestone**: M11d

## What / 做了什么

把 M11c 的 MPC+residual 加进回归矩阵，做**同场景**三档对比测试：

```
bare MPC           — bias-blind baseline, 自由漂移
MPC + residual     — PD-shaped 手调 affine 残差（M11c）
LQI                — 积分器消 bias（M9.3）
```

同一 realistic sim（1.5 m/s 风 + drag + motor lag + 15 秒）下实测：

| 控制器 | 横向误差 |
|---|---|
| bare MPC | > 0.5 m（假设最低漂移量，实际更大） |
| **MPC + residual** | **≈ 0.22 m** |
| **LQI** | **≈ 0.24 m** |

意外结果：**手调 PD 残差 + MPC 居然和 LQI 持平，甚至略胜**。理论上没记忆的 PD 应该输给有积分器的 LQI（PD 不能消常值 bias，只有积分能），但**MPC 的预测 look-ahead 承担了一部分 "积分" 功能**，两者合力接近了 LQI。

测试断言**档位**而非 ordering，允许两者相对关系随 tuning 摇摆。Workspace **248 tests 全绿**。

## Why / 为什么这么做

### 为什么断言档位，不是绝对 ordering

起初我写了 `lqi_horiz < residual_horiz`，假设 LQI 一定赢（有 integrator 嘛）。跑完实测 0.22 < 0.24 ——**residual 赢了**。手调 PD 系数 `(w[pos_err][x] = -2, w[vel][x] = -1)` 碰巧比 LQI tuning 略更激进。

两种 fix：
1. **调软 residual** 让它输给 LQI → 但这是 p-hacking，把 hand-tune 做得更差只为让测试符合预期
2. **承认 tie，assert 档位** → 两者都在"bias-tolerant class"，不比谁赢

选 (2) 因为：
- 测试的目的是**抓 regression**，不是证明"A 一定比 B 好"
- "residual 和 LQI 是同档次方法" 是有信息量的**正确描述**
- 将来真 NN policy 或改 LQI tuning 让 ordering 真有明确偏好时，再 tight test

断言的**档位**定义：
- `residual_err * 3 < bare_err` — residual 把误差削 3× 以上
- `lqi_err * 3 < bare_err` — LQI 把误差削 3× 以上
- `max(residual, lqi) / min(...) < 3` — 两者相对误差在 3× 之内
- `bare_err > 0.5` — 裸 MPC 确实在漂

这四条组合起来捕获四类 regression：
1. residual 或 LQI 突然变差 3× （单路退化）
2. 某个组件意外加了 bias 补偿让 bare MPC 不再漂 （测试前提失效）
3. residual 和 LQI 分化进不同档位（一个远好于另一个）
4. 任一控制器 step 函数数值异常（误差跳飞）

### 为什么 hand-tuned affine 能追上 LQI

物理直觉上不应该——常值 bias 需要积分。但：

**MPC 已经有内置 "pseudo-integrator"**：
- MPC solve 返回的 u_0 是**多步 horizon 最小化**的结果
- 对于稳态偏差，horizon H=10 里每步都有 pos_err cost，累积相当于"短期积分"
- 不是无限时间的真积分，所以不能完全消 bias，但**在 wind 稳态下减少 bias 的放大**

**PD residual 补上 "积分外" 的反馈**：
- `pos_err × -2` 相当于"直接按误差量值推回去"，比 LQR 的 k_p 更激进
- `vel × -1` 阻尼项防止 overshoot

两者**合力**：MPC 预测 + PD 侵略 ≈ LQI 的 `k_p + k_v + k_i` 三件套。

真 NN policy 会学到更复杂的非线性映射（比如根据姿态补偿重力投影），表现应该**超过** LQI，但 M11c/d 还没到那一步。

### 为什么 ratio 上限 3× 而不是 2×

保守性：
- sim 有随机性（Xorshift RNG 种子影响初始漂移）
- 两控制器在同设置下的误差可能 0.2 vs 0.3 m（1.5× 比率）—— 不应该 fail
- 3× 给足够余量让正常的参数/种子扰动通过，但挡住**真正的退化**

如果 residual 或 LQI 悄悄变差到 3× 以上，要么是真 bug，要么是有意的行为改变——都值得红。

### 为什么单独一条 `bare_err > 0.5`

这条断言**反向**保护：如果有人加了一个"意外的 bias fix"到 bare MPC（比如改 MPC 权重让它间接抗 bias），那么 bare vs residual 的对比就失真了。

`bare_err > 0.5` 确认 "裸 MPC 在这个场景真的有漂移"，否则三档对比没意义。

### 为什么用 `AffineBackend::zero()` 当 bare MPC 代理

`run_closed_loop_with_mpc_residual<B>` 签名要求一个 backend。zero backend 输出 `Residual::zeros()`，在 `MpcResidualController.step` 里：
```rust
let mut accel_cmd = accel_mpc + residual;  // residual = 0
```
→ accel_cmd = accel_mpc 完全等于裸 MPC。M11c 的 `residual_zero_backend_matches_bare_mpc_output` test 已经证明这条路径 byte-identical。

好处：**同一 test harness 覆盖 "bare MPC" 和 "MPC+residual"**，只有 backend 变。避免写两份平行的 SITL loop。

## How it's verified / 怎么验证的

```bash
$ cargo test -p sim-hil controller_shootout_residual
1 passed  (≈45 秒，三个 15k 步 SITL 串行)

$ cargo test --workspace
248 passed

$ cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --check
全绿
```

## Follow-ups / 遗留

- **更多 seed**：当前用 seed=13，加 `#[test]` with 多种子跑多次，验证 ordering 不是种子偶然
- **固定 seed 压力 test**：`controller_shootout_residual_robust_across_seeds` — 10 个种子跑 residual + LQI，统计平均 + std 断言
- **用真 NN policy 重跑**：等 tract 1.0 / candle 稳定，用训练的 MLP 替换 affine，预期 residual 明显胜 LQI（非线性 + 更多特征）
- **transient 对比**：当前只看 15 s 稳态。加个 "大 step setpoint 变化" 场景看瞬态响应谁更快
- **加 wind_feedforward 对比**：`M3.3 wind_ff` 也能消 bias；把它加进来做 4 档 shootout
- **跨场景 sweep**：wind magnitude 从 0.5 到 5 m/s 扫，画 error vs wind 曲线 —— benchmark 出每个 controller 的 envelope
- **non-linearity 测试**：quadratic drag / nonlinear motor thrust，验证 PD 残差在更非线性场景下是否仍能抗衡 LQI
