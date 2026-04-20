# 0029 — 姿态环（四元数误差 → 角速度命令）

**Commit**: `7e4ddec`  **Date**: 2026-04-21  **Milestone**: M2.4

## What / 做了什么

补全了 **内环** 的"上游"：从**期望姿态** 计算**期望角速度**。rate loop 原本由测试直接提供 rate_cmd，现在 `attitude_to_rate` 把 (q_current, q_desired) 变成 rate_cmd。

新增：
- `pub type AttitudeGain = Vector3<f32>`
- `pub fn attitude_to_rate(q_current, q_desired, &k_att) -> RateCommand`
- 4 单元 + 1 proptest（共 15）

## Why / 为什么这么做

### 数学

标准的四元数误差姿态控制器：
```
q_err = q_desired ⊗ q_current*                (⊗ 是 Hamilton 积, * 是共轭)
shortest rotation: 若 q_err.w < 0, 反号
ω_cmd = k_att ⊙ (2 · vec(q_err))
```

`2 · vec(q_err)` 是 Rodrigues 参数：对小误差 = 旋转轴 × 误差角。对大误差也在 `[−π, π]` 单调递增（有符号意义正确）。`2 ×` 的系数来自 `q = (cos(θ/2), sin(θ/2)·n̂)` 的 `2·sin(θ/2) ≈ θ` 一阶近似。

### 为什么必须取最短路径（shortest rotation）

四元数有**双重覆盖**：`q` 和 `-q` 表示同一旋转。`q_err` 和 `-q_err` 表示同一误差但"符号翻转"。

如果 q_err.w < 0，意味着走长路（比如 350° 而不是 -10°）。无脑用 `2·vec(q_err)` 会让控制器走错方向。

解法：若 `q_err.w < 0`，整体反号。相当于认为目标被重定义为 `-q_desired`（同一旋转的另一表示）。

**测试**：`attitude_takes_shortest_rotation` 故意用 350° about z，期望 rate_cmd 是**负** yaw（最短路径）。如果没加 shortest-rotation 判断，这个测试会失败。

### 为什么小角度近似 OK

1 kHz 姿态环 + 典型飞控带宽（10-50 Hz），每步姿态误差 < 5°（0.087 rad）。此时：
```
2·sin(θ/2) = θ - θ³/24 + ...
θ=0.087: 0.0866 vs 0.087 → 相对误差 0.15%
θ=0.1 rad (5.7°): 0.0996 vs 0.1 → 0.04%
```

远小于控制增益的 5-10% 不确定度。完全可以用线性近似。

大角度（比如飞机翻转超过 90°）下 `2·vec` 也没发散，只是非线性 —— PID-类控制器能容忍。

### 为什么退化四元数返回零

如果 EKF 在异常情况下输出 `q_current = (0, 0, 0, 0)`，`q_current.conj()` 也是 0，`q_err = q_desired * 0 = 0`，rate_cmd = (0, 0, 0)。

Zero rate_cmd 是**安全默认**：电机保持当前推力，不恶化姿态。FDIR 层负责触发 failsafe（RTL / 降落）。**`attitude_to_rate` 本身永远不产生 NaN**，这是 unwrap_used=deny 环境下的重要契约。

### 为什么把 `attitude_to_rate` 放在 `algo-indi` 而不是单独 crate

概念上姿态环 ≠ rate 环 ≠ INDI。但工程上：
- 三者都不带状态（纯函数）
- 都用 `RateCommand` / `AttitudeGain` / 其他 Vector 类型
- 都不大（< 20 行）

拆成 3 个 crate 会增加依赖管理成本、阻碍 clippy / Kani 的链式分析。**我选择按"都是控制律"合并在 `algo-indi`。**

如果以后加 LQR / L1-adaptive / Backstepping 等复杂控制律，可以拆出 `algo-attitude`。

### proptest 的 tolerance 5e-3

小角度近似下理论误差是 O(θ³)。θ = 0.05 rad 下三阶项 ≈ 1e-5。实际 proptest tolerance 5e-3 留出 **500× 的宽余** 以容纳：
- 浮点舍入
- Hamilton 积的 6 个乘加顺序差异
- 共轭的符号翻转

tolerance 松够不误报，紧够抓真 bug。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-indi
test result: ok. 15 passed; 0 failed
  含 3 proptest × 256 样本 = 768 随机实例

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **接入 app-copter**：在 rate_loop_step 前面加一层 `attitude_loop_step`，接受 `q_desired` 参数
- **position loop**：外层再加一层，输入期望位置 + 估计位置 → 期望姿态
- **rate limiter**：attitude loop 应该限制输出 ω_max，防止大姿态误差让 rate_cmd 炸（比如 1000 rad/s）
- **Attitude 设定点接口**：MAVLink / offboard 命令如何变成 q_desired，M3+ 接入
- **NMPC 替代 attitude loop**：M4 时用 NMPC 一次产生"未来 10 步姿态轨迹"，绕过 P 控制器
