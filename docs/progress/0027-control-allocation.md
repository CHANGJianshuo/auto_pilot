# 0027 — 控制分配（X-quad 效率矩阵）

**Commit**: `747b0a3`  **Date**: 2026-04-21  **Milestone**: M2.1a

## What / 做了什么

把 INDI 输出的**虚拟命令**（3 轴力矩 + 总推力）映射到**每个电机的推力**。对 N=4 的 X 构型用矩阵求逆；N>4（六/八旋翼）留给 QP 版本（M2.1b）。

新增：
- `pub struct MotorGeometry { position_m, yaw_torque_per_thrust }`
- `pub struct VirtualCommand { torque: Vector3, thrust_n: f32 }`（含 `as_vector4()` 打包）
- `pub fn build_effectiveness<const N: usize>(&[...; N]) -> SMatrix<f32, 4, N>`
- `pub fn invert_quad_effectiveness(&Matrix4) -> Option<Matrix4>`
- `pub fn allocate<const N: usize>(e_pinv, cmd) -> SVector<f32, N>`
- `pub fn saturate<const N: usize>(thrusts, min, max) -> SVector<f32, N>`
- `pub fn standard_x_quad(arm_m, k_yaw) -> [MotorGeometry; 4]` —— X 构型预设
- 7 测试（5 单元 + 2 proptest）

## Why / 为什么这么做

### 效率矩阵的物理

每个电机的推力 `T_i`（沿 body -z 方向）产生：
- **Roll 力矩** `τ_x = −y_i · T_i`（右侧电机 +y，推力增 → 右侧上升 → 负 roll）
- **Pitch 力矩** `τ_y = +x_i · T_i`（前方电机 +x，推力增 → 前方上升 → +pitch，即仰头）
- **Yaw 力矩** `τ_z = k_yaw_i · T_i`（螺旋桨反扭矩，CW/CCW 决定符号）
- **Thrust** `F = T_i`

写成矩阵：
```
[τ_x]     [ −y_1  −y_2  ...  −y_N ] [T_1]
[τ_y]  =  [ +x_1  +x_2  ...  +x_N ] [T_2]
[τ_z]     [ k_1   k_2   ...  k_N  ] [...]
[F]       [ 1     1     ...  1    ] [T_N]
```

`E` 是 **4×N**。求解 `T = E⁺ · virtual_cmd`（N=4 用直接逆，N>4 用 Moore-Penrose 伪逆）。

### 为什么暴露 `E` 和 `E_pinv` 是两个独立函数

`build_effectiveness` 接受几何，返回 `E`。应用层在启动时**一次**算 `E_pinv`，然后**每次 1 kHz rate loop** 跑 `allocate(e_pinv, cmd)` 纯矩阵乘法。

如果把整个管道封装起来每次都求逆，1 kHz 下 24³ = 13K 次操作每次都浪费 CPU。**预计算永远比重算快**。

### 为什么 `saturate` 只 clamp 不 redistribute

满载时简单 clamp 会让某些轴（比如 yaw）被"偷"。正确做法（M2.1b）：**优先满足姿态轴，牺牲 thrust**（宁可掉高度也要保持姿态）。用 QP 解：
```
minimize ‖τ_cmd − E·T_clamped‖²
subject to T_min ≤ T ≤ T_max
```

本 commit 故意**只做 clamp**，让 M2.1b 加 QP 时有明确的性能基线对比（saturated 空中姿态崩 vs QP 优雅降级）。

### 为什么 motor numbering 看起来怪

我用的不是 Betaflight 的"顺时针编号"，也不是 ArduCopter 的"按电机顺序"。而是**按几何位置随意**。真正的调用方（`app-copter`）会把 motor index → ESC channel 映射写在配置里。**分配算法不依赖 numbering**。

### round-trip proptest 的价值

```
virtual → allocate → motor thrusts → E · thrusts → virtual'
```

256 随机 virtual（在合理范围内），反 pack 回来应等于原值到 1e-3。这是**分配矩阵数学正确性**的最直接验证。如果 E 的哪一列符号写错，round-trip 会立刻失败。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-alloc
test result: ok. 7 passed; 0 failed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **M2.1b**：QP 版本，处理 saturation 重分配（宁牺牲 thrust 保姿态）
- **M2.1c**：电机失效重分配 —— 单电机失效（N=4 → N=3 有效）时退化到 3-DoF 控制，放弃 yaw 控制保住姿态
- **M2.1d**：rate limiter —— 电机加速有物理极限（~100 RPM/ms），防止指令跳变让 ESC 饱和
- **参数化**：`arm_m` / `k_yaw` 现在是字面量，需要进 parameter 系统
- **N>4 变体**：Hex/octa 的 pseudo-inverse（`nalgebra::SMatrix::pseudo_inverse` 返回 Result，要包装）
