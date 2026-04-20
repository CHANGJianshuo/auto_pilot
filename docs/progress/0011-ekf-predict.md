# 0011 — EKF predict 步：Strapdown IMU 积分

**Commit**: `d5ca634`  **Date**: 2026-04-20  **Milestone**: M1.7

## What / 做了什么

写出第一条**真算法** —— EKF 的 predict 环。消费一帧 IMU，把姿态/速度/位置向前积分一步。

新增：
- `pub const GRAVITY_M_S2 = 9.80665` —— 标准重力
- `pub struct ImuMeasurement { gyro_rad_s, accel_m_s2 }` —— algo 层自己的 IMU 类型（不依赖 `core-hal::ImuSample`）
- `pub fn quaternion_exp(ω, dt) -> Quaternion` —— 精确四元数指数，小角度 fallback
- `State::predict(&self, imu, dt) -> State`
- **5 个新 proptest**（本 crate 共 13 测试）：
  - `predict_free_fall_accumulates_gravity` —— `v_z → g·dt, p_z → ½·g·dt²`
  - `predict_level_stationary_holds_position` —— 水平静止时 v、p 保持零
  - `predict_zero_rate_preserves_attitude` —— ω=0 不改变 q
  - `predict_preserves_quaternion_norm` —— 任意 ω ∈[-10,10] rad/s，归一化不变
  - `predict_rejects_bad_dt` —— dt<0 或非有限 → 保持原状态

## Why / 为什么这么做

### Strapdown INS 数学

这套公式是所有工业级 EKF 的共同骨架（PX4 ekf2、ArduPilot AP_NavEKF3、Agilicious EKF、Mahony/Madgwick 互补滤波）：

```
ω = gyro_measured - b_g
f_body = accel_measured - b_a           (specific force, 含重力反作用)
q_new = q ⊗ exp(½·ω·dt)                 (四元数指数)
a_ned = R(q_new) · f_body + (0, 0, +g)  (NED 约定：重力在 +z)
v_new = v + a_ned · dt
p_new = p + v·dt + ½·a_ned·dt²
```

`bias / mag / wind` 此处不动 —— 它们在 measurement update 里通过过程噪声模型演化，不属于 predict 的职责。

### 为什么 `ImuMeasurement` 在 algo-ekf 里定义（不复用 core-hal::ImuSample）

严格的"**算法层不依赖 HAL**"原则。好处：
- `cargo check -p algo-ekf` 不触发整个 HAL 编译链
- Kani 的符号执行域更小（少一层类型抽象）
- 任何 CI 环境（host、FMU、别的公司的 FMU）都能跑 algo 测试

代价是应用层需要 glue：
```rust
fn imu_to_measurement(s: ImuSample) -> ImuMeasurement {
    ImuMeasurement { gyro_rad_s: s.gyro_rad_s, accel_m_s2: s.accel_m_s2 }
}
```
一行转换 + 丢弃时间戳（算法层用 dt 不用绝对时间）。

### 为什么四元数指数用**精确**形式 + 小角度 fallback

```rust
pub fn quaternion_exp(omega: Vector3<f32>, dt_s: f32) -> Quaternion<f32> {
    let half_vec = omega * (0.5 * dt_s);
    let half_norm_sq = half_vec.dot(&half_vec);
    if !half_norm_sq.is_finite() || half_norm_sq < 1.0e-16 {
        return Quaternion::new(1.0, half_vec.x, half_vec.y, half_vec.z);
    }
    let half_norm = libm::sqrtf(half_norm_sq);
    let (sin_h, cos_h) = (libm::sinf(half_norm), libm::cosf(half_norm));
    let scale = sin_h / half_norm;
    Quaternion::new(cos_h, half_vec.x * scale, half_vec.y * scale, half_vec.z * scale)
}
```

- **精确形式**（cos, sin/θ）在大角速度下也无累积误差。1 kHz × 1000°/s 的最坏情况是 θ≈17°，小角度一阶近似会有 0.005 rad (~0.3°) 误差，10 秒后累积成几度 —— 不可接受。
- **小角度 fallback**：当 `θ² < 1e-16`（即 θ < 1e-8 rad），`sin(θ)/θ → 1`，直接写成 `[1, θ_vec]`。避免 `0/0` 产生 NaN。

### 为什么 NED gravity 是 `+z`

NED 坐标系：x = North, y = East, z = **Down**。重力向量指向下，在这个坐标系下是 `(0, 0, +g)`。

加速度计测量的是**比力** `f = a_inertial - g_inertial`。静止时 `a_inertial = 0`，所以 `f_world = -g_world = (0, 0, -g)` —— 传感器感觉是"被向上推"（反作用力）。

算回 inertial 加速度：
```
a_inertial = f_world + g_world = R(q)·f_body + (0, 0, +g)
```

静止水平时：`f_body = R^-1 · (0,0,-g) = (0,0,-g)` （水平即 R=I），代入：
```
a_inertial = R·(0,0,-g) + (0,0,+g) = (0,0,-g) + (0,0,+g) = 0  ✓
```

这就是 `predict_level_stationary_holds_position` proptest 要验证的。

### 为什么 5 个 proptest 而不是单元测试

每个 proptest 都跑 256 随机样本：
- **free fall** 覆盖所有 `dt ∈ [0.1ms, 50ms]`
- **level stationary** 同上
- **zero rate preserves** 覆盖所有合法 dt
- **preserves norm** 覆盖 ω × dt 的整个可能矩形
- **rejects bad dt** 覆盖所有负数 dt

单元测试只能手写几个 case，proptest 把"在输入区间内公式恒成立"这种结构性质抓得极紧。后续我们改 predict 内部实现（比如加 RK4 积分器），这些不变量仍然必须成立，proptest 会立刻发现数学错误。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-ekf
test result: ok. 13 passed; 0 failed
  含 7 个 proptest × 256 样本 = 1792 随机实例通过

$ cargo clippy --workspace --all-targets -- -D warnings
Finished

$ cargo kani -p algo-ekf
VERIFICATION:- SUCCESSFUL (原有 check_zero_quaternion_returns_none 仍通过)
```

## Follow-ups / 遗留

- **predict 没有算协方差 P 的更新**：那是 EKF 完整 predict 的另一半。需要 24×24 的 Jacobian `F`，过程噪声 Q。下一步工作（M1.8）。
- **旋转应用了两次 sqrt**：一次 `quaternion_exp` 的半角范数，一次归一化。优化：用单位化的角轴表示省一次 sqrt。**暂不做**——正确性优先，微优化放到 benchmark 显示瓶颈时再说。
- **没用 Kani 证**：predict 内部密集使用 sqrt/sin/cos/除法，Kani 对 libm 内联汇编的限制（见 0005 文档）导致符号证明不可行。**proptest 覆盖足够**，Kani 留给状态机/整数部分。
- **测量更新步骤**：GPS / mag / baro 的 innovation + K-gain 计算。这是 M1 里最复杂的一块，分 3-4 个子步骤做。
