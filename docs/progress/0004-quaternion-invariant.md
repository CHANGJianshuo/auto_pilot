# 0004 — M1.0 四元数归一化不变量（项目第一个真正的测试）

**Commit**: `2a74d09`  **Date**: 2026-04-20  **Milestone**: M1.0

## What / 做了什么

给 `crates/algo-ekf/src/lib.rs` 加了：

- `State::normalize_attitude(&mut self) -> Option<UnitQuaternion<f32>>`
  - 成功：原地归一化 + 返回 `UnitQuaternion`
  - 失败：`‖q‖ < 1e-6` 或含 NaN/Inf 时返回 `None`——信号"姿态丢失，调用方必须 reset filter"
- 两个常量：`QUATERNION_NORM_FLOOR = 1e-6` / `QUATERNION_NORM_TOLERANCE = 1e-6`
- 4 个测试：
  - `normalize_produces_unit_quaternion`（**proptest**，256 随机样本）：任意有限四元数归一化后 `|‖q‖ − 1| ≤ 1e-6`
  - `default_state_has_unit_attitude`：默认 State 是单位姿态
  - `zero_quaternion_returns_none`：零四元数返回 None
  - `nan_quaternion_returns_none`：NaN 分量返回 None

## Why / 为什么这么做

### 为什么从这里开始？

EKF 的首要不变量就是四元数必须保持单位范数。如果积分过程中 `‖q‖` 漂移（浮点累积误差），所有下游计算（旋转矩阵、角速度转换）都会歪。工业级 EKF（PX4 的 ekf2、ArduPilot 的 AP_NavEKF3）每个 predict/update 步骤后都必 renormalize。

写 EKF 之前先把这个合同定死：
- 数值阈值写成常量，测试不变量就是在断言常量。
- 把"姿态丢失"这种灾难情况从"panic"降级成"`Option::None`"，让 FDIR 层可以捕获并切换到备份控制律。

### 为什么用 proptest 而不是几个手写 case

- 四元数可以在任何方向、任何范数下出现；手写几个 case 永远覆盖不全 scaling-sensitive 路径。
- proptest 用 256 随机样本 + shrinking 自动逼近最小反例。一旦我们以后改数值精度（f32 → f64、或引入 Kahan 求和），proptest 会立刻告诉我们不变量是否还成立。
- 为未来的 Kani 证明提供"对照组"：proptest 说"在随机样本上成立"，Kani 证明"在符号执行下对所有合法输入成立"。**两者组合**是 CLAUDE.md 要求的强度。

### 浮点数上限为什么是 `[-1e6, 1e6]`

- 太窄的范围不会触发尺度敏感的 bug（比如归一化前先平方再开根可能溢出）。
- 太宽（> 1e19）会让 `norm()` 直接 `f32::INFINITY`，shrink 出来的反例没意义。
- 1e6 足以覆盖 EKF 实际中可能出现的任何四元数状态（正常在 [0, 10]，偶尔因异常几个数量级）。

## How it's verified / 怎么验证的

```
$ docker exec docker-dev-1 bash -c 'cd /workspace && cargo test -p algo-ekf'
running 4 tests
test tests::default_state_has_unit_attitude ... ok
test tests::nan_quaternion_returns_none ... ok
test tests::zero_quaternion_returns_none ... ok
test tests::normalize_produces_unit_quaternion ... ok

test result: ok. 4 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

也跑过 `cargo clippy --workspace --all-targets -- -D warnings` 验证没有退步。

## Follow-ups / 遗留

下一步（0005）：**给同一条不变量写 Kani harness**。proptest 说"对 256 个样本成立"，Kani 要说"对符号执行下的所有合法输入都成立"。放在 `formal/ekf_quaternion.rs`，CI 里用 `kani` crate 跑。

再往后（M1 剩下的）：
- `State` 24 维向量序列化/反序列化（EKF 协方差矩阵需要按索引访问状态）
- `State::predict(imu: ImuSample, dt_s: f32)` 的真 IMU 积分（四元数 ẋ = ½ Ω·x 等）
- ICM-42688-P SPI 驱动（需要 `embassy-stm32` HAL，待 STM32H7 板子通电）
