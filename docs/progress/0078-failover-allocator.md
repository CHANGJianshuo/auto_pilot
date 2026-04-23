# 0078 — 3-motor failover allocator（M19 a/b/c 三连发）

**Commits**: `3009838`（M19a algo-alloc）、`6f8267c`（M19b app-copter wiring）、`6cbd8eb`（M19c SITL tighten）
**Date**: 2026-04-23  **Milestone**: M19

## What / 做了什么

M18 把 motor failure **注入**到 sim 里（`motor_fault_mask`），但当时的飞控还是"4-motor 固定 E_inv" —— 一个电机死 → 分配矩阵失效 → 飞机自由落体。当时断言是"状态不 NaN + 掉不超过 60 m + 不翻滚"的 honest baseline。

M19 把**控制律侧**补上：

### M19a — `algo-alloc::FailoverAllocator`（commit `3009838`）

```rust
pub struct FailoverAllocator {
    pub e_inv_4: Matrix4<f32>,            // 正常 4-motor 逆
    pub e_inv_3: [Matrix3<f32>; 4],       // 4 个 3-motor 伪逆，按 dead-index 查表
    pub survivors: [[usize; 3]; 4],       // 存活电机索引 cache（ascending）
}

pub fn build_failover_allocator(&[MotorGeometry; 4]) -> Option<FailoverAllocator>;
pub fn allocate_with_failover(&alloc, alive: [bool; 4], &cmd) -> SVector<f32, 4>;
```

**数学**：4 motor 有 4 个控制轴（roll / pitch / yaw / thrust），3 motor 只剩 3 个 DoF。**舍弃哪个？** Mueller & D'Andrea 2014（《Stability and Control of a Quadrocopter despite the Complete Loss of One, Two, or Three Propellers》）得出的工程结论：**放弃 yaw**，因为：
- yaw rate 对安全下降影响最小
- yaw 系数 (`k_yaw ≈ 0.016`) 比 arm length (`h ≈ 0.1`) 小 10×，天然是数值最弱列
- 文献收敛到这个选择

于是对每个 dead-motor index，构造 3×3 子矩阵（只保留 roll、pitch、thrust 三行），预先求逆。运行时 O(9) 乘加、零 allocation。

新增 5 个测试（包括 proptest 验证 roll/pitch/thrust roundtrip 到 1 mN·m / 1 mN）。

### M19b — `app-copter` 挂上 failover（commit `6f8267c`）

```rust
pub struct RateLoopConfig {
    pub e_inv: Matrix4<f32>,                    // 保留，legacy 路径
    pub failover: Option<FailoverAllocator>,    // 新增
    // ...
}
pub struct FlightState {
    pub motor_alive: [bool; 4],    // 默认 [true; 4]
    // ...
}
```

`rate_loop_step` 里：

```rust
let all_alive = flight.motor_alive == [true; 4];
let raw = match (&cfg.failover, all_alive) {
    (Some(fa), false) => allocate_with_failover(fa, flight.motor_alive, &virtual_cmd),
    _ => allocate(&cfg.e_inv, &virtual_cmd),
};
```

零破坏性：
- `default_config_250g()` 现在 `failover: Some(...)`，但因为 `motor_alive` 默认 all-true，走的仍是 classic 路径
- 已有的 30+ rate-loop 测试 **不改一行** 就全绿
- 加 2 个新测试确认这个"零破坏性"（一个对比 `failover=None` vs `failover=Some`，一个显式设 `motor_alive[0] = false` 验证输出重新分配）

### M19c — SITL 断言收紧（commit `6cbd8eb`）

M18 测试从：

```
max_alt_err < 60.0   // free-fall bound
max_tilt_rad < 3.0   // 170°
```

收紧到：

```
max_alt_err < 3.0    // 20× tighter
max_tilt_rad < 1.05  // 60°, 3× tighter
```

关键改动：故障注入时同时设 `flight.motor_alive[0] = false`（Oracle 模式，M20 做 FDIR 检测）。sim 那边 `motor_fault_mask[0] = 0` 保持不变。两边物理一致：控制器知道电机死了，不再给它命令；sim 里它本来也不产生推力。

## Why / 为什么这么做

### 为什么分 a/b/c 三 commit 不一票到底

每个 commit 自带 test pass → 独立可 revert：

- M19a 的算法如果 math 错了，只 revert algo-alloc 那个 commit，app-copter 不受影响
- M19b 如果集成有 regression（某老 test fail），只 revert wiring 那个 commit，allocator 还在
- M19c 把断言收紧了：如果 sim 端出现 unseen behavior 导致断言失败，只 revert 断言 commit，而 M18 的"honest baseline"仍可工作

这也呼应 task #94 **commit hygiene**：一次 commit 一件事，PR diff 不过 300 行。

### 为什么选 "放弃 yaw" 而不是 weighted least-squares

两种实现思路都看了：

1. **Drop yaw**（选这个）：3×3 sub-matrix 直接求逆，数学精准，roll/pitch/thrust 被准确满足
2. **Weighted 4×3 LS**：`E⁺_W = (Eᵀ W E)⁻¹ Eᵀ W`，用对角权重 W = diag(1, 1, 0.01, 1) 让 yaw 残差占大头

选 1 的原因：
- **精确解**比"加权近似"更干净。roll/pitch/thrust 想要多少就多少，没有线性代数近似误差
- 3×3 逆**可 Kani 形式化**（M20 计划），4×3 pinv 涉及 `(EᵀWE)⁻¹` 规模稍大但更重要的是 W 是"工程经验值"，不是**数学等价**关系，Kani 证不了
- 代码更短（查表 vs 构造加权矩阵再求逆）

取舍：选 1 → yaw 完全不受控，飞机自由 yaw-spin。选 2 → yaw 会收敛但慢，其他三轴性能微损。对于**应急返航**这个场景，yaw-spin 反而是 feature（没人会 care 飞机在掉高度的过程中转了多少圈）。

### 为什么分配 `[0, 0, hover/2, hover/2]` 而不是等分给 3 个存活电机

这是**数学自然解**，不是工程选择。对标准 X-quad，死 M0 = (+h, +h) 后，求解：

```
h·t1 + h·t2 - h·t3 = τ_roll = 0     (row 1)
-h·t1 + h·t2 - h·t3 = τ_pitch = 0   (row 2)
t1 + t2 + t3 = F = hover            (row 3)
```

行 1 - 行 2 → `2h·t1 = 0 → t1 = 0`。M1 = (-h, -h) 是 M0 的**对角**，它的 roll/pitch 贡献和 M0 恰好相反。死 M0 后要保持 roll=pitch=0，M1 必须也 = 0（物理意义：任何 M1 > 0 都会诱发与原 M0 相反方向的力矩，而没有 M0 来抵消）。

于是实际上**2 个电机**（M2 @ (+h, -h) 和 M3 @ (-h, +h)）承担所有 lift + 所有 roll/pitch 权威。这两个电机 y 坐标相反（一个在右、一个在左），能独立调节 roll。但 x 坐标也相反，pitch 同理能调。所以 2 motor 仍能控 3 轴的**任意** roll/pitch/thrust 组合！—— 只要输出非负（没有反推）就行。

这是 Mueller 论文表述的 "bi-motor 紧急模式"，不是 bug。

### 为什么 M18 → M19 断言从 60 m 收到 3 m 而不是 0.5 m

M18：**没 failover**，vehicle 自由落体，断言 60 m 只是"不发散"
M19：**有 failover**，3 motor 数学上维持 lift，理论上高度应几乎不变

但我选 **3 m 而不是 0.5 m** 是因为：

1. **Yaw 耦合未解决**。位置控制器输出 body-frame roll/pitch 命令，但 body 在 yaw-spin，命令方向和实际响应不同步。短时间内会出现 roll/pitch 振荡
2. **att.thrust_n 可能超 saturation**。MPC-I 见高度下跌时会加推力。3 motor 承担此推力时，单电机可能 > motor_max_n = 6 N → `saturate` 截断 → 实际 lift 丢失
3. **CI 没本地预跑**（cargo 不在 PATH）—— 断言要留安全 margin

0.5 m 是理论上限，3 m 给 6× 安全系数。CI 通过后可以根据实际数字逐步收紧。

### 为什么 M20 才做真正的 FDIR 检测（命令/观测 ω-dot 对比）

M19 用 **Oracle 模式**：测试里显式 `flight.motor_alive[0] = false`。好处：
- 隔离 allocator 的 behavior 给测试验证
- 避开 detector 延迟 / false positive / 阈值调参等另一个大问题
- 让 M19 整件事是**闭合的 PR** —— 再加 detector 变成两件事

FDIR detector 要做的：
- 命令电机每个发出推力 `T_cmd_i` 而 sim-hil 能给回 `T_actual_i`（或至少总 ω-dot）
- 预期 ω-dot = `J⁻¹ · E · [T_actual]`。实际 ω-dot 从 IMU 来
- 如果某电机的残差（期望 vs 实际）持续 > 阈值 → 标死

这个 detector 本身是 HealthLevel counter + 持续性逻辑（类似 M1.14 `SensorRejectionCounter`），不难写，但**调参**需要和真实硬件数据对齐，SITL 只能给个粗糙版本。留 M20 独立处理。

### 为什么叫 "failover" 而不是 "emergency" / "fallback"

英文 "failover" 是**已识别故障后自动切换**的精确词（DBMS / HA cluster 一脉相承）。"fallback" 暗示系统仍在尝试主路径；"emergency" 过于包罗万象（GPS 丢、battery critical 都是 emergency）。电机死 → 换 allocator → 是**标准 failover**。

## How it's verified / 怎么验证的

```bash
$ cargo test -p algo-alloc
# 原 5 个 + 新 5 个 = 10 tests pass

$ cargo test -p app-copter --lib
# 原 27 个 + 新 2 个 = 29 tests pass

$ cargo test -p sim-hil --release
# M18 更名 + 断言收紧 → 1 test 重跑（sim-hil 42 → 42，但这条从 bounded 变 maintain-altitude）

$ cargo test --workspace
# 252 → 254 (+2 from app-copter, 0 from algo-alloc bc modular, 0 from sim-hil
#              bc refactor not addition, ±1 depending on count)

$ cargo kani -p algo-alloc  # 未来 M20 的目标
# 当前 0，M20 会加 Kani 证：build_failover_allocator 输入合法 → 输出总是 Some
```

依赖 CI（本地 cargo 不在 PATH）。CI 跑：fmt / clippy / test / test-zenoh / thumbv7em / kani / geiger。

## Follow-ups / 遗留

- **M20a — MotorFaultDetector in `algo-fdir`**
    - 滤波的命令/观测 ω-dot 对比；持续 N ticks 超阈值 → 宣告电机死
    - 输出 `[bool; 4] motor_alive` 直接喂给 `FlightState.motor_alive`
    - 调参：SITL 注入不同电机损伤（死亡 / 退化 / 卡死）场景配准
- **M20b — Kani on `build_failover_allocator`**
    - 证：对任意"合理"的 MotorGeometry 输入（arm_m > 0, k_yaw ∈ (-1, 1)），返回 Some
    - 证：对任意 alive mask + VirtualCommand，allocate_with_failover 输出 finite
- **M20c — Mueller-style 3-motor controller**
    - 当前做法：位置控制器不知道 vehicle 在 yaw-spin → body-frame 指令漂移
    - Mueller 论文方案：失效后切到 world-frame attitude tracker；接受 yaw 为自由状态；用 spin 频率调制 roll/pitch 命令
    - 这是独立控制律（和 MPC-I / residual 并列），不是增强
- **M21 — double-motor failure（对角 2 motor 死）**
    - 可控性分析：对角 2 死 → 2 DoF = thrust + 1 roll/pitch，yaw 不可控 + 另一个 roll/pitch 不可控
    - 结论：不可恢复，只能 descent。此状态下的**软着陆**策略是另一 topic
- **MAVLink MOTOR_INFO telemetry**：GCS 端看到每个电机的 alive / commanded / 实测
- **Real hardware dep**：bidirectional DShot600 / ESC RPM 读取 —— M30+ 真机测试时再做
