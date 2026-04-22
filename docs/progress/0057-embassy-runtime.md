# 0057 — Embassy async runtime 上 firmware

**Commit**: `1064645`  **Date**: 2026-04-22  **Milestone**: M7.2

## What / 做了什么

从"cortex-m-rt busy-loop firmware"升级到"**真正的异步 runtime** 跑多任务"：

- 替换 `#[cortex_m_rt::entry]` 为 `#[embassy_executor::main]`
- `embassy_stm32::init(Config::default())` 配 PLL/时钟/TIM2 time driver
- **两个并发 task**：
  - `rate_loop_task` — 1 kHz Ticker，每 tick 跑一次 `rate_loop_step`（EKF + LPF + INDI + allocation 全栈）
  - `heartbeat_task` — 1 Hz Ticker，占位将来的 defmt log / GPIO blink
- workspace deps: `embassy-executor`, `embassy-time`, `embassy-stm32`（有 `stm32h753ii` + `time-driver-tim2`）、`cortex-m` 加 `critical-section-single-core` 特性

Release ELF 从 132 KiB 涨到 **298 KiB**（embassy runtime + stm32-metapac 占 ~170 KiB，合理）。仍远低于 H7 的 2 MiB flash。

## Why / 为什么这么做

### 为什么 embassy 而不是 RTIC / Hubris

M0 plan.md 里我把 Hubris 和 embassy 并列。现阶段选 embassy 的理由：

1. **生态成熟度**：embassy-stm32 的 `stm32h753ii` HAL 已生产就绪（SPI / UART / PWM / DMA 都有），Hubris 没有等价 STM32 HAL 集合；
2. **`async fn` 对嵌入式友好**：飞控的大部分逻辑是 "等传感器到位 → 处理"，天然 async；
3. **小团队友好**：embassy 的任务模型只需 `#[embassy_executor::task]` 一个属性，Hubris 要写 IDL + manifest；
4. **可替换**：embassy 不侵入业务代码（`algo-*` / `app-copter` lib 都没碰 embassy 类型）。以后真需要 ARINC 653 风格隔离，switch 到 Hubris 只改 firmware.rs。

### 为什么 `critical-section-single-core` 必须开

`embassy-sync` / `embassy-executor` 用 `critical_section` crate 来保护共享 state。`critical_section` 本身是 facade —— 需要某处提供实现：

```
_critical_section_1_0_acquire / _release
```

嵌入式 Cortex-M 有两种实现：
- `cortex-m` crate 的 `critical-section-single-core` 特性（关中断）
- `critical-section` crate 的 `std` feature（host 用，不适用）

单核 MCU 选 `cortex-m` 那个即可。不开 feature → 符号缺失 → link fail。

### 为什么丢掉 `embassy-stm32` 的 `defmt` feature

原 workspace 配置：
```toml
embassy-stm32 = { ... features = ["defmt", ...] }
```

但 workspace 没有任何地方 `use defmt_rtt as _;` 或类似引入 logger。`defmt` crate 不自带 backend —— 需要 `defmt-rtt`（SWD RTT 通道）或 `defmt-serial`（UART 输出）。没有 backend 就：

```
rust-lld: error: undefined symbol: _defmt_acquire
```

两种选择：
- 去掉 `defmt` feature（当前路径）
- 加 `defmt-rtt = "0.4"` 到 firmware deps

暂时去掉，因为 **现在没地方看 log**（没有 probe-rs 连 SWD）。将来真机调试时加 defmt-rtt，重新开 feature。

### 为什么两个 task 而不是一个

一个 task 直接 `loop { rate_loop_step(); ticker.next().await; }` 就够了。加第二个 heartbeat_task 是**故意**的：

- **验证 executor 多任务调度真工作**：如果 rate_loop 占用 100% CPU，heartbeat 永远不 wake up —— 这就是 starvation bug，必须提前暴露；
- **尺寸 smoke test**：第二个 task 的存储开销在 embassy 里是固定的（一个 state + future），验证内存分配；
- **M7.3 预热**：UART / SPI / IMU 驱动都会是独立 task，多任务布局此时就该落下来。

### 为什么 1 kHz rate loop 在 `Ticker::every` 下合理

embassy_time 的 TIM2 驱动是 **硬件 timer tick**，不是软件计数。`Ticker::every(Duration::from_hz(1000))`:
- TIM2 每 1 ms 产生中断；
- 中断把 rate_loop_task 标记 ready；
- executor 恢复执行。

延迟抖动：
- 最优 case ~1 µs（中断到恢复）；
- Worst case 主要被更高优先级任务 preempt。我们只有两个任务，rate loop 是默认优先级，1 Hz 不会 preempt；
- CLAUDE.md WCET budget 500 µs / 内环 ms —— 500µs + 1µs 调度 = 501µs，远低于下一 tick。

实机 WCET 测量（用 `cortex_m::peripheral::DWT`）留给 M7.3。

### 为什么 `embassy_stm32::init(Config::default())` 现在够了

默认 config：
- PLL 启动（但不超频 —— 用 HSI 或默认外部）；
- AHB/APB 分频默认；
- 所有 peripheral clocks 默认关；
- TIM2 启用作为 time driver；
- 不初始化 GPIO / SPI / UART —— 这些要自己开。

H7 最大频率 480 MHz，默认可能在 64 MHz（HSI）。对现在"跑空 rate loop 证明 runtime 工作"已经够；M7.3 要超频就 override `Config::rcc`。

### 为什么 ELF 膨胀到 298 KB 是 OK 的

拆解：
- cortex-m-rt + core + libm: ~30 KB
- app_copter + algo_* 控制栈: ~80 KB
- nalgebra 矩阵运算: ~30 KB
- **embassy-executor + embassy-stm32 HAL**: ~150 KB
- **stm32-metapac SVD 导出**: 压到 ~5 KB（大部分被 LTO 剪了）

300 KB / 2 MiB = 15% 占用。M7.3 加 SPI/UART/PWM 驱动预计再加 ~200 KB。还是很富余。

真关注的是：
- **Flash**：远低于 2 MiB，放心；
- **RAM**：stack 默认 64 KB（cortex-m-rt 默认值），embassy task stacks 是 async future 的 sizeof()，编译期就能算出。M7.3 测；
- **ELF 里的调试 section**：`cargo size` 看具体。

### 为什么不直接加 UART TX

本来想加：UART3（Pixhawk 6X TELEM1 or TELEM2）每秒发 "alive" 字节。但：

- 没在真机上测 → 选错 pin 也发现不了；
- CI 无法验证 UART 行为；
- M7.3 要系统性接 MAVLink UART（能直接出 HEARTBEAT），中间加临时 UART 反倒要改两次；

保留 UART 到 M7.3 一次做对。

## How it's verified / 怎么验证的

```bash
$ cargo build -p app-copter --bin firmware --target thumbv7em-none-eabihf
Finished  (embassy link OK)

$ cargo build -p app-copter --bin firmware --target thumbv7em-none-eabihf --release
Finished, 298 KiB ELF

$ cargo test --workspace
182 passed

$ cargo clippy --workspace --all-targets -- -D warnings
Clean

$ cargo fmt --check
Clean
```

**尚未验证**（要真硬件 or QEMU）：
- 任务真的在跑（不是编译时 DCE 后空壳）；
- 1 kHz ticker 精度；
- multi-task 调度没 starvation。

## Follow-ups / 遗留

- **M7.3 defmt-rtt log + probe-rs runner**：`.cargo/config.toml` 加 `runner = "probe-rs run --chip STM32H753ZITx"`，加 defmt-rtt，log 出 SWD。
- **M7.4 真 IMU 驱动**：embassy-stm32 SPI + `core-hal::ImuSource` 实现 ICM-42688 读取。这一步之后 firmware 是**真能用**不只是"编译过"。
- **M7.5 UART / MAVLink serial**：Pixhawk TELEM1 pinout → USART2，发 HEARTBEAT 到地面站，QGC 能看见。
- **M7.6 DShot600 PWM**：`embassy-stm32::timer` 做 DShot。
- **M7.7 WCET 测量**：`DWT::CYCCNT` 包住 `rate_loop_step`，验证 ≤ 500 µs。
- **RCC 超频到 480 MHz**：`Config::rcc` 手动配 HSE + PLL。
- **ITCM 迁移控制环代码**：用 `#[link_section = ".itcm"]` 搬 hot path 到零等待 SRAM。
- **完整 embassy task 优先级层级**：当前两个 task 同优先级，M7.4 后要定 priority 实施 "INDI > attitude > EKF > MAVLink > heartbeat"。
