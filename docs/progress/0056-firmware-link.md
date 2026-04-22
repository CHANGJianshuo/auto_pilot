# 0056 — 第一个真的 firmware `.elf`

**Commit**: `336ca0a`  **Date**: 2026-04-22  **Milestone**: M7.1

## What / 做了什么

从"lib 能编译到 thumbv7em"前进一步到"**真的能链出可烧录 firmware**"。产物：`target/thumbv7em-none-eabihf/release/firmware`，**132 KiB ELF**，STM32H753 flash 布局，cortex-m-rt 启动代码接进来了。

改动：

1. `crates/app-copter/memory.x` —— STM32H753 (Pixhawk 6X) 内存布局：2 MiB flash @ 0x0800_0000，512 KiB AXI SRAM @ 0x2400_0000。
2. `crates/app-copter/build.rs` —— `include_bytes!("memory.x")` 写入 `OUT_DIR`，告诉 rustc 搜该目录。**不发 `-Tlink.x`**（cortex-m-rt 已发，重发会让 memory.x 被 linker 解析两次 → "region 'FLASH' already defined"）。
3. `crates/app-copter/src/bin/firmware.rs` —— 第二个 binary target：
   - `#![no_std] + #![no_main]` 只在 `cfg(target_arch = "arm", target_os = "none")` 启用；
   - Entry 只做 "new FlightState → 一步 rate_loop_step → 永远 nop loop"；
   - Host check 有 stub `fn main()` 打条 "embedded-only"，所以 `cargo check -p app-copter --bin firmware` 在 Linux 上也不红。
4. workspace `Cargo.toml` —— 加 `cortex-m = "0.7"`、`cortex-m-rt = "0.7"`、`panic-halt = "0.2"`。
5. `crates/app-copter/Cargo.toml` —— `[target.'cfg(all(target_arch = "arm", target_os = "none"))'.dependencies]` 只在交叉编译时拉这三个依赖，host 构建零污染。
6. CI workflow —— build-firmware 现在不只 `cargo build --lib`，还 `cargo build --bin firmware --release`，真链 ELF 捕获 "少 symbol / region 溢出" 这类 linker-only 回归。

## Why / 为什么这么做

### 为什么 STM32H753 / Pixhawk 6X

候选：
- **STM32F765**（Pixhawk 5）：老但成熟，216 MHz；
- **STM32H743 / H753**（Pixhawk 6X）：480 MHz Cortex-M7，双 precision FPU，生态最新；
- **iMX RT1176**（Teensy 4 / Auterion）：1 GHz M7 + M4 协处理，更强但 memory map 更复杂；
- **RP2040**（Raspberry Pi）：便宜，M0+ 没 FPU —— 跑不动 EKF。

H753 = **当前 Pixhawk 主流 + FPU + 480MHz + ST 支持稳定 + embassy-stm32 已有 feature**。生态最成熟（M7.2 要用 embassy HAL 驱动，已经有 `stm32h753ii` feature）。

### 为什么 AXI SRAM 不是 DTCM

STM32H753 内存分布：
- ITCM 64 KB @ 0x0000_0000 —— 零等待，IRQ 热路径
- DTCM 128 KB @ 0x2000_0000 —— 零等待，core-coupled data
- AXI SRAM 512 KB @ 0x2400_0000 —— 主 SRAM，经 AXI bus
- SRAM1-4, 总共 ~290 KB —— peripheral DMA 专用
- Backup SRAM 4 KB —— VBAT 持续

最简设定：**所有东西（stack/data/bss/heap）放 AXI SRAM**。
- 512 KB 够了：EKF 24×24 covariance = 2.3 KB，setpoint/state 各 <200 B；
- 不需要复杂 section 分配；
- 按照 CLAUDE.md "EKF 用 f32 为主" 原则，也不用 FPU 的高精度模式；
- 将来可以加 `.ekf_data ITCM` section，把内环关键数据搬过去。

### 为什么 `#![cfg_attr(...)]` gate 而不是两份 `firmware.rs`

两套文件更"清晰"但难维护：改了 entry 要改两份。用 cfg gate 一份代码既可：
- 真正 build for thumbv7em → no_std + no_main + cortex-m entry；
- Host `cargo check` → 普通 std + stub main。

这样 `cargo check --workspace`（host）对 firmware bin 友好，CI 能快速验证一般性语法；`cargo build -p app-copter --bin firmware --target thumbv7em` 才真干活。

### 为什么 `-Tlink.x` 冲突是真坑

cortex-m-rt 的 build.rs 里：
```rust
println!("cargo:rustc-link-arg=-Tlink.x");
```

我的 build.rs 也写了一样的 → 链接命令变成 `rust-lld ... -Tlink.x -Tlink.x`。LLD 按顺序执行每个 script，cortex-m-rt 的 `link.x` 文件内部用 `INCLUDE memory.x` —— 两次 include → FLASH/RAM 都被定义两次 → linker 报 "region already defined"。

**修复**: 只发 `rustc-link-search={OUT_DIR}`，不重复 `-Tlink.x`。cortex-m-rt 的那一条会从 OUT_DIR 找到我们的 memory.x 完成链接。

### 为什么 132 KB ELF 这么小

- release 模式 `opt-level = 3` + dead code elimination；
- no_std 本身 ~30 KB；
- cortex-m-rt 启动代码 ~1 KB；
- cortex-m vectors + ISR stubs ~4 KB；
- EKF + INDI + control alloc + app logic：~50 KB；
- nalgebra + libm 导入的数学子集：~40 KB；
- debug info 放在 release 里：~5 KB。

对比: PX4 最简 airframe 固件 ~1-2 MB（更多功能）、Betaflight ~600 KB。**132 KB 是因为我们还没实现驱动**（M7.2 会涨到估计 300-500 KB），但架构决定量级比 C 飞控小。

### 为什么 build.rs 用 `Result<(), Box<dyn Error>>` + `?`

Workspace lint 禁止 `.expect()` / `.unwrap()`。build.rs 传统写法是 unwrap 一把梭，但我们把安全规约贯彻到构建脚本：**即使一次性代码也走 `Result` 通路**。

好处：
- 错误信息带 source，失败时知道是 `OUT_DIR not set` 还是 `memory.x missing`；
- clippy 不用 allow 任何东西，规约在 workspace 级别统一。

### 为什么 firmware entry 里用 `cortex_m::asm::nop()` 而不是 `loop {}`

空 `loop {}` rustc 会 unwind-fold 成"wait for interrupt"——在 M7 上是 `wfi`，进入低功耗。但我们当前不想低功耗，想让代码真跑（M7.2 加 timer 后才该 sleep）。`nop()` = 实际执行一条指令。

对调试更友好：probe-rs 连上能看见 PC 在 loop 里跳，而不是停在 `wfi`。

## How it's verified / 怎么验证的

```bash
$ cargo build -p app-copter --bin firmware --target thumbv7em-none-eabihf --release
Finished `release` profile [optimized] target(s) in 8.59s

$ ls -la target/thumbv7em-none-eabihf/release/firmware
-rwxr-xr-x 1 dev dev 132756 Apr 22 08:41 firmware

$ cargo test --workspace
182 passed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished, clean

$ cargo fmt --check
Clean
```

**尚未验证（需要真硬件）**:
- `probe-rs run --chip STM32H753ZITx firmware` 真烧进板子
- 上电后 LED 能亮（还没写 GPIO 驱动）
- rate_loop_step 在 MCU 上 WCET 是否 < 500 µs（CLAUDE.md 要求）

这些都等 M7.2 硬件 HAL 驱动到位后一起验。

## Follow-ups / 遗留

- **M7.2 embassy-stm32 驱动**：GPIO 推挽、ICM-42688 SPI 读、UART MAVLink 出口、PWM DShot600 ESC；entry fn 换成 `#[embassy_executor::main]`。
- **真机 WCET 测量**：用 `DWT::CYCCNT` 自测 `rate_loop_step` 的周期数，对比 CLAUDE.md budget。
- **ITCM/DTCM 优化**：把控制环热路径代码 / EKF 矩阵迁到 core-coupled memory，减少 AXI bus 竞争。
- **probe-rs runner**：`.cargo/config.toml` 里加 `runner = "probe-rs run --chip STM32H753ZITx"`，`cargo run -p app-copter --bin firmware --target thumbv7em-none-eabihf` 就直接烧+log。
- **defmt-rtt log**：换 `panic-halt` 为 `panic-probe`，log 通过 SWD 出来，在 probe-rs console 里看。
- **link-time memory analysis**：`cargo-size` + `cargo-bloat` 看 section 占用，防止悄悄超过 2 MiB。
- **更多 target triple**：thumbv8m.main-none-eabihf（M33，iMX RT1176）同样 smoke-test 一遍。
