# 0058 — defmt-rtt log + probe-rs runner

**Commit**: `498f320`  **Date**: 2026-04-22  **Milestone**: M7.3

## What / 做了什么

一键烧录 + 读 log 的工作流：

```bash
cargo run -p app-copter --bin firmware --target thumbv7em-none-eabihf
```

`probe-rs` 把 .elf 烧进 STM32H753，连上 RTT channel，把 firmware 里的 `defmt::info!` log 实时打回终端。

具体：
- `.cargo/config.toml` 新增 `runner = "probe-rs run --chip STM32H753ZITx"` 和 `-C link-arg=-Tdefmt.x`（defmt-rtt 要这个 linker script 来把 interned format strings 放进 probe-rs 认识的 section）。
- firmware.rs：`use {defmt_rtt as _, panic_probe as _};` 把 logger backend 和 panic handler 链进来。main 进来打 `"auto_pilot firmware started"`，heartbeat task 每秒 `defmt::info!("heartbeat {=u32}", n)`。
- workspace Cargo.toml 把 `panic-halt` 从 workspace deps 移除（现在用 panic-probe），embassy-stm32 的 `defmt` feature 重开（M7.2 为避免 `_defmt_acquire` unresolved 临时关的，现在有 backend 了）。
- **Bug fix**：`.cargo/config.toml` 里 `+fp-armv8d16sp` target-feature 是 Armv8-M (M33/M55) 的特性，Cortex-M7 不认——LLVM silent fallback 到 soft-float math，release ELF 肿成 298 KiB。去掉这个 flag → 91 KiB（硬件 FPU 指令正确 emit）。

## Why / 为什么这么做

### 为什么选 probe-rs 不是 OpenOCD

- **纯 Rust**：和我们工具链一致，Rust 1.88 可以直接 `cargo install probe-rs-tools`，没有 C 依赖链；
- **defmt 原生支持**：probe-rs 能直接读 RTT channel 并解 defmt 格式，GDB 要额外脚本；
- **多 probe 兼容**：ST-Link v2/v3、J-Link、DAPLink、Black Magic、CMSIS-DAP——probe-rs 都能走；
- **CI 友好**：GitHub Actions 自托管 runner 接实体板时装 probe-rs 一条命令，OpenOCD + arm-gcc 要装十分钟。

### 为什么 `.cargo/config.toml` 里放 runner，不是做成 xtask

也考虑过 `cargo xtask flash` pattern。但：
- **runner 是 cargo 原生特性**，`cargo run -p <bin> --target ...` 自动调，零学习成本；
- **xtask** 额外层 boilerplate；飞控不是复杂多步构建，一条 runner 够；
- 将来需要 "先 flash 再启动 GDB" 之类复杂流程，再引 xtask，现在保持简单。

### 为什么 `-Tdefmt.x` 必须加

defmt 的核心优化：**格式字符串不放 flash**。
```rust
defmt::info!("heartbeat {=u32}", n);
```
常规 `log::info!` 会把 `"heartbeat {}"` 字符串压进 flash。defmt 把它放进一个特殊 linker section `_defmt_`，probe-rs 从 ELF 的这个 section 读字典，运行时只通过 RTT 传 **数字索引 + 参数**。

`defmt.x` 就是告诉 linker "创建 _defmt_ section 且不放进 LOAD 段"。没有它：
- 运行时 defmt 宏找不到 section，link fail；
- 或者 section 意外被 strip。

cortex-m-rt 的 `link.x` 不知道 defmt（它比 defmt 老），所以要额外加。

### 为什么 `+fp-armv8d16sp` 是个坑

LLVM 的 target feature 命名规则：
- Cortex-M4F / M7F：`+vfp4d16sp`（VFPv4 单精度）
- Cortex-M33F / M55F：`+fp-armv8d16sp`（Armv8-M FPU）

Cortex-M7 是 **Armv7E-M** —— 不是 Armv8-M。强行指定 `+fp-armv8d16sp`：
- rustc 发 "unknown and unstable feature" **warning**（不是 error）；
- LLVM 认为目标**没有 FPU**，所有 `f32 * f32` 退化为 `__mulsf3()` soft-float 调用；
- 每个 multiply ~50 clock cycles vs 硬件 3 cycles —— 速度差 17×；
- 代码体积：每处乘法变成函数调用，ELF 肿 3-4 倍。

`thumbv7em-none-eabihf` 目标三元组里的 `hf` 已经意味着 "hard float + VFPv4 单精度"。**不加任何 `-C target-feature` 就对了**。

这种"silent correctness bug"特别讨厌——编译通过，只是慢 17 倍。M7.2 firmware 里的 1 kHz rate_loop 可能真的撞了 timing budget。M7.7 WCET 测量时要重点看这个的改善。

### 为什么 release ELF 从 298 KB → 91 KB 这么大

298 KB 时：
- 每次 f32 算术 → `__mulsf3`、`__divsf3`、`__addsf3` 等 compiler-rt 函数
- nalgebra 的矩阵运算 inline 展开成千百次这种调用
- defmt format strings 以前也在代码段（因为 -Tdefmt.x 没加，fallback 到普通 rodata）

91 KB 时：
- 硬件 `vmul.f32` / `vdiv.f32` 单指令
- nalgebra 展开成 sequential FPU 指令
- defmt 字符串在 `_defmt_` 非 LOAD 段，不计入 binary size
- 同样用 probe-rs 烧进去的也是 91 KB flash 占用

**真正的代码效率改善是 ~3-4×**（硬件 FPU）。剩下的节省来自 defmt 元数据 offload。

### 为什么 ELF 大小不等于 flash 占用

其实 ELF 里包括 debug info、非 LOAD section、metadata。真正烧进 flash 的是 `.text + .rodata + .vector_table`。用 `cargo size` / `llvm-objdump` 看准确数字。91 KB ELF 里可能只有 70 KB 真进 flash。

M7.4+ 加更多驱动后应该用 `cargo-bloat` 跟踪。

## How it's verified / 怎么验证的

```bash
$ cargo build -p app-copter --bin firmware --target thumbv7em-none-eabihf
Finished (defmt link OK)

$ cargo build -p app-copter --bin firmware --target thumbv7em-none-eabihf --release
Finished, 91 KiB ELF

$ cargo test --workspace
182 passed

$ cargo clippy --workspace --all-targets -- -D warnings
Clean

$ cargo fmt --check
Clean
```

**尚未真机验证**（要 Pixhawk 6X + ST-Link/J-Link）：
- `cargo run --bin firmware --target thumbv7em-none-eabihf` 烧板成功；
- 终端看到 `INFO  auto_pilot firmware started (STM32H753, embassy)`；
- 每秒看到 `INFO  heartbeat 0/1/2/...`；
- panic 时通过 panic-probe 打出 backtrace。

## Follow-ups / 遗留

- **M7.4 真 IMU 驱动**：embassy-stm32 SPI + CS → ICM-42688 驱动 + `core-hal::ImuSource` 实现。rate_loop_task 读真数据。
- **M7.5 UART MAVLink serial**：USART3 DMA 把 HEARTBEAT 发出去，QGC 连 serial 也能看到 vehicle。
- **M7.6 `cargo-size` + `cargo-bloat` 报告进 CI**：release 编译后自动输出"flash used: X KB / 2048 KB"；超阈值 fail。
- **M7.7 WCET in-chip 测量**：`DWT::CYCCNT` 包住 rate_loop_step。每 1000 tick 输出 min/max/avg，对比 CLAUDE.md 500 µs。
- **多 probe chip id 参数化**：目前 `.cargo/config.toml` 硬 pin `STM32H753ZITx`。多板支持用 `probe-rs` `--chip <env>` + Make/just 脚本。
- **GDB support**：某些情况 gdb 更方便（设断点 step）。probe-rs 有 `attach` 模式，加一个 alias。
- **CI 跑 `cargo size`**：现在 CI 只 check 编译，不 check 大小。加进 build-firmware job。
