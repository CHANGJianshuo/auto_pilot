# 0051 — GitHub Actions CI 第一版

**Commit**: `47b1283`  **Date**: 2026-04-22  **Milestone**: M6.1

## What / 做了什么

把本地开发容器里跑的 fmt/clippy/test 三件套迁移成 GitHub Actions workflow：`.github/workflows/ci.yml`，每个 push 到 main 和每个 PR 都触发。

五个 job：

| Job | 命令 | 作用 |
|---|---|---|
| `fmt` | `cargo fmt --all -- --check` | 防 fmt 回归 |
| `clippy` | `cargo clippy --workspace --all-targets -- -D warnings` | lint 全开 |
| `test` | `cargo test --workspace` | 170 单元测试全绿 |
| `build-firmware` | `thumbv7em-none-eabihf` 占位 | 将来 embedded main.rs 上线时激活 |
| `geiger` | `cargo geiger` `continue-on-error` | unsafe 统计（不 block merge，只告警） |

其中 `fmt`、`clippy`、`test`、`build-firmware` 都固定在 **Rust 1.88.0**（与 `rust-toolchain.toml` 对齐），避免 "本地绿 CI 红" 的版本漂移。

## Why / 为什么这么做

### 为什么 1.88.0 硬 pin

`dtolnay/rust-toolchain@stable` 会安装 GitHub Actions runner 跑时**最新的 stable**。问题：
- rustfmt 的 reflow 规则每个 minor version 都可能变；
- Clippy 每个 minor 增加 10-30 个新 lint（有的直接是 `deny` 级）；
- 我本地 Rust 1.88.0，CI 跑 1.90 → **本地绿、CI 红** 但 diff 不是我引入的。

工业实践：**CI 必须锁死工具链**，和开发容器一致。要升级就显式 PR 改 toolchain 文件，一起重跑。

### 为什么 geiger 是 `continue-on-error`

`cargo geiger --all-features` 数 `unsafe` 块。工业目标是 0（除了 HAL 必要的 MMIO/DMA），但：
- 我们当前 HAL crate 还没写实机驱动，geiger 数的全是依赖里的 unsafe；
- 强制 = 0 会让 CI 红得莫名，有噪声；
- 报告型 job 合适：把结果 push 上来供 review，不 block merge。

未来 HAL 成型后可以切换成 `-D` 严格模式。

### 为什么 test 和 clippy 分开 job，不串行

- **并行化**：三个 job 同时跑 → 墙时间 ~3 分钟，而不是 10 分钟；
- **独立失败定位**：单一 job 失败，log 清爽，不必扒一大块；
- **缓存分片**：`Swatinem/rust-cache` 按 job 缓存更有效。

### 为什么没加 MSRV 测试

一般库/命令行工具会加 "Minimum Supported Rust Version" job 测 `cargo check --workspace` 用 MSRV 版本编译。我们不需要——这是飞控固件，不是库。就用 pinned 1.88.0。未来发 binary release 时再补。

### 为什么 `RUSTFLAGS: -D warnings`

Workspace env 里全局开。含义：
- 所有 warning 升级为 error；
- 防止 "warning 累积到看不过来"。

我们本地 `cargo clippy -- -D warnings` 是同样语义；CI 保持一致。

## How it's verified / 怎么验证的

刚 push `47b1283`，GitHub Actions 会在几十秒内跑起来：

```bash
$ gh run list --limit 3
STATUS  TITLE                                               WORKFLOW  BRANCH
*       ci(fmt+clippy+test): GitHub Actions workflow...     CI        main
✓       (previous commit)
```

本地已经验证过所有 job 用到的命令都绿：
```bash
$ cargo fmt --check                       # ok
$ cargo clippy --workspace --all-targets -- -D warnings  # ok
$ cargo test --workspace                   # 170 passed
```

所以推到 main 的第一次 CI 应该 immediately 全绿（fmt/clippy/test 都已本地 pre-verified）。build-firmware 是 echo 占位，geiger 是 continue-on-error，都 OK。

## Follow-ups / 遗留

- **Kani 形式化证明也进 CI**：M1.3 我们把 Kani 装进 Docker 镜像了；但还没做 Action。需要 install cargo-kani + CBMC，CBMC 编译慢，放单独 job + 每日跑（不是每个 PR）。
- **Coverage**：加 `cargo-llvm-cov` job 生成 HTML 报告 + Codecov 徽章。
- **benchmark regression**：Criterion.rs 跑 INDI / EKF 速度基准，记录历史曲线。
- **multi-platform**：macOS / Windows runner 至少 build check（我们有 `tokio`, Rust 应该可移植）。
- **`cargo deny`**：license 合规检查（Apache-2.0 不能依赖 GPL crate）+ security advisory。
- **真正的 embedded build**：把 build-firmware 的 echo 占位换成真的 `cargo build -p app-copter --target thumbv7em-none-eabihf`（需要 app-copter 拆出 `#[cfg(...)]` 嵌入式入口）。
- **Kani proof timeout**：CBMC 跑浮点 15+ 分钟限制，CI runner 60 分钟上限容易撞墙。`--default-unwind`、`--fixpoint` 等参数调优。
