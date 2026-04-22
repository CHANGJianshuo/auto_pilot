# 0050 — Workspace cargo fmt 统一

**Commit**: `9d986af`  **Date**: 2026-04-22  **Milestone**: M6.0

## What / 做了什么

`cargo fmt --check` 以前在这个 workspace 跑会抛一堆诊断：

```
Diff in /workspace/crates/sim-hil/src/lib.rs at line 420:
-    pub fn step(cfg: &SimConfig, state: &mut SimState, motor_thrusts_cmd_n: &SVector<f32, 4>, dt_s: f32) {
+    pub fn step(
+        cfg: &SimConfig,
...
```

原因：容器里 Rust 1.88 的 rustfmt 比原始代码提交时的版本对 import 排序、行长更严格。

这一步就一件事：`cargo fmt` 一把梭，把 10 个文件的格式打理干净，11 files changed / 472 insertions / 250 deletions。一个文件没有语义改动，全是 whitespace + import reorder。

**零行为改动**：workspace 170 单元测试全绿，clippy 也绿。

## Why / 为什么这么做

### 为什么现在做，不拖到下一轮

- `cargo fmt --check` 是 CI 三件套（fmt/clippy/test）里目前**唯一**没对齐到红线的——历史债越积越重，合并新 PR 时 fmt 冲突一多反而误伤语义审查；
- M5.x 把 MAVLink 栈收尾了，是自然的"换一口气"的断点；
- 没有业务代码修改，低风险，放路上挡着的不适合带进下一轮 feature PR。

### 为什么单独一个 commit，不和 feature 混

从 M5.6 到 M5.8 我反复遇到"一个 feature commit 里混进 200 行 fmt 改动"的困扰——review 时**两种噪声无法区分**：
- 语义改动（新字段、新逻辑分支）
- rustfmt reflow（多/少一个换行）

这次把它单独一个 `style:` commit，从今天起 feature commit 里只应有语义 diff。`git blame` 也更清晰。

### 为什么 Cargo.lock 跟着变

`sim-hil` 的 `Cargo.toml` 在 M5.x 里加了 `comms-mavlink` 和 `mavlink` 的直接依赖（以前只通过 `algo-nmpc` 等间接依赖）。Cargo.lock 的**依赖声明区**记录这种信息（与 dependency 树无关的纯 metadata），所以两行多余插入。这和 fmt 没关系，但是同一批次修好的——一起进。

### 为什么以后每个 commit 都应该通过 fmt --check

以前就"应该"，但历史债没清。CI 能强制，但只有所有人（包括 AI）写每个 patch 时都先跑 `cargo fmt` 才能避免累积。下一步往 `.github/workflows/ci.yml` 里加 `cargo fmt --check`，让违规立刻 fail。

## How it's verified / 怎么验证的

```bash
$ cargo fmt --check
(no output — clean)

$ cargo test --workspace
170 passed, 0 failed

$ cargo clippy --workspace --all-targets -- -D warnings
Finished

$ cargo build -p sim-hil --example sitl_mavlink
Finished
```

全绿。

## Follow-ups / 遗留

- **CI 加 fmt-check**：把这条作为 blocking check 进 GitHub Actions；
- **rustfmt.toml 固化**：显式写 `edition = "2024"` + 关键规则（`imports_granularity`, `group_imports`），免得以后 toolchain bump 再抖；
- **pre-commit hook**：本地 `.git/hooks/pre-commit` 自动跑 `cargo fmt`（或者集中在 `just fmt` 任务里）；
- **Clippy lint profile 写进 workspace 根**：`[workspace.lints.clippy]` 里 deny 的列表。目前散在各 crate。
