# Progress log / 进度日志

每完成一个里程碑或一次有意义的提交，在这里加一份简短记录。格式：`NNNN-slug.md`，按时间顺序递增编号。这样未来任何人（包括你自己、下一个对话里的 Claude）扫一遍就知道项目到哪了、为什么这么做、下一步该干啥。

| # | 日期 | 标题 | Commit | 摘要 |
|---|------|------|--------|------|
| [0001](./0001-m0-scaffold.md) | 2026-04-20 | M0 工作区脚手架 | `7ed3421` | 12-crate Cargo workspace、Apache-2.0、CLAUDE.md 规约、toolchain 固定到 1.88、docs/plan.md 凝练路线图 |
| [0002](./0002-docker-container.md) | 2026-04-20 | Docker 开发容器 | `0bfef4c` + `e83b695` | Ubuntu 24.04 + Rust 1.88 + ROS 2 Jazzy + probe-rs + gh CLI；docker-compose 三 profile（dev/gpu/sim），校园网下 apt/gh 各种兜底 |
| [0003](./0003-m0.5-verification.md) | 2026-04-20 | M0.5 验证全绿 | `4c23282` | 在容器里跑通 cargo check / clippy -D warnings / test；调教 workspace lints（去掉 pedantic+cargo 组，保留 safety-critical deny） |
| [0004](./0004-quaternion-invariant.md) | 2026-04-20 | M1.0 四元数归一化不变量 | `2a74d09` | `algo-ekf::State::normalize_attitude` + proptest（256 样本） + 3 个边界用例。**项目第一个真正的测试**。 |
| [0005](./0005-kani-harness.md) | 2026-04-20 | M1.1 Kani 形式化证明 | `e421dc2` | 重构 normalize_attitude 让退化检查先于 sqrt；Kani 0.67.0 装好，证明零四元数 100% 被拒绝，1.8 s 通过。**项目第一个形式化证明**。 |

## 写新文档时遵守的模板

```markdown
# NNNN — 标题

**Commit**: `xxxxxxx`  **Date**: YYYY-MM-DD  **Milestone**: MX.Y

## What / 做了什么
一段话 + 代码/文件清单。

## Why / 为什么这么做
决策背景、取舍、拒绝的方案及理由。

## How it's verified / 怎么验证的
具体命令 + 期望输出。

## Follow-ups / 遗留
未完成但有必要标出的事项。
```

简短优先，不要写成小说。三行能说完的就不要写三段。
