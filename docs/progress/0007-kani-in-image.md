# 0007 — 把 Kani 装进 Dockerfile

**Commit**: `f371d3c`  **Date**: 2026-04-20  **Milestone**: M1.3（基础设施硬化）

## What / 做了什么

在 `docker/Dockerfile` 加了两步，把 Kani 固化进镜像：

1. **Section 4 末尾**（root 阶段）：`cargo install --locked kani-verifier`
   - 和其他 cargo 工具一样，装进 `/opt/rust/cargo/bin/`
   - 用 `KANI_ENABLE=1` build arg 默认启用，可以 `--build-arg KANI_ENABLE=0` 关掉
   - 失败 `|| echo skipping` 兜底
2. **Section 6b**（USER dev 之后）：`cargo kani setup`
   - 必须作为 dev 用户跑（Kani 把 CBMC + nightly toolchain 存在 `$HOME/.kani/`）
   - 下载 ~2 GB（CBMC 二进制 + nightly-2025-11-21 toolchain）
   - 一样有兜底

## Why / 为什么这么做

### 问题

`0005-kani-harness.md` 里我是**手动在运行中的 container 里装** Kani 的。意味着：
- `make prune` 或 `docker compose down -v` 清理 volume 后，Kani 没了
- 换一台机器 `make build` 重建镜像，新镜像里没 Kani
- 整条 CI 流水线跑 Kani 要每次在 job 里重装（慢）

把 Kani baked into the image 把这些坑堵掉。

### 为什么分两个 RUN 步

| | Section 4（root） | Section 6b（dev） |
|---|---|---|
| 内容 | `cargo install kani-verifier` | `cargo kani setup` |
| 产物去处 | `/opt/rust/cargo/bin/` 全局可执行 | `/home/dev/.kani/` 用户私有 |
| 为什么不能合并 | Kani setup 如果在 root 下跑，产物去 `/root/.kani`，dev 用户看不到 |

Kani 作者选择把下载物放 `$HOME/.kani` 是因为不同用户可能想装不同版本——对我们 single-user dev container 来说这是硬约束，不是特性。

### `KANI_ENABLE` 的意义

CI 如果只想跑 `cargo test`（不跑 Kani），可以 `--build-arg KANI_ENABLE=0` 构建一个更小的 variant 镜像，省 2 GB。我们不做这个优化，但 flag 留着以后用得上。

### 镜像大小代价

预估从 1.91 GB 涨到 ~4 GB。接受这个代价因为：
- Kani 工具链是 self-contained，不会污染宿主
- 每个贡献者第一次 `make build` 就拿到能跑 Kani 的镜像，不用人肉 `cargo kani setup`
- CI job 的镜像预热能摊平这 2 GB

## How it's verified / 怎么验证的

重建镜像后跑一个干净容器，确认 Kani 能立刻工作：

```bash
# Rebuild
$ make -C docker build

# Smoke test — 跑一个 "--rm" throwaway container，没有残留 state
$ docker run --rm auto_pilot-dev:latest bash -c 'cargo kani --version'
# Expect: cargo-kani 0.67.0

# Full regression: 跑之前提交的两个 Kani 证明在新镜像里还过
$ docker run --rm -v /home/chang/auto_pilot:/workspace auto_pilot-dev:latest \
    bash -c 'cd /workspace && cargo kani -p algo-ekf && cargo kani -p core-rtos'
# Expect: algo-ekf 1/1 SUCCESSFUL (1.8s); core-rtos 5/5 SUCCESSFUL (49ms)
```

## Follow-ups / 遗留

- CI workflow（一旦能推）加一个 `kani` job，跑 `cargo kani --workspace`（会自动发现 `#[cfg(kani)]` mod）
- docker/README.md 的工具表加一行 Kani 0.67.0
- 镜像尺寸涨到 ~4 GB 可能让 GitHub Actions runner 的 cache 变慢；后续考虑用 multi-stage build 瘦身（build-time 用 Kani，ship-time 不带）
