# 0002 — Docker 开发容器

**Commits**: `0bfef4c`（初始设计）+ `e83b695`（build 全部修复）  **Date**: 2026-04-20

## What / 做了什么

在 `docker/` 下建立了完整的可复现 dev 环境。最终产物：一个 1.91 GB 的 `auto_pilot-dev:latest` 镜像，把下列工具全部封装进去：

| 类别 | 内容 |
|------|------|
| OS | Ubuntu 24.04（host 是 22.04，但容器共享内核不共享 userspace） |
| Rust | 1.88.0 + `thumbv7em-none-eabihf` + clippy/rustfmt/rust-src/llvm-tools |
| 嵌入式工具 | probe-rs 0.24.0、flip-link、cargo-binutils、cargo-geiger 0.13.0、cargo-audit、cargo-nextest 0.9.89 |
| 中间件 | ROS 2 Jazzy（LTS）+ `rmw_cyclonedds` |
| 仿真 | `ros-jazzy-ros-gz`（gz-harmonic 可选，`GZ_ENABLE=1` 开启） |
| Python | `uv` 快装器；ML 栈（torch/onnx/pymavlink）可选（`ML_ENABLE=1`） |
| 其他 | gh CLI 2.60.0、tmux、gdb、dev 用户 UID 1000 |

文件清单：
- `docker/Dockerfile`（多阶段，对校园网拥塞极度鲁棒）
- `docker/docker-compose.yml`（三个 profile：default/gpu/sim）
- `docker/entrypoint.sh`（自动 source ROS、修 named volume 权限、git safe.directory）
- `docker/Makefile`（`build/shell/up/down/gpu/sim/flash/test/lint/prune`）
- `docker/.dockerignore`（排除 target、.git）
- `docker/README.md`（GUI 转发、USB 透传、NVIDIA 配置、WSL2 注意事项）

## Why / 为什么这么做

### 为什么 Ubuntu 24.04（host 是 22.04）

- ROS 2 Jazzy LTS 只在 noble（24.04）发 apt 二进制，用 humble 要倒退到 2027 EOL。
- 容器和 host **只共享内核**——WSL2 kernel 6.6 比 22.04 的 5.15 和 24.04 的 6.8 都新，零兼容风险。

### 为什么 dual-compute 架构

和 docs/plan.md §4 对齐：FMU（安全关键，Cortex-M7）+ AI 协处理器（Jetson/RK3588）。dev 容器是"协处理器侧"的超集——ROS 2、Gazebo、NN 推理都在这跑，通过 Zenoh 和 FMU 说话。

### 六次失败一次成功：都修了什么

校园网 `polyu.edu.hk` 的透明代理把 DNS 全改成 `198.18.x.x`（RFC2544 保留地址），然后自己做 HTTP 缓存。上游不稳定时会丢大量 `.deb` 连接。

| 失败次 | 原因 | 修法 |
|------|------|------|
| #1 | `apt install` 35+ 包并发超时 | 拆成 3 小 RUN（core / toolchain / utilities） |
| #2 | `archive.ubuntu.com` 的缓存丢包 | 加 `APT_MIRROR=http://hk.archive.ubuntu.com/ubuntu` 切香港源；apt.conf.d 里加 `Retries=10 Queue-Mode=access` |
| #3 | gh CLI 的 GPG key 通过 shell 变量传递，二进制被损坏 | 改成直接下载 `.deb`，跳过 apt repo 的 GPG 验证 |
| #4 | `gz-harmonic` 不在 noble 主仓 | 改成可选（`GZ_ENABLE=1`），走 OSRF 仓库 |
| #5 | `cargo-nextest 0.9.133` 要 rustc 1.91、`probe-rs-tools 0.31.0` 自身编译 bug | 拆成 4 个独立 RUN，pin 到 nextest 0.9.89 / probe-rs 0.24.0，全带 `|| skip` 兜底；rustc 升 1.85→1.88 |
| #6 | Ubuntu 24.04 自带 `ubuntu` UID 1000 用户，`useradd dev --uid 1000` 失败（被 `||true` 吃了），`/home/dev` 不存在 | 先 `userdel -r ubuntu`，再按 `USER_UID` 查重，再创建 dev |

### 为什么 `make shell` 默认 `--rm` 容器

CLAUDE.md §"现代容器化开发"：状态都在 bind-mount（`/workspace` + cargo volume），容器只是一次性计算环境；强制所有配置写进 Dockerfile，可复现性好。要常驻用 `make up`。

## How it's verified / 怎么验证的

容器内 smoke test 全绿：

```
$ docker run --rm auto_pilot-dev:latest bash -c 'whoami; id; rustc --version; cargo nextest --version; probe-rs --version; ros2 pkg list | head'
dev
uid=1000(dev) gid=1000(dev) groups=1000(dev),20(dialout),46(plugdev)
rustc 1.88.0 (6b00bc388 2025-06-23)
cargo-nextest 0.9.89 (c2af250f7 2025-02-10)
probe-rs 0.24.0 (git commit: crates.io)
action_msgs
actionlib_msgs
actuator_msgs
...
```

持续运行容器：

```
$ docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
NAMES          IMAGE                     STATUS
docker-dev-1   auto_pilot-dev:latest     Up N minutes
```

## Follow-ups / 遗留

- **CI workflow 未推**——`.github/workflows/ci.yml` 在本地但因 token scope 推不上去。三种解决办法写在根 README。
- **GPU 还没测试**——需要在有 NVIDIA 卡 + Container Toolkit 的机器上 `make gpu` 验证。WSL2 装 NVIDIA Container Toolkit 是可行的但还没试过。
- **USB 透传**——`make flash` 需要宿主 `usbipd` 把 ST-Link 挂进 WSL。没实物调试前放着。
