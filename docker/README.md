# auto_pilot dev container

A batteries-included development environment. Everything you need to build
firmware, run simulations, flash hardware, and train neural networks lives
inside one image so the host OS stays clean and every contributor gets the
same toolchain.

## What's inside

| Layer | Contents |
|-------|----------|
| OS | Ubuntu 24.04 |
| Rust | 1.85 (pinned by `rust-toolchain.toml`) + `thumbv7em-none-eabihf` target, `clippy`, `rustfmt`, `rust-src`, `llvm-tools` |
| Embedded | `probe-rs`, `flip-link`, `cargo-binutils`, `cargo-geiger`, `cargo-audit`, `cargo-nextest` |
| Verification | `kani-verifier` (symbolic execution + CBMC backend, pinned via `KANI_ENABLE=1`) |
| Middleware | ROS 2 Jazzy (LTS) + `rmw_cyclonedds`, `rmw_zenoh` |
| Simulation | Gazebo Harmonic (`gz-harmonic`), `ros-gz` bridge |
| Python | `uv` fast installer; optional ML stack (`torch`, `onnx`, `pymavlink`) |
| Dev UX | `gh` CLI, `git`, `tmux`, `neovim`, `bash-completion`, non-root `dev` user with passwordless sudo |

## Quick start

```bash
# From the repo root
make -C docker build            # ~15 min first time
make -C docker shell            # drops you into /workspace with the source
# inside the container:
cargo check --workspace
cargo test --workspace
```

The workspace is **bind-mounted**, so every edit you make on the host shows
up instantly in the container (and vice versa).

## Profiles

| Profile | Command | When |
|---------|---------|------|
| default | `make shell` | everyday firmware / Rust work |
| `gpu` | `make gpu` | PyTorch / Isaac Gym / RL training |
| `sim` | `make sim` | Gazebo + QGroundControl, MAVLink UDP 14550 forwarded |

## GPU (NVIDIA)

Install the NVIDIA Container Toolkit on the host first:

```bash
# Ubuntu / WSL2-Ubuntu
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Then:

```bash
make -C docker build-gpu
make -C docker gpu
# inside the container:
python3 -c "import torch; print(torch.cuda.is_available())"
```

## GUI apps (Gazebo, QGroundControl, RViz)

### Linux (native)

```bash
xhost +local:docker    # one-time per session
make -C docker sim
# inside:
gz sim -v 4 shapes.sdf
```

### WSL2

GUI works out of the box via WSLg — `$DISPLAY` is auto-forwarded. Just
`make sim` and launch apps normally.

### macOS

Install [XQuartz](https://www.xquartz.org/), enable `Allow connections from
network clients`, then:

```bash
export DISPLAY=host.docker.internal:0
xhost +localhost
make -C docker shell
```

## USB device passthrough (flashing firmware)

probe-rs talks to ST-Link / J-Link / CMSIS-DAP over USB. The device must be
exposed to the container.

### Linux

Add to `docker-compose.yml` under the `dev` service:

```yaml
devices:
  - "/dev/bus/usb:/dev/bus/usb"
```

Then:

```bash
make -C docker flash
```

### WSL2

WSL doesn't see host USB devices by default. Install
[`usbipd`](https://github.com/dorssel/usbipd-win) on the Windows side, then:

```powershell
usbipd list
usbipd bind --busid <id>
usbipd attach --wsl --busid <id>
```

After that the device appears in `/dev/bus/usb/` inside WSL and can be
passed into the container.

## Persistent caches

Cargo's registry and git caches are stored in named Docker volumes
(`cargo-registry`, `cargo-git`) so `cargo build` is fast on repeat runs.
Wipe with `make prune` if they get corrupted.

## Updating the image

1. Edit `docker/Dockerfile` or `docker/docker-compose.yml`.
2. `make -C docker rebuild` (pass `--no-cache`).
3. Commit both files + this README so teammates pick up the change.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `permission denied` on bind-mounted files | Pass `UID=$(id -u) GID=$(id -g)` — the Makefile does this automatically |
| `could not connect to display` | `xhost +local:docker` (Linux) or verify WSLg is running |
| `probe-rs: no probe found` | On WSL, attach the USB device with `usbipd`; on Linux, ensure udev rules reloaded |
| `torch.cuda.is_available()` returns False | NVIDIA Container Toolkit not configured; re-run `nvidia-ctk runtime configure` |
| Ports in use (14550, 8765) | `make down` then `make up`, or change the mapping in `docker-compose.yml` |
