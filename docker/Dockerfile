# syntax=docker/dockerfile:1.7
# ---------------------------------------------------------------------------
# auto_pilot dev container
#
# Layers are ordered so that slow-moving system packages are cached and only
# the fast-moving application layers rebuild when you edit source.
#
#   1. base system + apt packages
#   2. ROS 2 Jazzy + Gazebo Harmonic (optional, gated by ROS_ENABLE)
#   3. Rust toolchain (pinned via rust-toolchain.toml in the repo)
#   4. Embedded tools (probe-rs, flip-link, cargo-binutils)
#   5. Python / uv + ML stack (optional, gated by ML_ENABLE)
#   6. Dev user, shell, entrypoint
# ---------------------------------------------------------------------------
ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=Etc/UTC

# ---- 1. System packages ---------------------------------------------------
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl wget gnupg lsb-release sudo \
        build-essential cmake ninja-build pkg-config \
        git git-lfs openssh-client \
        gdb lldb strace ltrace \
        libssl-dev libudev-dev libusb-1.0-0-dev \
        clang lld \
        python3 python3-pip python3-venv python3-dev \
        tmux neovim htop tree file unzip zip jq \
        bash-completion locales tzdata \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# GitHub CLI (for `gh auth login` inside the container)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    mkdir -p -m 755 /etc/apt/keyrings \
    && out=$(wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg) \
    && echo "$out" | tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
    && chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
        > /etc/apt/sources.list.d/github-cli.list \
    && apt-get update && apt-get install -y --no-install-recommends gh \
    && rm -rf /var/lib/apt/lists/*

# ---- 2. ROS 2 Jazzy + Gazebo Harmonic (optional) --------------------------
ARG ROS_ENABLE=1
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    if [ "$ROS_ENABLE" = "1" ]; then \
        add-apt-repository() { \
            apt-get install -y --no-install-recommends software-properties-common; \
            command add-apt-repository "$@"; \
        }; \
        apt-get update && apt-get install -y --no-install-recommends software-properties-common && \
        add-apt-repository universe && \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
              http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
            > /etc/apt/sources.list.d/ros2.list && \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-ros-base \
            ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
            ros-${ROS_DISTRO}-rmw-zenoh-cpp \
            ros-dev-tools \
            gz-harmonic \
            ros-${ROS_DISTRO}-ros-gz && \
        rm -rf /var/lib/apt/lists/*; \
    fi

# ---- 3. Rust toolchain ----------------------------------------------------
ENV RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:${PATH}

ARG RUST_VERSION=1.85.0
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --no-modify-path --profile minimal --default-toolchain ${RUST_VERSION} && \
    rustup component add rustfmt clippy rust-src llvm-tools-preview && \
    rustup target add thumbv7em-none-eabihf && \
    chmod -R a+rw /opt/rust

# ---- 4. Embedded Rust tooling --------------------------------------------
# probe-rs: flash + debug Cortex-M via CMSIS-DAP / J-Link / ST-Link
# flip-link: stack-overflow-safe linker for embedded Rust
# cargo-binutils: objcopy / objdump / size for Rust
RUN --mount=type=cache,target=/opt/rust/cargo/registry \
    --mount=type=cache,target=/opt/rust/cargo/git \
    cargo install --locked \
        probe-rs-tools@0.24.0 \
        flip-link \
        cargo-binutils \
        cargo-geiger \
        cargo-audit \
        cargo-nextest \
    && chmod -R a+rw /opt/rust

# udev rules so the dev user can access USB debug probes without sudo
RUN mkdir -p /etc/udev/rules.d \
    && probe-rs complete --shell bash > /etc/bash_completion.d/probe-rs 2>/dev/null || true \
    && curl -sSL https://probe.rs/files/69-probe-rs.rules -o /etc/udev/rules.d/69-probe-rs.rules || true

# ---- 5. Python / uv (fast installer) + optional ML stack -----------------
ARG ML_ENABLE=0
RUN curl -LsSf https://astral.sh/uv/install.sh | sh \
    && mv /root/.local/bin/uv /usr/local/bin/uv \
    && mv /root/.local/bin/uvx /usr/local/bin/uvx

# The ML stack is heavy; install lazily unless ML_ENABLE=1 at build time.
# Inside the container you can always run: `uv pip install --system torch ...`
RUN if [ "$ML_ENABLE" = "1" ]; then \
        uv pip install --system --no-cache \
            torch torchvision \
            onnx onnxruntime \
            numpy scipy matplotlib \
            pymavlink dronekit \
            pytest jupyterlab; \
    fi

# ---- 6. Non-root dev user --------------------------------------------------
# UID / GID are passed at build or run time so bind-mounted files keep the
# correct ownership on the host.
ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ARG WORKSPACE=/workspace

RUN groupadd --gid ${USER_GID} ${USER_NAME} 2>/dev/null || true \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} 2>/dev/null || true \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME} \
    && usermod -aG plugdev,dialout ${USER_NAME} 2>/dev/null || true \
    && mkdir -p ${WORKSPACE} \
    && chown -R ${USER_UID}:${USER_GID} ${WORKSPACE} /opt/rust

# Auto-source ROS and set a helpful prompt
RUN if [ -n "${ROS_DISTRO}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then \
        echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc; \
    fi \
    && echo 'export PS1="\[\033[1;36m\][ap]\[\033[0m\] \w \$ "' >> /home/${USER_NAME}/.bashrc \
    && echo 'export CARGO_HOME=/opt/rust/cargo' >> /home/${USER_NAME}/.bashrc \
    && echo 'export RUSTUP_HOME=/opt/rust/rustup' >> /home/${USER_NAME}/.bashrc \
    && echo 'export PATH=$CARGO_HOME/bin:$PATH' >> /home/${USER_NAME}/.bashrc

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USER_NAME}
WORKDIR ${WORKSPACE}

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
