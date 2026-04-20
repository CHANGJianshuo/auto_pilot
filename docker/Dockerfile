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

# ---- 0. apt resilience ----------------------------------------------------
# Campus networks / transparent proxies frequently drop parallel connections
# when a large apt install is in flight. Make apt patient and serial, and
# optionally swap to a Chinese mirror (only if APT_MIRROR is provided at
# build time, e.g. APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu).
ARG APT_MIRROR=""
RUN set -eux; \
    printf 'Acquire::Retries "10";\nAcquire::http::Timeout "30";\nAcquire::https::Timeout "30";\nAcquire::Queue-Mode "access";\nAcquire::http::Dl-Limit "0";\nAPT::Install-Recommends "0";\nAPT::Install-Suggests "0";\n' \
        > /etc/apt/apt.conf.d/80-ap-retries; \
    if [ -n "$APT_MIRROR" ]; then \
        sed -i "s|http://archive.ubuntu.com/ubuntu|${APT_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${APT_MIRROR}|g" \
            /etc/apt/sources.list /etc/apt/sources.list.d/*.sources 2>/dev/null || true; \
    fi

# ---- 1a. Core packages (small set so retries are cheap) ------------------
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates curl wget gnupg lsb-release sudo \
        locales tzdata bash-completion \
    && locale-gen en_US.UTF-8

# ---- 1b. Build toolchain ---------------------------------------------------
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build pkg-config \
        git git-lfs openssh-client \
        libssl-dev libudev-dev libusb-1.0-0-dev \
        clang lld

# ---- 1c. Debugging + Python + editors (not critical, can retry alone) ----
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        gdb strace \
        python3 python3-pip python3-venv python3-dev \
        tmux htop tree file unzip zip jq \
    && rm -rf /var/lib/apt/lists/*

# GitHub CLI (for `gh auth login` inside the container).
# Install via direct .deb download — avoids the apt repo signature dance that
# fails behind captive/transparent proxies that rewrite binary responses.
RUN set -eux; \
    arch="$(dpkg --print-architecture)"; \
    GH_VERSION="2.60.0"; \
    wget -qO /tmp/gh.deb "https://github.com/cli/cli/releases/download/v${GH_VERSION}/gh_${GH_VERSION}_linux_${arch}.deb" \
        && apt-get install -y /tmp/gh.deb \
        && rm -f /tmp/gh.deb \
        || echo 'gh CLI install skipped (network); install later inside container if needed'

# ---- 2. ROS 2 Jazzy + Gazebo Harmonic (optional) --------------------------
ARG ROS_ENABLE=1
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}

ARG GZ_ENABLE=0
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    set -eux; \
    if [ "$ROS_ENABLE" = "1" ]; then \
        apt-get update && apt-get install -y --no-install-recommends software-properties-common && \
        add-apt-repository -y universe && \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        . /etc/os-release && \
        CODENAME="${UBUNTU_CODENAME}"; \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
            > /etc/apt/sources.list.d/ros2.list && \
        if [ "$GZ_ENABLE" = "1" ]; then \
            curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
                -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${CODENAME} main" \
                > /etc/apt/sources.list.d/gazebo-stable.list; \
        fi && \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-ros-base \
            ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
            ros-dev-tools \
            python3-colcon-common-extensions \
            python3-rosdep && \
        # ros-gz is in the ROS repo; pulls in the Gazebo runtime bits it needs.
        (apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-gz \
            || echo 'ros-gz unavailable, skipping') && \
        # Optional standalone Gazebo Harmonic CLI (requires OSRF repo above).
        if [ "$GZ_ENABLE" = "1" ]; then \
            apt-get install -y --no-install-recommends gz-harmonic \
                || echo 'gz-harmonic install failed, continuing'; \
        fi && \
        # rmw_zenoh is opt-in (not always packaged on release day); ignore if absent.
        (apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-zenoh-cpp || echo 'rmw_zenoh not in apt yet, skipping') && \
        rm -rf /var/lib/apt/lists/*; \
    fi

# ---- 3. Rust toolchain ----------------------------------------------------
ENV RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:${PATH}

ARG RUST_VERSION=1.88.0
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --no-modify-path --profile minimal --default-toolchain ${RUST_VERSION} && \
    rustup component add rustfmt clippy rust-src llvm-tools-preview && \
    rustup target add thumbv7em-none-eabihf && \
    chmod -R a+rw /opt/rust

# ---- 4. Embedded Rust tooling --------------------------------------------
# probe-rs: flash + debug Cortex-M via CMSIS-DAP / J-Link / ST-Link
# flip-link: stack-overflow-safe linker for embedded Rust
# cargo-binutils: objcopy / objdump / size for Rust
#
# Each tool is installed in its own RUN so a single failure doesn't sink the
# whole image. Versions are pinned to avoid MSRV churn (cargo-nextest 0.9.133
# requires rustc 1.91; probe-rs-tools 0.31.0 has an upstream compile bug).
RUN --mount=type=cache,target=/opt/rust/cargo/registry \
    --mount=type=cache,target=/opt/rust/cargo/git \
    cargo install --locked flip-link \
    && cargo install --locked cargo-binutils \
    && chmod -R a+rw /opt/rust

RUN --mount=type=cache,target=/opt/rust/cargo/registry \
    --mount=type=cache,target=/opt/rust/cargo/git \
    (cargo install --locked cargo-audit || echo 'cargo-audit failed, skipping') \
    && (cargo install --locked cargo-geiger || echo 'cargo-geiger failed, skipping') \
    && chmod -R a+rw /opt/rust

# cargo-nextest: pin < 0.9.100 for rustc 1.88 compatibility.
RUN --mount=type=cache,target=/opt/rust/cargo/registry \
    --mount=type=cache,target=/opt/rust/cargo/git \
    (cargo install --locked cargo-nextest --version '^0.9.99' \
        || cargo install --locked cargo-nextest --version '0.9.89' \
        || echo 'cargo-nextest failed, skipping') \
    && chmod -R a+rw /opt/rust

# probe-rs-tools: 0.31.0 has a compile bug; 0.24.0 is the last known-good.
# If install fails, flashing is done by the user on host or later with a
# different version; not blocking for host-side development.
RUN --mount=type=cache,target=/opt/rust/cargo/registry \
    --mount=type=cache,target=/opt/rust/cargo/git \
    (cargo install --locked probe-rs-tools --version '0.24.0' \
        || cargo install --locked probe-rs-tools --version '0.23.0' \
        || echo 'probe-rs-tools failed, skipping — install manually later') \
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

RUN set -eux; \
    # Ubuntu 24.04 ships with a pre-created `ubuntu` user on UID 1000 that
    # collides with the host-UID mapping. Remove it before adding `dev`.
    if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu 2>/dev/null || true; fi; \
    if getent passwd ${USER_UID} >/dev/null; then \
        userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)" 2>/dev/null || true; \
    fi; \
    if getent group ${USER_GID} >/dev/null; then \
        groupdel "$(getent group ${USER_GID} | cut -d: -f1)" 2>/dev/null || true; \
    fi; \
    groupadd --gid ${USER_GID} ${USER_NAME}; \
    useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USER_NAME}; \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}; \
    chmod 0440 /etc/sudoers.d/${USER_NAME}; \
    usermod -aG dialout ${USER_NAME} 2>/dev/null || true; \
    usermod -aG plugdev ${USER_NAME} 2>/dev/null || true; \
    mkdir -p ${WORKSPACE}; \
    chown -R ${USER_UID}:${USER_GID} ${WORKSPACE} /opt/rust

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
