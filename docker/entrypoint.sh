#!/usr/bin/env bash
# auto_pilot dev container entrypoint.
#
# Sources ROS (if present), configures git safe.directory for the bind-mounted
# workspace, and hands off to the requested command.

set -euo pipefail

# --- ROS 2 ----------------------------------------------------------------
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# --- git safe.directory ---------------------------------------------------
# The workspace is bind-mounted from the host; UID mismatches used to make
# git refuse to operate. Mark the workspace as trusted for this user.
if [[ -d /workspace/.git ]]; then
    git config --global --add safe.directory /workspace || true
fi

# --- Pass-through --------------------------------------------------------
if [[ $# -eq 0 ]]; then
    exec bash
else
    exec "$@"
fi
