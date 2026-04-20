#!/usr/bin/env bash
# auto_pilot dev container entrypoint.
#
# Sources ROS (if present), configures git safe.directory for the bind-mounted
# workspace, and hands off to the requested command.

set -eo pipefail   # NOTE: NOT -u — ROS setup.bash references unbound vars.

# --- ROS 2 ----------------------------------------------------------------
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # ROS scripts assume these exist; keep them empty so `-u` callers downstream
    # don't trip either.
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-}"
    export COLCON_TRACE="${COLCON_TRACE:-}"
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
