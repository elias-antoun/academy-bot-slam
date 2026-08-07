#!/usr/bin/env bash
#
# run_ros_container.sh
# -----------------------------------------------------------------------------
# Starts an interactive container for the SLAM track with:
#   * the ros2_ws/ source tree bind-mounted at /ros2_ws  (edit on the host,
#     build inside the container — your work survives container restarts)
#   * X11 forwarding so RViz2 and Gazebo GUIs appear on your desktop
#   * host networking so ROS 2 / Gazebo discovery "just works"
#
# Usage:
#   ./run_ros_container.sh           # start (or attach to) the container
#   ./run_ros_container.sh build     # one-shot: colcon build the workspace
#
# -----------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_DIR}/ros2_ws"

IMAGE_TAG="${IMAGE_TAG:-acadbot:jazzy}"
CONTAINER_NAME="${CONTAINER_NAME:-acadbot}"

# Allow local X clients (RViz2 / Gazebo GUI). Reverted is optional; harmless.
if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
fi

# GPU passthrough if nvidia-container-toolkit is present (optional, speeds up Gazebo).
GPU_ARGS=()
if docker info 2>/dev/null | grep -qi nvidia; then
    GPU_ARGS=(--gpus all)
fi

# Direct Rendering Interface. Without /dev/dri the container's Mesa cannot reach
# any GPU and RViz2/Gazebo die with "failed to load driver: iris" /
# "glx: failed to create dri3 screen". On a hybrid-graphics laptop the X server
# usually renders on the Intel iGPU, which --gpus all does *not* cover.
# logind puts an ACL for the seat user (uid 1000) on these nodes and the
# container's 'ros' user is also uid 1000, so no --group-add is needed.
if [ -d /dev/dri ]; then
    GPU_ARGS+=(--device /dev/dri)
fi

# If a container is already running, just attach a new shell to it.
if [ "$(docker ps -q -f name="^${CONTAINER_NAME}$")" ]; then
    echo "==> Attaching to running container '${CONTAINER_NAME}'"
    exec docker exec -it "${CONTAINER_NAME}" /bin/bash
fi

# Remove a stopped container of the same name.
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

RUN_CMD=(/bin/bash)
if [ "${1:-}" == "build" ]; then
    RUN_CMD=(/bin/bash -lc "cd /ros2_ws && colcon build --symlink-install && echo 'Build complete.'")
fi

echo "==> Launching '${CONTAINER_NAME}' from image '${IMAGE_TAG}'"
exec docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network host \
    --ipc host \
    "${GPU_ARGS[@]}" \
    --env DISPLAY="${DISPLAY:-:0}" \
    --env QT_X11_NO_MITSHM=1 \
    --env XDG_RUNTIME_DIR=/tmp/runtime-ros \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "${WS_DIR}:/ros2_ws:rw" \
    "${IMAGE_TAG}" \
    "${RUN_CMD[@]}"
