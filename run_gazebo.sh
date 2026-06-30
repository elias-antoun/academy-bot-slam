#!/usr/bin/env bash
#
# run_gazebo.sh — build the workspace and spawn AcadBot in Gazebo (with GUI).
# -----------------------------------------------------------------------------
# One command for "show me the robot in Gazebo":
#   1. (re)builds the ROS 2 workspace with colcon
#   2. launches the simulation — Gazebo Harmonic GUI + the robot + ros_gz bridge
#
# Usage:
#   ./run_gazebo.sh                         # build + launch the default sim
#   ./run_gazebo.sh acadbot_bringup mapping.launch.py   # build + a different launch
#
# X11 + GPU passthrough is wired up automatically so the GUI appears on your
# desktop (works on NVIDIA / Optimus laptops via /dev/dri + the video/render
# groups). Press Ctrl-C in this terminal to stop everything.
# -----------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ros2_ws"

IMAGE_TAG="${IMAGE_TAG:-acadbot:jazzy}"
CONTAINER_NAME="${CONTAINER_NAME:-acadbot_gazebo}"

# What to launch once the build is done (override via CLI args).
LAUNCH_ARGS=("$@")
if [ ${#LAUNCH_ARGS[@]} -eq 0 ]; then
    LAUNCH_ARGS=(acadbot_gazebo simulation.launch.py)
fi

# -----------------------------------------------------------------------------
# 0. Make sure the Docker image exists (build it if not).
# -----------------------------------------------------------------------------
if ! docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
    echo "==> Image '${IMAGE_TAG}' not found — building it first..."
    "${SCRIPT_DIR}/docker/build_ros_image.sh"
fi

# -----------------------------------------------------------------------------
# 1. Allow local X clients so the Gazebo GUI can open on this display.
# -----------------------------------------------------------------------------
if command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
fi

# -----------------------------------------------------------------------------
# 2. Assemble GPU / rendering passthrough.
# -----------------------------------------------------------------------------
GPU_ARGS=()
if docker info 2>/dev/null | grep -qi nvidia; then
    GPU_ARGS+=(--gpus all
               -e NVIDIA_DRIVER_CAPABILITIES=all
               -e NVIDIA_VISIBLE_DEVICES=all)
fi
if [ -d /dev/dri ]; then
    GPU_ARGS+=(--device /dev/dri:/dev/dri)
fi
# Give the container access to the GPU device groups (varies per machine).
for grp in video render; do
    gid="$(getent group "${grp}" 2>/dev/null | cut -d: -f3 || true)"
    [ -n "${gid}" ] && GPU_ARGS+=(--group-add "${gid}")
done

# -----------------------------------------------------------------------------
# 3. Clear any previous AcadBot sim containers (avoid two sims clashing).
# -----------------------------------------------------------------------------
docker rm -f "${CONTAINER_NAME}" acadbot_sim >/dev/null 2>&1 || true

# -----------------------------------------------------------------------------
# 4. Build the workspace, then launch — in one interactive container.
# -----------------------------------------------------------------------------
echo "=============================================================="
echo " Workspace : ${WS_DIR}"
echo " Image     : ${IMAGE_TAG}"
echo " Launch    : ${LAUNCH_ARGS[*]}"
echo " Display   : ${DISPLAY:-<unset>}"
echo "=============================================================="

exec docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network host --ipc host \
    "${GPU_ARGS[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-ros \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${WS_DIR}:/ros2_ws" \
    "${IMAGE_TAG}" \
    bash -lc '
        set -e
        source /opt/ros/jazzy/setup.bash
        echo "==> [1/2] Building the workspace (colcon)..."
        cd /ros2_ws
        colcon build --symlink-install
        source install/setup.bash
        echo "==> [2/2] Launching Gazebo: '"${LAUNCH_ARGS[*]}"'"
        ros2 launch '"${LAUNCH_ARGS[*]}"'
    '
