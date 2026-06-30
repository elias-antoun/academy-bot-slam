#!/usr/bin/env bash
#
# build_ros_image.sh
# -----------------------------------------------------------------------------
# Builds the ROS 2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox image used for
# the Robotics Academy SLAM track.
#
#   ./build_ros_image.sh                # build the image
#   IMAGE_TAG=acadbot:dev ./build_ros_image.sh   # custom tag
#
# -----------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_TAG="${IMAGE_TAG:-acadbot:jazzy}"
USER_UID="${USER_UID:-$(id -u)}"
USER_GID="${USER_GID:-$(id -g)}"

echo "=============================================================="
echo " Building image : ${IMAGE_TAG}"
echo " Build context  : ${SCRIPT_DIR}"
echo " User UID:GID    : ${USER_UID}:${USER_GID}"
echo "=============================================================="

docker build \
    --tag "${IMAGE_TAG}" \
    --build-arg USER_UID="${USER_UID}" \
    --build-arg USER_GID="${USER_GID}" \
    --file "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

echo ""
echo "==> Done. Image '${IMAGE_TAG}' is ready."
echo "    Start it with:  ${SCRIPT_DIR}/run_ros_container.sh"
