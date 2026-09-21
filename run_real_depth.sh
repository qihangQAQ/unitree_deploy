#!/usr/bin/env bash
set -euo pipefail

DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_ROS_DISTRO="${UNITREE_DEPLOY_ROS_DISTRO:-humble}"
DEPLOY_ROS_SETUP="/opt/ros/${DEPLOY_ROS_DISTRO}/setup.bash"
DEPLOY_DEPTH_TOPIC="${UNITREE_DEPTH_TOPIC:-/camera/depth/image_rect_raw}"
DEPLOY_CAMERA_INFO_TOPIC="${UNITREE_DEPTH_CAMERA_INFO_TOPIC:-/camera/depth/camera_info}"

if [[ ! -f "${DEPLOY_ROS_SETUP}" ]]; then
  echo "ROS2 setup not found: ${DEPLOY_ROS_SETUP}" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090
source "${DEPLOY_ROS_SETUP}"
set -u

if ! ros2 topic list | grep -Fxq "${DEPLOY_DEPTH_TOPIC}"; then
  echo "Depth topic is not available yet: ${DEPLOY_DEPTH_TOPIC}" >&2
  echo "Start the RealSense ROS2 driver before this script." >&2
  exit 1
fi
if ! ros2 topic list | grep -Fxq "${DEPLOY_CAMERA_INFO_TOPIC}"; then
  echo "Depth CameraInfo topic is not available yet: ${DEPLOY_CAMERA_INFO_TOPIC}" >&2
  exit 1
fi

exec "${DEPLOY_ROOT}/run_real.sh" "$@"
