#!/usr/bin/env bash
set -euo pipefail

DEPLOY_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${1:-}" ]]; then
  echo "Usage: $0 <dds-network-interface>" >&2
  exit 1
fi
if [[ -f "${DEPLOY_SCRIPT_DIR}/scripts/ros_env.sh" ]]; then
  DEPLOY_ROS_HELPER="${DEPLOY_SCRIPT_DIR}/scripts/ros_env.sh"
  DEPLOY_CONFIG_FILE="${DEPLOY_SCRIPT_DIR}/robots/g1_29dof/config/config.yaml"
  DEPLOY_ROS_MARKER="${DEPLOY_SCRIPT_DIR}/robots/g1_29dof/build/ros2_enabled"
else
  DEPLOY_ROS_HELPER="${DEPLOY_SCRIPT_DIR}/ros_env.sh"
  DEPLOY_CONFIG_FILE="${DEPLOY_SCRIPT_DIR}/../config/config.yaml"
  DEPLOY_ROS_MARKER="${DEPLOY_SCRIPT_DIR}/ros2_enabled"
fi
if [[ ! -f "${DEPLOY_ROS_MARKER}" ]]; then
  echo "Controller build marker not found: ${DEPLOY_ROS_MARKER}" >&2
  exit 1
fi
if [[ "$(< "${DEPLOY_ROS_MARKER}")" != "ON" ]]; then
  echo "Controller was not built with ROS 2 depth support." >&2
  exit 1
fi
if [[ ! -f "${DEPLOY_CONFIG_FILE}" ]]; then
  echo "Controller configuration not found: ${DEPLOY_CONFIG_FILE}" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${DEPLOY_ROS_HELPER}"
deploy_source_ros

DEPLOY_CONFIG_TOPICS="$(python3 - "${DEPLOY_CONFIG_FILE}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    depth = yaml.safe_load(stream).get("depth", {})
print(depth.get("topic", "/camera/depth/image_rect_raw"))
print(depth.get("camera_info_topic", "/camera/depth/camera_info"))
PY
)"
mapfile -t DEPLOY_TOPIC_LINES <<< "${DEPLOY_CONFIG_TOPICS}"
DEPLOY_DEPTH_TOPIC="${UNITREE_DEPTH_TOPIC:-${DEPLOY_TOPIC_LINES[0]}}"
DEPLOY_CAMERA_INFO_TOPIC="${UNITREE_DEPTH_CAMERA_INFO_TOPIC:-${DEPLOY_TOPIC_LINES[1]}}"

if ! ros2 topic list | grep -Fxq "${DEPLOY_DEPTH_TOPIC}"; then
  echo "Depth topic is not available yet: ${DEPLOY_DEPTH_TOPIC}" >&2
  echo "Start the RealSense ROS2 driver before this script." >&2
  exit 1
fi
if ! ros2 topic list | grep -Fxq "${DEPLOY_CAMERA_INFO_TOPIC}"; then
  echo "Depth CameraInfo topic is not available yet: ${DEPLOY_CAMERA_INFO_TOPIC}" >&2
  exit 1
fi
if ! ros2 topic type "${DEPLOY_DEPTH_TOPIC}" | grep -Fxq sensor_msgs/msg/Image; then
  echo "Depth topic is not sensor_msgs/msg/Image: ${DEPLOY_DEPTH_TOPIC}" >&2
  exit 1
fi
if ! ros2 topic type "${DEPLOY_CAMERA_INFO_TOPIC}" | grep -Fxq sensor_msgs/msg/CameraInfo; then
  echo "CameraInfo topic is not sensor_msgs/msg/CameraInfo: ${DEPLOY_CAMERA_INFO_TOPIC}" >&2
  exit 1
fi

exec "${DEPLOY_SCRIPT_DIR}/run_real.sh" "$@"
