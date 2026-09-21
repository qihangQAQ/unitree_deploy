#!/usr/bin/env bash
set -euo pipefail

DEPLOY_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_NETWORK="${1:-}"

if [[ -z "${DEPLOY_NETWORK}" ]]; then
  echo "Usage: $0 <dds-network-interface>" >&2
  exit 1
fi

if [[ -x "${DEPLOY_SCRIPT_DIR}/robots/g1_29dof/build/g1_ctrl" ]]; then
  DEPLOY_BINARY="${DEPLOY_SCRIPT_DIR}/robots/g1_29dof/build/g1_ctrl"
  DEPLOY_ROS_HELPER="${DEPLOY_SCRIPT_DIR}/scripts/ros_env.sh"
  DEPLOY_ROS_MARKER="${DEPLOY_SCRIPT_DIR}/robots/g1_29dof/build/ros2_enabled"
else
  DEPLOY_BINARY="${DEPLOY_SCRIPT_DIR}/g1_ctrl"
  DEPLOY_ROS_HELPER="${DEPLOY_SCRIPT_DIR}/ros_env.sh"
  DEPLOY_ROS_MARKER="${DEPLOY_SCRIPT_DIR}/ros2_enabled"
fi
if [[ ! -x "${DEPLOY_BINARY}" ]]; then
  echo "Controller is not built: ${DEPLOY_BINARY}" >&2
  echo "Run ./build.sh or install the controller package first." >&2
  exit 1
fi

if [[ ! -f "${DEPLOY_ROS_MARKER}" ]]; then
  echo "Controller build marker not found: ${DEPLOY_ROS_MARKER}" >&2
  exit 1
fi
if [[ "$(< "${DEPLOY_ROS_MARKER}")" == "ON" ]]; then
  # shellcheck disable=SC1090
  source "${DEPLOY_ROS_HELPER}"
  deploy_source_ros
fi

exec "${DEPLOY_BINARY}" --network "${DEPLOY_NETWORK}"
