#!/usr/bin/env bash
set -euo pipefail

DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_BUILD_DIR="${DEPLOY_BUILD_DIR:-${DEPLOY_ROOT}/robots/g1_29dof/build}"
DEPLOY_WITH_ROS2="${UNITREE_DEPLOY_WITH_ROS2:-ON}"
DEPLOY_BUILD_TYPE="${UNITREE_DEPLOY_BUILD_TYPE:-Release}"
DEPLOY_BUILD_TESTS="${UNITREE_DEPLOY_BUILD_TESTS:-ON}"

if [[ "${DEPLOY_WITH_ROS2}" == "ON" ]]; then
  # shellcheck source=scripts/ros_env.sh
  source "${DEPLOY_ROOT}/scripts/ros_env.sh"
  deploy_source_ros
fi

DEPLOY_CMAKE_ARGS=(
  -S "${DEPLOY_ROOT}/robots/g1_29dof"
  -B "${DEPLOY_BUILD_DIR}"
  -DCMAKE_BUILD_TYPE="${DEPLOY_BUILD_TYPE}"
  -DUNITREE_DEPLOY_WITH_ROS2="${DEPLOY_WITH_ROS2}"
  -DUNITREE_DEPLOY_BUILD_TESTS="${DEPLOY_BUILD_TESTS}"
)

# Prefer the system compiler used by ROS2 unless the caller explicitly selected one.
if [[ "${DEPLOY_WITH_ROS2}" == "ON" && -z "${CXX:-}" && -x /usr/bin/g++ ]]; then
  DEPLOY_CMAKE_ARGS+=( -DCMAKE_CXX_COMPILER=/usr/bin/g++ )
fi
if [[ -n "${ONNXRUNTIME_ROOT:-}" ]]; then
  DEPLOY_CMAKE_ARGS+=( -DONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT}" )
fi

cmake "${DEPLOY_CMAKE_ARGS[@]}"
cmake --build "${DEPLOY_BUILD_DIR}" --parallel "${UNITREE_DEPLOY_JOBS:-2}"
if [[ "${DEPLOY_BUILD_TESTS}" == "ON" ]]; then
  ctest --test-dir "${DEPLOY_BUILD_DIR}" --output-on-failure
fi
