#!/usr/bin/env bash
set -euo pipefail

DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_BUILD_DIR="${DEPLOY_BUILD_DIR:-${DEPLOY_ROOT}/robots/g1_29dof/build}"
DEPLOY_WITH_ROS2="${UNITREE_DEPLOY_WITH_ROS2:-ON}"
DEPLOY_BUILD_TYPE="${UNITREE_DEPLOY_BUILD_TYPE:-Release}"
DEPLOY_ROS_DISTRO="${UNITREE_DEPLOY_ROS_DISTRO:-humble}"

if [[ "${DEPLOY_WITH_ROS2}" == "ON" ]]; then
  DEPLOY_ROS_SETUP="/opt/ros/${DEPLOY_ROS_DISTRO}/setup.bash"
  if [[ ! -f "${DEPLOY_ROS_SETUP}" ]]; then
    echo "ROS2 setup not found: ${DEPLOY_ROS_SETUP}" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "${DEPLOY_ROS_SETUP}"
  set -u
fi

DEPLOY_CMAKE_ARGS=(
  -S "${DEPLOY_ROOT}/robots/g1_29dof"
  -B "${DEPLOY_BUILD_DIR}"
  -DCMAKE_BUILD_TYPE="${DEPLOY_BUILD_TYPE}"
  -DUNITREE_DEPLOY_WITH_ROS2="${DEPLOY_WITH_ROS2}"
  -DUNITREE_DEPLOY_BUILD_TESTS=ON
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
ctest --test-dir "${DEPLOY_BUILD_DIR}" --output-on-failure
