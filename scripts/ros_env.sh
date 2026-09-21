#!/usr/bin/env bash

# Source the same ROS 2 installation for building and running the controller.
deploy_source_ros() {
  local setup="${UNITREE_DEPLOY_ROS_SETUP:-}"
  if [[ -z "${setup}" && -n "${UNITREE_DEPLOY_ROS_DISTRO:-}" ]]; then
    setup="/opt/ros/${UNITREE_DEPLOY_ROS_DISTRO}/setup.bash"
  fi
  if [[ -z "${setup}" && -n "${ROS_DISTRO:-}" ]]; then
    setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  fi
  if [[ -z "${setup}" ]]; then
    if [[ -f /opt/ros/foxy/setup.bash && ! -f /opt/ros/humble/setup.bash ]]; then
      setup=/opt/ros/foxy/setup.bash
    elif [[ -f /opt/ros/humble/setup.bash && ! -f /opt/ros/foxy/setup.bash ]]; then
      setup=/opt/ros/humble/setup.bash
    else
      echo "Select ROS 2 with UNITREE_DEPLOY_ROS_SETUP or UNITREE_DEPLOY_ROS_DISTRO." >&2
      return 1
    fi
  fi

  if [[ ! -f "${setup}" ]]; then
    echo "ROS 2 setup not found: ${setup}" >&2
    return 1
  fi
  if [[ "${UNITREE_DEPLOY_ACTIVE_ROS_SETUP:-}" == "${setup}" ]]; then
    return 0
  fi

  local restore_nounset=0
  [[ $- == *u* ]] && restore_nounset=1
  set +u
  # shellcheck disable=SC1090
  if source "${setup}"; then
    if [[ ${restore_nounset} == 1 ]]; then set -u; fi
    export UNITREE_DEPLOY_ACTIVE_ROS_SETUP="${setup}"
    return 0
  fi
  if [[ ${restore_nounset} == 1 ]]; then set -u; fi
  echo "Failed to source ROS 2 setup: ${setup}" >&2
  return 1
}
