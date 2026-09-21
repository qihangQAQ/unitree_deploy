#!/usr/bin/env bash
set -euo pipefail

DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_BINARY="${DEPLOY_ROOT}/robots/g1_29dof/build/g1_ctrl"
DEPLOY_NETWORK="${1:-}"
DEPLOY_BLIND_POLICY="${DEPLOY_ROOT}/robots/g1_29dof/config/policy/velocity/v0/exported/policy.onnx"

if [[ ! -x "${DEPLOY_BINARY}" ]]; then
  echo "Controller is not built: ${DEPLOY_BINARY}" >&2
  echo "Run ./build.sh first." >&2
  exit 1
fi
if [[ ! -f "${DEPLOY_BLIND_POLICY}" ]]; then
  echo "Blind-walk policy is missing: ${DEPLOY_BLIND_POLICY}" >&2
  exit 1
fi
if [[ -z "${DEPLOY_NETWORK}" ]]; then
  echo "Usage: $0 <dds-network-interface>" >&2
  exit 1
fi

exec "${DEPLOY_BINARY}" --network "${DEPLOY_NETWORK}"
