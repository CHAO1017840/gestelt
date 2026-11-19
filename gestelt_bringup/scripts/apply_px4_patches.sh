#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GESTELT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PX4_DIR="${1:-${GESTELT_DIR}/../PX4-Autopilot}"
SITL_DIR="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
PATCH_DIR="${SCRIPT_DIR}/patches"

apply_one() {
  local repo="$1"
  local patch_file="$2"
  local label="$3"

  if git -C "${repo}" apply --check "${patch_file}" >/dev/null 2>&1; then
    git -C "${repo}" apply "${patch_file}"
    echo "Applied ${label}: ${patch_file}"
  elif git -C "${repo}" apply --reverse --check "${patch_file}" >/dev/null 2>&1; then
    echo "Already applied ${label}: ${patch_file}"
  else
    echo "Cannot apply ${label}: ${patch_file}" >&2
    echo "The PX4 checkout is not at the expected v1.14 base or has conflicting edits." >&2
    return 1
  fi
}

if [ ! -d "${PX4_DIR}" ]; then
  echo "PX4-Autopilot not found: ${PX4_DIR}" >&2
  exit 1
fi

if [ ! -d "${SITL_DIR}" ]; then
  echo "Gazebo Classic SITL submodule not found: ${SITL_DIR}" >&2
  exit 1
fi

apply_one \
  "${PX4_DIR}" \
  "${PATCH_DIR}/px4_core_validity_hold.patch" \
  "PX4 validity-hold patch"

apply_one \
  "${SITL_DIR}" \
  "${PATCH_DIR}/gazebo_classic_user_camera_lock.patch" \
  "Gazebo GUI model-lock patch"
