#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GESTELT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PX4_DIR="${1:-${GESTELT_DIR}/../PX4-Autopilot}"

MODEL_SRC="${GESTELT_DIR}/gestelt_bringup/simulation/models/raynor"
AIRFRAME_SRC="${GESTELT_DIR}/gestelt_bringup/simulation/px4_airframes/1020_gazebo-classic_raynor"
MODEL_DST="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/raynor"
AIRFRAME_DST="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/1020_gazebo-classic_raynor"
SITL_CMAKE="${PX4_DIR}/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
AIRFRAME_CMAKE="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"

if [ ! -d "${PX4_DIR}" ]; then
  echo "PX4-Autopilot not found: ${PX4_DIR}" >&2
  exit 1
fi

if [ ! -f "${MODEL_SRC}/raynor.sdf" ]; then
  echo "Raynor model not found: ${MODEL_SRC}" >&2
  exit 1
fi

if [ ! -f "${AIRFRAME_SRC}" ]; then
  echo "Raynor airframe not found: ${AIRFRAME_SRC}" >&2
  exit 1
fi

mkdir -p "${MODEL_DST}"
cp -a "${MODEL_SRC}/." "${MODEL_DST}/"
cp "${AIRFRAME_SRC}" "${AIRFRAME_DST}"

if ! grep -Eq '^[[:space:]]*raynor[[:space:]]*$' "${SITL_CMAKE}"; then
  sed -i '/^[[:space:]]*quadtailsitter[[:space:]]*$/i\
\t\traynor' "${SITL_CMAKE}"
fi

if ! grep -Fq '1020_gazebo-classic_raynor' "${AIRFRAME_CMAKE}"; then
  sed -i '/^[[:space:]]*1021_gazebo-classic_uuv_hippocampus[[:space:]]*$/i\
\t\t1020_gazebo-classic_raynor' "${AIRFRAME_CMAKE}"
fi

echo "Installed Raynor SITL model:"
echo "  model:    ${MODEL_DST}"
echo "  airframe: ${AIRFRAME_DST}"
echo
echo "Rebuild the SITL target if it is not listed:"
echo "  cd ${PX4_DIR}"
echo "  DONT_RUN=1 make px4_sitl gazebo-classic_raynor"
