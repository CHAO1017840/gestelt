#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GESTELT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PX4_DIR="${1:-${GESTELT_DIR}/../PX4-Autopilot}"
PARAMS_SRC="${2:-${GESTELT_DIR}/gestelt_bringup/simulation/px4_airframes/v1_16_0_SITL_altitude_qgc_v443_30_Oct_2026.params}"

MODEL_SRC="${GESTELT_DIR}/gestelt_bringup/simulation/models/raynor"
AIRFRAME_SRC="${GESTELT_DIR}/gestelt_bringup/simulation/px4_airframes/1020_gazebo-classic_raynor"
AIRFRAME_POST_SRC="${AIRFRAME_SRC}.post"
MODEL_DST="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/raynor"
AIRFRAME_DST="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/1020_gazebo-classic_raynor"
AIRFRAME_POST_DST="${AIRFRAME_DST}.post"
SITL_CMAKE="${PX4_DIR}/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
AIRFRAME_CMAKE="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
ROOTFS_DIR="${PX4_DIR}/build/px4_sitl_default/rootfs"
PARAM_CONVERTER="${SCRIPT_DIR}/qgc_params_to_px4_bson.py"
PARAM_XML="${PX4_DIR}/build/px4_sitl_default/parameters.xml"
PATCH_APPLIER="${SCRIPT_DIR}/apply_px4_patches.sh"

# These values describe the simulated Raynor airframe and its verified SITL
# controller tuning. They must win over the imported QGC profile, which was
# exported from an Iris and otherwise replaces the rotor geometry and gains.
RAYNOR_PARAM_OVERRIDES=(
  "SYS_AUTOSTART=1020"
  "SYS_AUTOCONFIG=0"
  "EKF2_EV_CTRL=7"
  "EKF2_GPS_CTRL=0"
  "EKF2_HGT_REF=0"
  "COM_POS_FS_EVV=0"
  "COM_POS_FS_HOLD=1"
  "CBRK_SUPPLY_CHK=894281"

  "CA_ROTOR0_PX=0.056"
  "CA_ROTOR0_PY=0.056"
  "CA_ROTOR0_KM=0.0088"
  "CA_ROTOR1_PX=-0.056"
  "CA_ROTOR1_PY=-0.056"
  "CA_ROTOR1_KM=0.0088"
  "CA_ROTOR2_PX=0.056"
  "CA_ROTOR2_PY=-0.056"
  "CA_ROTOR2_KM=-0.0088"
  "CA_ROTOR3_PX=-0.056"
  "CA_ROTOR3_PY=0.056"
  "CA_ROTOR3_KM=-0.0088"

  "MC_ROLLRATE_P=0.02"
  "MC_ROLLRATE_I=0.02"
  "MC_ROLLRATE_D=0"
  "MC_PITCHRATE_P=0.02"
  "MC_PITCHRATE_I=0.02"
  "MC_PITCHRATE_D=0"
  "MC_ROLL_P=6.0"
  "MC_PITCH_P=5.0"

  "MPC_THR_HOVER=0.3"
  "MPC_THR_MIN=0.12"
  "MPC_XY_P=2.2"
  "MPC_XY_VEL_P_ACC=4.0"
  "MPC_Z_P=0.8"
  "MPC_Z_VEL_P_ACC=4.0"
  "MPC_TILTMAX_AIR=45"
  "MPC_TILTMAX_LND=12"
)

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

if [ ! -f "${AIRFRAME_POST_SRC}" ]; then
  echo "Raynor airframe post script not found: ${AIRFRAME_POST_SRC}" >&2
  exit 1
fi

bash "${PATCH_APPLIER}" "${PX4_DIR}"

mkdir -p "${MODEL_DST}"
cp -a "${MODEL_SRC}/." "${MODEL_DST}/"
cp "${AIRFRAME_SRC}" "${AIRFRAME_DST}"
cp "${AIRFRAME_POST_SRC}" "${AIRFRAME_POST_DST}"

if ! grep -Eq '^[[:space:]]*raynor[[:space:]]*$' "${SITL_CMAKE}"; then
  sed -i '/^[[:space:]]*quadtailsitter[[:space:]]*$/i\
\t\traynor' "${SITL_CMAKE}"
fi

if ! grep -Fq '1020_gazebo-classic_raynor' "${AIRFRAME_CMAKE}"; then
  sed -i '/^[[:space:]]*1021_gazebo-classic_uuv_hippocampus[[:space:]]*$/i\
\t\t1020_gazebo-classic_raynor' "${AIRFRAME_CMAKE}"
fi

if ! grep -Fq '1020_gazebo-classic_raynor.post' "${AIRFRAME_CMAKE}"; then
  sed -i '/^[[:space:]]*1021_gazebo-classic_uuv_hippocampus[[:space:]]*$/i\
\t\t1020_gazebo-classic_raynor.post' "${AIRFRAME_CMAKE}"
fi

echo "Installed Raynor SITL model:"
echo "  model:    ${MODEL_DST}"
echo "  airframe: ${AIRFRAME_DST}"
echo "  post:     ${AIRFRAME_POST_DST}"
echo

if [ -f "${PARAMS_SRC}" ] && [ -f "${PARAM_XML}" ]; then
  PARAM_OVERRIDE_ARGS=()
  for override in "${RAYNOR_PARAM_OVERRIDES[@]}"; do
    PARAM_OVERRIDE_ARGS+=(--set "${override}")
  done

  echo "Applying PX4 parameter profile:"
  echo "  source:   ${PARAMS_SRC}"
  echo "  target:   ${ROOTFS_DIR}/parameters.bson"
  python3 "${PARAM_CONVERTER}" \
    "${PARAMS_SRC}" \
    --parameters-xml "${PARAM_XML}" \
    --output "${ROOTFS_DIR}/parameters.bson" \
    --sync-backup "${ROOTFS_DIR}/parameters_backup.bson" \
    --backup-existing \
    --backup-label raynor_qgc \
    "${PARAM_OVERRIDE_ARGS[@]}" \
    --report "${ROOTFS_DIR}/parameters.applied.txt"
  echo
else
  echo "WARNING: PX4 parameter profile was not applied."
  echo "  parameters file: ${PARAMS_SRC}"
  echo "  built parameter XML: ${PARAM_XML}"
  echo "Build PX4 once, then rerun this installer to generate parameters.bson."
  echo
fi

echo "Rebuild the SITL target if it is not listed:"
echo "  cd ${PX4_DIR}"
echo "  DONT_RUN=1 make px4_sitl gazebo-classic_raynor"
