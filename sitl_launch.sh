#!/bin/bash
# Single-drone PX4 SITL launcher for Gazebo Harmonic.
#
# Starts gz externally (server + GUI), waits for the world, then launches
# PX4 as *instance 2* with PX4_UXRCE_DDS_NS=px4_1. Why instance 2: when
# the first PX4 to spawn after gz loads is instance 1, its gz-transport
# sensor subscriptions hit a race that intermittently drops the compass,
# blocking arming with "Compass Sensor 0 missing". Starting at instance
# 2 avoids the race; the UXRCE override keeps ROS DDS topics at /px4_1/
# so any downstream code hard-coded to that namespace still works.
#
# Args:
#   -m <model>   model (default: x500_depth)
#   -w <world>   gz world (default: warehouse)
#   -t <target>  PX4 build target (default: px4_sitl_default)
#   -p "x,y"     initial pose (default: 0,-1)
#   -d <dir>     PX4-Autopilot directory (default: ~/px4_jazzy_ws/PX4-Autopilot)
#
# Set HEADLESS=1 to skip the gz GUI.

set -u

trap 'pkill -x px4 || true; pkill -f "gz sim" || true; pkill -x ruby || true' SIGINT SIGTERM EXIT

MODEL=x500_depth
WORLD=warehouse
TARGET=px4_sitl_default
POSE="0,-1"
PX4_DIR_ARG=""

while getopts "m:w:t:p:d:" opt; do
    case "${opt}" in
        m) MODEL=${OPTARG} ;;
        w) WORLD=${OPTARG} ;;
        t) TARGET=${OPTARG} ;;
        p) POSE=${OPTARG} ;;
        d) PX4_DIR_ARG=${OPTARG} ;;
        *) echo "unknown option"; exit 2 ;;
    esac
done

PX4_DIR="${PX4_DIR_ARG:-${HOME}/px4_jazzy_ws/PX4-Autopilot}"
BUILD="${PX4_DIR}/build/${TARGET}"

if [ ! -x "${BUILD}/bin/px4" ]; then
    echo "ERROR: PX4 binary not found at ${BUILD}/bin/px4" >&2
    echo "       Build via: cd ${PX4_DIR} && make ${TARGET}" >&2
    exit 1
fi

# Airframe ID lookup (covers the x500 family shipped by PX4 release/1.15).
case "${MODEL}" in
    x500)           AIRFRAME=4001 ;;
    x500_depth)     AIRFRAME=4002 ;;
    x500_vision)    AIRFRAME=4005 ;;
    x500_mono_cam)  AIRFRAME=4010 ;;
    *) echo "ERROR: unknown model '${MODEL}'" >&2; exit 1 ;;
esac

IFS=',' read -r POSE_X POSE_Y <<< "${POSE}"
POSE_X=${POSE_X:-0}
POSE_Y=${POSE_Y:--1}

echo "killing any running PX4 / gz"
pkill -x px4 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true
pkill -x ruby 2>/dev/null || true
sleep 1

# PX4's gz env (GZ_SIM_RESOURCE_PATH, etc). Plus our aws_warehouse models.
set +u
if [ -f "${BUILD}/rootfs/gz_env.sh" ]; then . "${BUILD}/rootfs/gz_env.sh"; fi
export GZ_SIM_RESOURCE_PATH="${HOME}/px4_jazzy_ws/external/aws_warehouse/models:${GZ_SIM_RESOURCE_PATH:-}"
set -u

WORLD_SDF="${PX4_DIR}/Tools/simulation/gz/worlds/${WORLD}.sdf"
[ -f "${WORLD_SDF}" ] || { echo "ERROR: world sdf not found: ${WORLD_SDF}" >&2; exit 1; }

echo "Starting gz sim server (world=${WORLD})"
# Hybrid-GPU: pin the SERVER's EGL to NVIDIA so Ogre2 sensor offscreen
# rendering uses the NVIDIA GPU (X-independent EGL, avoids the NV-GLX
# crash that hits GUI when we pin GLX to NVIDIA). Also request PRIME
# render offload so NVIDIA is picked even when X is on AMD.
env __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    __NV_PRIME_RENDER_OFFLOAD=1 \
    __VK_LAYER_NV_optimus=NVIDIA_only \
    gz sim --verbose=1 -r -s "${WORLD_SDF}" > /tmp/gz_server.log 2>&1 &
if [ -z "${HEADLESS:-}" ]; then
    echo "Starting gz sim GUI (default GPU routing — X on AMD)"
    gz sim -g > /tmp/gz_gui.log 2>&1 &
fi

# Wait for scene/info so gz is fully initialized before spawning PX4.
for attempt in $(seq 1 60); do
    if gz service -i --service "/world/${WORLD}/scene/info" 2>/dev/null | grep -q "Service providers"; then
        echo "gz world '${WORLD}' is ready (attempt ${attempt})"
        break
    fi
    sleep 1
    [ "${attempt}" -eq 60 ] && { echo "ERROR: timed out waiting for gz world" >&2; exit 1; }
done
sleep 5

# -----------------------------------------------------------------------------
# Launch a SACRIFICIAL instance 1 first. Its only job is to be the first
# subscriber to gz-transport sensor topics, losing the race that drops the
# compass. We park it far outside the warehouse (100, 100) where it sits
# on the ground forever; nothing commands it, no ROS namespace collides
# (DDS namespace set to px4_sacrifice).
# -----------------------------------------------------------------------------
SACRIFICE_DIR="${BUILD}/rootfs/1"
mkdir -p "${SACRIFICE_DIR}"
rm -rf "${SACRIFICE_DIR}"/*

echo "Starting sacrificial PX4 instance 1 at (100,100) to warm up gz subscriptions"
pushd "${SACRIFICE_DIR}" > /dev/null
env PX4_UXRCE_DDS_NS=px4_sacrifice \
    PX4_GZ_STANDALONE=1 \
    PX4_SYS_AUTOSTART="${AIRFRAME}" \
    PX4_GZ_WORLD="${WORLD}" \
    PX4_SIM_MODEL="gz_${MODEL}" \
    PX4_GZ_MODEL_POSE="100,100,0.5,0,0,0" \
    "${BUILD}/bin/px4" -i 1 -d "${BUILD}/etc" > out.log 2> err.log &
popd > /dev/null

# Wait long enough for instance 1 to fully subscribe to sensors (and lose
# the race). ~10 s has proven reliable in testing.
echo "  waiting 12 s for sacrifice to warm up gz…"
sleep 12

# -----------------------------------------------------------------------------
# Now launch the REAL drone as instance 2. It's the second subscriber, so
# gz sensor topics are warm; compass / baro / GPS come up cleanly every time.
# Force PX4_UXRCE_DDS_NS=px4_1 so ROS code sees the familiar /px4_1/... prefix.
# -----------------------------------------------------------------------------
INSTANCE=2
WORKING_DIR="${BUILD}/rootfs/${INSTANCE}"
mkdir -p "${WORKING_DIR}"
rm -rf "${WORKING_DIR}"/*

echo "Starting PX4 instance ${INSTANCE}: airframe=${AIRFRAME} model=gz_${MODEL} pose=(${POSE_X},${POSE_Y})"
pushd "${WORKING_DIR}" > /dev/null
env PX4_UXRCE_DDS_NS=px4_1 \
    PX4_GZ_STANDALONE=1 \
    PX4_SYS_AUTOSTART="${AIRFRAME}" \
    PX4_GZ_WORLD="${WORLD}" \
    PX4_SIM_MODEL="gz_${MODEL}" \
    PX4_GZ_MODEL_POSE="${POSE_X},${POSE_Y},0.5,0,0,0" \
    "${BUILD}/bin/px4" -i "${INSTANCE}" -d "${BUILD}/etc" > out.log 2> err.log &
popd > /dev/null

echo "Both PX4 instances launched. Instance 2 (/px4_1/ topics) is the real drone."
echo "Press Ctrl-C to stop."
wait
