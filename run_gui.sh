#!/bin/bash
# Launch the single-drone SIM INFRA only: gz Harmonic + PX4 SITL + MicroXRCEAgent.
# The drone stays DISARMED and idle on the ground — run `run_slam.sh` (terminal 2)
# and `run_tour.sh` (terminal 3) to start SLAM then arm + fly the tour.

set -uo pipefail

unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH \
      CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

export PATH=/usr/bin:${PATH}

# Hybrid-GPU: NVIDIA RTX 4070 + AMD iGPU. X display is on AMD.
# - GUI (gz sim -g): use default / Mesa GLX → no NV-GLX X BadValue.
# - Server sensor rendering (gz sim -s): pinned to NVIDIA EGL inside
#   sitl_multiple_run.sh so Ogre2 offscreen render targets go through
#   NVIDIA's proprietary EGL (X-independent) and produce valid frames.
# No global GPU env here — we scope the NVIDIA pin to the server only.

HERE="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${HOME}/px4_jazzy_ws/PX4-Autopilot"
WS_INSTALL="${HOME}/px4_jazzy_ws/install"

mkdir -p "${HERE}/logs"

set +u
source /opt/ros/jazzy/setup.bash
source "${WS_INSTALL}/setup.bash" 2>/dev/null || true
set -u

function cleanup() {
    echo "stopping…"
    pkill -x px4 2>/dev/null || true
    pkill -f 'gz sim' 2>/dev/null || true
    pkill -x ruby 2>/dev/null || true
    pkill -x MicroXRCEAgent 2>/dev/null || true
    pkill -f 'px4_swarm_controller/waypoint' 2>/dev/null || true
    pkill -f 'px4_swarm_controller/arming' 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

echo "== [1/2] Micro-XRCE-DDS Agent =="
MicroXRCEAgent udp4 -p 8888 > "${HERE}/logs/xrce.log" 2>&1 &

echo "== [2/2] gz + PX4 SITL (x500_depth instance 2, DDS namespace /px4_1/) =="
bash "${HERE}/sitl_launch.sh" \
    -m x500_depth -w warehouse -p "0,-1" \
    > "${HERE}/logs/sitl.log" 2>&1 &

# Wait for PX4 to come up and DDS topics to be registered.
echo "    waiting for /px4_1/fmu/out/vehicle_status…"
for attempt in $(seq 1 120); do
    if ros2 topic list 2>/dev/null | grep -q "^/px4_1/fmu/out/vehicle_status$"; then
        echo "    PX4 up (attempt ${attempt})"
        break
    fi
    sleep 1
    [ "${attempt}" -eq 120 ] && { echo "ERROR: PX4 topics never appeared" >&2; exit 1; }
done
sleep 3

echo ""
echo "Sim is up. Drone is DISARMED / idle on the ground."
echo "Next steps (in separate terminals):"
echo "  terminal 2:  bash ${HERE}/run_slam.sh   # start SLAM"
echo "  terminal 3:  bash ${HERE}/run_tour.sh   # arm + fly the tour"
echo "Ctrl-C to stop sim."
wait
