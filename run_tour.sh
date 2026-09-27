#!/bin/bash
# Arm the drone + start the waypoint tour. Run AFTER run_gui.sh (sim) and
# run_slam.sh (SLAM) are both up and rtabmap is streaming.
#
# Steps:
#   1. Launch the waypoint node on /px4_1 pointing at the tour yaml
#      (it starts publishing position setpoints immediately; the drone
#       ignores them while disarmed).
#   2. Launch the arming node — arms drone and sets offboard mode.
#      The drone then takes off and follows the tour.
#
# Uses waypoints_tour.yaml (spiral-out tour at altitude 2.5 m).

set -uo pipefail

unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH \
      CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

export PATH=/usr/bin:${PATH}

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_INSTALL="${HOME}/px4_jazzy_ws/install"

WP_YAML="${HERE}/waypoints_tour.yaml"

mkdir -p "${HERE}/logs"

set +u
source /opt/ros/jazzy/setup.bash
source "${WS_INSTALL}/setup.bash" 2>/dev/null || true
set -u

function cleanup() {
    echo "stopping tour…"
    pkill -f 'px4_swarm_controller/waypoint' 2>/dev/null || true
    pkill -f 'px4_swarm_controller/arming'   2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

echo "== [1/2] waypoint node   (yaml: ${WP_YAML}) =="
ros2 run px4_swarm_controller waypoint \
    --ros-args \
    -r __node:=waypoint \
    -r __ns:=/px4_1 \
    -p use_sim_time:=false \
    -p wp_path:="${WP_YAML}" \
    -p x_init:=0.0 \
    -p y_init:=-1.0 \
    > "${HERE}/logs/waypoint.log" 2>&1 &

# Give waypoint ~1s to start publishing setpoints before arming engages offboard.
sleep 1

echo "== [2/2] arming node =="
ros2 run px4_swarm_controller arming \
    --ros-args \
    -r __node:=arming \
    -r __ns:=/simulation \
    -p nb_drones:=1 \
    > "${HERE}/logs/arming.log" 2>&1 &

echo ""
echo "Drone should arm and take off to 2.5 m, then fly the tour."
echo "Watch logs in ${HERE}/logs/. Ctrl-C to stop (waypoint + arming only — sim keeps running)."
wait
