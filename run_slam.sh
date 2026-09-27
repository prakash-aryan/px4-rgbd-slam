#!/bin/bash
# Launch the SLAM stack: ros_gz_bridge (RGB+depth+camera_info) +
# static TFs + frame_rewriter + rtabmap (RGB-D, no rviz).
# Run AFTER `run_gui.sh` has the sim flying.

set -uo pipefail

unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH \
      CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

export PATH=/usr/bin:${PATH}

# No GPU env here — SLAM nodes don't render. GUI uses X/Mesa by default.

HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "${HERE}/logs" "${HERE}/rtabmap_db"

set +u
source /opt/ros/jazzy/setup.bash
source "${HOME}/px4_jazzy_ws/install/setup.bash" 2>/dev/null || true
set -u

function cleanup() {
    echo "stopping SLAM…"
    pkill -f 'parameter_bridge' 2>/dev/null || true
    pkill -f 'frame_rewriter'   2>/dev/null || true
    pkill -f 'static_transform' 2>/dev/null || true
    pkill -f 'rtabmap'          2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

echo "== [1/4] ros_gz_bridge =="
ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:="${HERE}/slam_bridge.yaml" \
    > "${HERE}/logs/bridge.log" 2>&1 &
sleep 3

echo "== [2/4] static TFs (base_link -> camera_link -> camera_optical_link) =="
ros2 run tf2_ros static_transform_publisher \
    --x 0.1323 --y 0.0 --z 0.2608 \
    --frame-id base_link --child-frame-id camera_link \
    > "${HERE}/logs/tf_cam.log" 2>&1 &
ros2 run tf2_ros static_transform_publisher \
    --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
    --frame-id camera_link --child-frame-id camera_optical_link \
    > "${HERE}/logs/tf_opt.log" 2>&1 &
sleep 1

echo "== [3/4] frame_rewriter (consistent image frame_id) =="
python3 "${HERE}/frame_rewriter.py" --ros-args -p use_sim_time:=true \
    > "${HERE}/logs/rewriter.log" 2>&1 &
sleep 2

echo "== [4/4] RTAB-Map (RGB-D, headless) =="
ros2 launch rtabmap_launch rtabmap.launch.py \
    args:="--delete_db_on_start" \
    database_path:="${HERE}/rtabmap_db/warehouse.db" \
    frame_id:="base_link" \
    odom_frame_id:="odom" \
    map_frame_id:="map" \
    rgb_topic:="/drone_1/camera/image" \
    depth_topic:="/drone_1/depth/image" \
    camera_info_topic:="/drone_1/camera/camera_info" \
    approx_sync:="true" \
    approx_sync_max_interval:="0.05" \
    use_sim_time:="true" \
    qos:="2" \
    rviz:="false" \
    wait_imu_to_init:="false" \
    publish_tf_map:="true"
