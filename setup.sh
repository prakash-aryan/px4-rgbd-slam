#!/bin/bash
# One-shot setup for px4-rgbd-slam on native Ubuntu 24.04 + ROS 2 Jazzy.
#
# Clones PX4-Autopilot (release/1.15), Micro-XRCE-DDS-Agent, the AWS
# warehouse world, px4_msgs, and a MINIMAL subset of the PX4_Swarm_Controller
# package (arming + waypoint nodes only — no multi-drone weighted-topology
# controllers, no Python simulation_node, no rerun viewer hooks, no
# custom_msgs dep) into ~/px4_jazzy_ws, then builds everything.
#
# Also applies:
#   sim_assets/warehouse.sdf        -> PX4-Autopilot/.../worlds/warehouse.sdf
#   sim_assets/OakD-Lite-model.sdf  -> PX4-Autopilot/.../models/OakD-Lite/model.sdf
#   patches/px4_swarm_controller/{CMakeLists.txt,package.xml}
#   SLAM-friendly MPC_* speed defaults appended to the 4002_gz_x500_depth airframe
#
# Prereqs: sudo privileges, `uv` installed (https://docs.astral.sh/uv/),
# internet. Expect 30-60 min (PX4 build dominates). Run from this repo:
#
#   bash setup.sh

set -euo pipefail

THIS_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/px4_jazzy_ws"

# mise/pyenv can shadow /usr/bin/python3 — PX4 CMake + ROS build both need it.
export PATH=/usr/bin:${PATH}

# ----------------------------------------------------------------------
echo "== [1/8] Installing apt packages =="
sudo apt update
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf-transformations \
    gz-harmonic \
    build-essential cmake git \
    libeigen3-dev libyaml-cpp-dev \
    python3-colcon-common-extensions

# ----------------------------------------------------------------------
echo "== [2/8] Installing PX4 Python build deps via uv =="
if ! command -v uv >/dev/null 2>&1; then
    echo "ERROR: 'uv' is not in PATH. Install from https://docs.astral.sh/uv/getting-started/installation/ and re-run."
    exit 1
fi
UV_BIN="$(command -v uv)"
sudo "${UV_BIN}" pip install --system --break-system-packages \
    empy==3.3.4 pyros-genmsg setuptools jinja2 pyyaml numpy toml \
    jsonschema kconfiglib future pyserial pymavlink symforce

# ----------------------------------------------------------------------
echo "== [3/8] Cloning external repos into ${WS} =="
mkdir -p "${WS}/src" "${WS}/external"
cd "${WS}"

if [ ! -d PX4-Autopilot ]; then
    git clone -b release/1.15 --recursive https://github.com/PX4/PX4-Autopilot.git
fi

if [ ! -d src/px4_msgs ]; then
    git clone -b release/1.15 https://github.com/PX4/px4_msgs.git src/px4_msgs
fi

if [ ! -d Micro-XRCE-DDS-Agent ]; then
    git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi

if [ ! -d external/aws_warehouse ]; then
    git clone --depth 1 -b ros2-jazzy \
        https://github.com/Juancams/aws-robomaker-small-warehouse-world.git \
        external/aws_warehouse
fi

if [ ! -d src/px4_swarm_controller ]; then
    # Clone upstream, then strip multi-drone and unused parts.
    git clone --depth 1 https://github.com/artastier/PX4_Swarm_Controller.git \
        src/px4_swarm_controller

    cd src/px4_swarm_controller

    # Remove multi-drone weighted-topology controllers + headers they need.
    rm -rf src/SwarmControllers include/SwarmControllers
    rm -f  include/NearestNeighbors.hpp \
           include/NeighborsTraits.hpp \
           include/PID.hpp \
           include/SwarmController.hpp

    # Remove Python simulation launcher + multi-drone launch/config assets.
    rm -rf px4_swarm_controller launch config

    # Remove upstream multi-drone launcher + its custom_msgs subdir.
    rm -f  sitl_multiple_run.sh
    rm -rf custom_msgs

    # Drop in our minimal CMakeLists.txt + package.xml (arming + waypoint only,
    # no custom_msgs dependency).
    cp "${THIS_DIR}/patches/px4_swarm_controller/CMakeLists.txt" CMakeLists.txt
    cp "${THIS_DIR}/patches/px4_swarm_controller/package.xml"    package.xml

    cd "${WS}"
fi

# ----------------------------------------------------------------------
echo "== [4/8] Building Micro-XRCE-DDS-Agent =="
cd "${WS}/Micro-XRCE-DDS-Agent"
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DUAGENT_USE_SYSTEM_FASTDDS=OFF \
      -DUAGENT_USE_SYSTEM_FASTCDR=OFF ..
cmake --build . -j"$(nproc)"
sudo cmake --install .
sudo ldconfig /usr/local/lib/

# ----------------------------------------------------------------------
echo "== [5/8] Building PX4 SITL (20-40 min) =="
cd "${WS}/PX4-Autopilot"
sudo "${UV_BIN}" pip install --system --break-system-packages --quiet \
    -r Tools/setup/requirements.txt || true
make px4_sitl_default

# ----------------------------------------------------------------------
echo "== [6/8] Applying sim_assets patches to the PX4 tree =="
cp "${THIS_DIR}/sim_assets/warehouse.sdf" \
   "${WS}/PX4-Autopilot/Tools/simulation/gz/worlds/warehouse.sdf"
cp "${THIS_DIR}/sim_assets/OakD-Lite-model.sdf" \
   "${WS}/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf"

# ----------------------------------------------------------------------
echo "== [7/8] Patching airframe 4002_gz_x500_depth with SLAM-friendly speeds =="
AIRFRAME="${WS}/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4002_gz_x500_depth"
if [ -f "${AIRFRAME}" ] && ! grep -q "MPC_XY_CRUISE" "${AIRFRAME}"; then
    cat >> "${AIRFRAME}" <<'EOF'

# SLAM-friendly speed limits (appended by px4-rgbd-slam/setup.sh)
param set-default MPC_XY_CRUISE 1.2
param set-default MPC_XY_VEL_MAX 1.5
param set-default MPC_Z_VEL_MAX_UP 2.0
param set-default MPC_Z_VEL_MAX_DN 1.5
param set-default MPC_TKO_SPEED 1.5
param set-default MPC_YAWRAUTO_MAX 0.4
param set-default MPC_ACC_HOR_MAX 1.5
EOF
fi

# ----------------------------------------------------------------------
echo "== [8/8] Building ROS workspace (px4_msgs -> px4_swarm_controller) =="
cd "${WS}"
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH \
      CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
set -u

colcon build --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u; source install/setup.bash; set -u

colcon build --packages-select px4_swarm_controller --cmake-args -DCMAKE_BUILD_TYPE=Release

# ----------------------------------------------------------------------
cat <<EOF

======================================================================
  SETUP COMPLETE
======================================================================

If this is a hybrid-GPU laptop (NVIDIA + AMD/Intel), switch X to
NVIDIA-only before running the sim:

    sudo prime-select nvidia
    # log out of the graphical session and log back in

Then run the sim in three terminals from ${THIS_DIR}:

    bash run_gui.sh      # terminal 1 — gz + PX4 + MicroXRCEAgent
    bash run_slam.sh     # terminal 2 — ros_gz_bridge + TFs + RTAB-Map
    bash run_tour.sh     # terminal 3 — arm + fly the tour

EOF
