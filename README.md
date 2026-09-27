# px4-rgbd-slam: Single-drone PX4 + Gazebo Harmonic + RTAB-Map SLAM

Single-drone PX4 SITL + Gazebo Harmonic warehouse world + RTAB-Map RGB-D SLAM, built on top of PX4 release/1.15 and ROS 2 Jazzy. The drone takes off autonomously and flies a spiral-out tour inside the warehouse while RTAB-Map builds a 3D RGB-D map from the forward-facing OakD-Lite.

![RTAB-Map SLAM in the Gazebo warehouse](rtab-slam.gif)

This project contains the **launchers, trajectory, and SLAM bridge**. It reads pre-built binaries and models from `~/px4_jazzy_ws` (PX4 SITL, Micro-XRCE-DDS agent, AWS warehouse meshes, and the `waypoint` + `arming` ROS nodes from the `px4_swarm_controller` package) and does not modify them.

## Install

**Prerequisites**: install these first on a fresh Ubuntu 24.04 (native, no distrobox):

```bash
# 1. Base build tools + git (apt install handles the rest automatically inside setup.sh)
sudo apt update && sudo apt install -y curl git build-essential

# 2. ROS 2 Jazzy apt source (if not already added); skip if `apt list ros-jazzy-desktop` finds it
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
     sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Gazebo Harmonic apt source (if not already added)
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
     http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
     sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 4. uv (Python package manager, used by setup.sh for PX4 Python deps)
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"   # or just open a new terminal
```

**Clone + run setup**. The working-copy directory must be named `px4_single_ws`: the launch scripts reference that path.

```bash
git clone https://github.com/prakash-aryan/px4-rgbd-slam.git ~/px4_single_ws
cd ~/px4_single_ws
bash setup.sh
```

What `setup.sh` does:

1. `sudo apt install`: ROS 2 Jazzy desktop, `ros-gz`, `rtabmap-ros`, `rmw-cyclonedds-cpp`, `tf-transformations`, `gz-harmonic`, CMake, Eigen, yaml-cpp, colcon.
2. `sudo uv pip install --system`: PX4 Python deps (`empy==3.3.4`, `pyros-genmsg`, `pymavlink`, `symforce`, `kconfiglib`, …).
3. Clones into `~/px4_jazzy_ws`:
   - `PX4-Autopilot` (`release/1.15` branch, recursive submodules)
   - `px4_msgs` (`release/1.15` branch)
   - `Micro-XRCE-DDS-Agent`
   - `aws-robomaker-small-warehouse-world` (`ros2-jazzy` branch, Juancams fork)
   - `PX4_Swarm_Controller`: **stripped down to just the `arming` + `waypoint` nodes**; the multi-drone weighted-topology controllers, Python launchers, rerun-adjacent assets, and `custom_msgs` dependency are deleted, and a minimal `CMakeLists.txt` + `package.xml` from `patches/` are dropped in on top.
4. Builds Micro-XRCE-DDS-Agent (installed to `/usr/local/bin/MicroXRCEAgent`).
5. Builds PX4 SITL (`make px4_sitl_default`).
6. Copies `sim_assets/warehouse.sdf` and `sim_assets/OakD-Lite-model.sdf` into the PX4 tree (warehouse world + per-instance camera topics).
7. Appends SLAM-friendly `MPC_*` speed defaults to the `4002_gz_x500_depth` airframe.
8. `colcon build`s `px4_msgs` and the stripped `px4_swarm_controller`.

`setup.sh` is idempotent: re-running it skips steps that are already done (existence-guarded clones, `! grep -q` for the airframe append, etc.).

### NVIDIA hybrid-GPU note

On laptops with NVIDIA + AMD/Intel hybrid GPUs, Gazebo Ogre2 sensor rendering fails silently in `prime-select on-demand` mode (camera images come out uniformly gray, depth = +Inf). Switch X to NVIDIA-only after `setup.sh` finishes:

```bash
sudo prime-select nvidia
# log out of the graphical session and log back in
glxinfo -B | grep "OpenGL vendor"   # should say NVIDIA Corporation
```

## Running the sim

Three terminals, native Ubuntu 24.04:

```bash
# Terminal 1: sim infra (gz + PX4 SITL + uXRCE-DDS agent). Drone idle on ground.
cd ~/px4_single_ws
bash run_gui.sh

# Terminal 2: SLAM stack (bridge + TFs + RTAB-Map). Waits for images.
cd ~/px4_single_ws
bash run_slam.sh

# Terminal 3: arms drone + flies the tour.
cd ~/px4_single_ws
bash run_tour.sh
```

The drone climbs to 2.5 m and flies the spiral-out tour defined in `waypoints_tour.yaml`. RTAB-Map's `rtabmap_viz` window shows the map growing in real time.

## File layout

```
px4_single_ws/
├── setup.sh                      one-shot install: apt, uv, clone + build everything
├── run_gui.sh                    Terminal 1: gz + PX4 + MicroXRCEAgent (idle sim)
├── run_slam.sh                   Terminal 2: ros_gz_bridge + static TFs + frame_rewriter + RTAB-Map
├── run_tour.sh                   Terminal 3: waypoint node + arming → takeoff + tour
├── sitl_launch.sh                gz + PX4 launcher with the sacrificial-instance-1 workaround
├── slam_bridge.yaml              ros_gz_bridge config (maps gz topics → /drone_1/*)
├── frame_rewriter.py             republishes camera/depth with frame_id=camera_optical_link
├── waypoints_tour.yaml           spiral-out tour (18 waypoints at 2.5 m altitude)
├── sim_assets/
│   ├── warehouse.sdf             warehouse world (AWS RoboMaker, jazzy-compatible)
│   └── OakD-Lite-model.sdf       patched OakD model (per-instance camera topics)
├── patches/px4_swarm_controller/
│   ├── CMakeLists.txt            stripped-down, single-drone only
│   └── package.xml               drops custom_msgs dep
├── logs/                         runtime logs (gitignored)
├── rtabmap_db/                   RTAB-Map session databases (gitignored)
└── README.md
```

## Shared dependencies (from `~/px4_jazzy_ws`)

| Path | Purpose |
|---|---|
| `~/px4_jazzy_ws/PX4-Autopilot/build/px4_sitl_default/bin/px4` | PX4 SITL binary |
| `~/px4_jazzy_ws/PX4-Autopilot/Tools/simulation/gz/worlds/warehouse.sdf` | warehouse world |
| `~/px4_jazzy_ws/install/px4_swarm_controller/lib/.../waypoint` | waypoint trajectory node |
| `~/px4_jazzy_ws/install/px4_swarm_controller/lib/.../arming` | arm + offboard commander |
| `~/px4_jazzy_ws/external/aws_warehouse/models/` | warehouse meshes |

Override `PX4_DIR` in the environment to use a PX4 build elsewhere.

## Stopping

`Ctrl-C` in each terminal. Each script has a trap that kills its own children. Full wipe:

```bash
pkill -9 -f 'run_gui|run_slam|run_tour|px4 -i|gz |MicroXRCE|ruby|rtabmap|waypoint|arming|parameter_bridge|static_transform|frame_rewriter'
```

## References

Upstream projects this stack depends on (all links verified):

- **[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)**: flight stack (uses the `release/1.15` branch).
- **[px4_msgs](https://github.com/PX4/px4_msgs)**: ROS 2 message definitions for PX4 uORB topics.
- **[Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)**: uXRCE-DDS broker that bridges PX4 ↔ ROS 2.
- **[PX4_Swarm_Controller](https://github.com/artastier/PX4_Swarm_Controller)** (Arthur Astier): source of the `waypoint` and `arming` ROS 2 nodes used by `run_tour.sh`.
- **[aws-robomaker-small-warehouse-world](https://github.com/Juancams/aws-robomaker-small-warehouse-world)** (`ros2-jazzy` branch by Juancams): warehouse meshes + world used for the sim scene.
- **[rtabmap_ros](https://github.com/introlab/rtabmap_ros)**: ROS 2 wrapper for RTAB-Map (the `rtabmap.launch.py` invoked by `run_slam.sh`).
- **[RTAB-Map](https://github.com/introlab/rtabmap)**: the underlying real-time appearance-based mapping library.

Platforms:

- **[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/)**: simulator + Ogre2 sensor rendering.
- **[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)**: middleware, rmw, and launch infrastructure.
