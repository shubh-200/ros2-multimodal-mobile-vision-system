# ROS 2 Multimodal Spatial Target Locator

**Production-grade autonomous mobile robot simulation with real-time RGB-D sensor fusion, AprilTag detection, and 6-DoF spatial pose estimation — built on ROS 2 Jazzy, Gazebo Harmonic, and the Nav2 autonomy stack.**

![Demo: Autonomous navigation with real-time TF2 pose overlay in RViz](https://github.com/user-attachments/assets/9f80b6c4-ae4b-4e18-80ff-2921c13c7381) 


---

## System Architecture

The system is decomposed into three isolated microservice layers, each independently launchable and testable:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ROS 2  COMPUTE GRAPH                         │
│                                                                     │
│  ┌───────────────┐   ┌───────────────────┐    ┌──────────────────┐  │
│  │  KINEMATICS   │   │    NAVIGATION     │    │  SPATIAL VISION  │  │
│  │               │   │                   │    │                  │  │
│  │ URDF / Xacro  │   │  Nav2 (A* + DWB)  │    │ target_locator   │  │
│  │ ros2_control  │──▶│  AMCL Localization│──▶ │ (C++ microservice│  │
│  │ diff_drive    │   │  Static Map Server│    │  message_filters)│  │
│  │ Gazebo Sim    │   │  twist_stamper    │    │                  │  │
│  └───────┬───────┘   └─────────┬─────────┘    └────────┬─────────┘  │
│          │                     │                       │            │
│          ▼                     ▼                       ▼            │
│     /joint_states         /cmd_vel              /cargo_target       │
│     /scan                 /map                  (TF2 broadcast)     │
│     /camera/image         /odom                                     │
│     /camera/points                                                  │
└─────────────────────────────────────────────────────────────────────┘
```

| Layer | Package | Responsibility |
|---|---|---|
| **Kinematics & Simulation** | `inspector_bot` | Parametric URDF model, `ros2_control` differential drive, Gazebo Harmonic physics, RGB-D + LiDAR sensor plugins, ROS–Gz bridge |
| **Mapping & Autonomy** | `inspector_bot` (Nav2 config) | SLAM-generated static maps, Nav2 global/local planning, AMCL particle filter localization, `twist_stamper` bridge node |
| **Spatial Vision** | `inspector_vision` | Time-synchronized RGB-D fusion, OpenCV optical preprocessing, PCL depth extraction, `tf2_ros` dynamic frame broadcasting |

---

## Key Features & Technical Highlights

### Parametric Robot Model with Accurate Dynamics
- Fully parametric URDF/Xacro definition with computed mass, inertia tensors, and collision geometry for chassis, drive wheels, and caster.
- `ros2_control` hardware interface via `gz_ros2_control` exposing velocity command and joint state interfaces at 100 Hz.
- Configurable differential drive kinematics with velocity/acceleration/jerk limits and odometry covariance tuning.

### Autonomous Warehouse Navigation
- Synchronous SLAM-generated occupancy grid (`.pgm` + `.yaml`) of a simulated warehouse environment.
- Full Nav2 integration: A\* global planner, DWB local controller, AMCL particle filter localization against the static map.
- Custom `twist_stamper` bridging node to resolve strict `geometry_msgs/TwistStamped` vs. `Twist` type mismatches between Nav2 outputs and the `diff_drive_controller`.

### Multimodal Spatial Vision Pipeline
The `target_locator` C++ node is the core of this system, a standalone spatial intelligence microservice that fuses 2D optical and 3D depth data in real-time:

1. **Hardware-Tick Synchronization** : `message_filters::Synchronizer` with an `ApproximateTime` policy fuses `sensor_msgs/Image` and `sensor_msgs/PointCloud2` streams arriving from the 720p RGB-D sensor.
2. **Optical Preprocessing** : Raw `BGR8` frames are converted to grayscale via `cv_bridge` + OpenCV to maximize edge contrast for robust AprilTag 36h11 detection under simulated lighting conditions.
3. **Sub-Pixel 2D Extraction** : The four corners of the detected marker are averaged to compute the tag center with sub-pixel precision.
4. **2D → 3D Spatial Mapping** : The 2D pixel coordinate is projected directly into the organized PCL `PointCloud<PointXYZ>` to extract physical `(X, Y, Z)` depth metrics in the camera optical frame.
5. **TF2 Frame Broadcasting** : A `cargo_target` coordinate frame is dynamically published to the ROS 2 TF tree, making the spatial pose immediately consumable by downstream planners, manipulators, or RViz.

### Robust Systems Engineering


| Challenge | Root Cause | Resolution |
|---|---|---|
| **Silent frame dropping** | Gazebo sensors publish with `BEST_EFFORT` QoS; the C++ node defaulted to `RELIABLE`, causing DDS incompatibility | Explicitly configured `rmw_qos_profile_sensor_data` on all `message_filters` subscribers |
| **AprilTag detection failures** | Simulated environment lighting washes out contrast; default OpenCV thresholds rejected valid geometry | Upgraded sensor model to 720p, converted streams to grayscale, tuned `minDistanceToBorder=0` and `minMarkerDistanceRate=0.01` |
| **AMCL initialization deadlock** | Global costmap crashes when `/map` frame is absent until an initial pose is published | Diagnosed the initialization ordering dependency and resolved the AMCL ↔ `map_server` startup race |
| **Launch race conditions** | Gazebo physics engine requires seconds to initialize; ROS 2 nodes launch in milliseconds, causing crashes on entity spawn | Implemented `TimerAction` with a 12-second delay to defer target spawning until the physics engine is fully available |
| **CMake library collisions** | Anaconda Python environments inject conflicting `libpng` / `Qt5` libraries into the linker search path during `colcon build` | Isolated system library paths and resolved CMake `find_package` precedence conflicts |

---

## Repository Structure

```
ros2-multimodal-mobile-vision-system/
├── inspector_bot/                  # Core robot package
│   ├── urdf/
│   │   └── inspector_bot.urdf.xacro   # Parametric robot model (mass, inertia, sensors)
│   ├── config/
│   │   └── controllers.yaml            # ros2_control diff-drive configuration
│   ├── launch/
│   │   ├── sim_robot.launch.py         # Gazebo Harmonic simulation bootstrap
│   │   └── launch_robot.launch.py      # Physical robot launch (ros2_control)
│   ├── maps/
│   │   ├── warehouse_map.pgm           # SLAM-generated occupancy grid
│   │   └── warehouse_map.yaml          # Map metadata (resolution, origin)
│   ├── models/
│   │   └── cargo_box/                  # SDF model with AprilTag 36h11 texture
│   └── src/
│       └── inspector_node.cpp          # Base robot node
│
├── inspector_vision/               # Spatial vision microservice
│   ├── src/
│   │   └── target_locator.cpp          # Multimodal fusion + TF2 broadcaster
│   ├── CMakeLists.txt
│   └── package.xml
│
└── .gitignore
```

---

## Build & Run

### Prerequisites

| Dependency | Version |
|---|---|
| Ubuntu | 24.04 LTS (Noble) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| OpenCV | 4.x (with `aruco` module) |
| PCL | 1.14+ |
| Nav2 | Jazzy release |

### Install Dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-cv-bridge \
  ros-jazzy-pcl-conversions \
  ros-jazzy-message-filters \
  ros-jazzy-tf2-ros \
  ros-jazzy-controller-manager \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-topic-tools
```

### Build

```bash
# Clone the repository into your colcon workspace
cd ~/ros2_ws/src
git clone https://github.com/shubh-200/ros2-multimodal-mobile-vision-system.git

# Build from workspace root
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Sequence (5 Terminals)

The system requires a staged boot sequence to respect physics engine initialization ordering:

```bash
# Terminal 1 — Gazebo Simulation + Robot Spawn + Sensor Bridge
ros2 launch inspector_bot sim_robot.launch.py
```

```bash
# Terminal 2 — Nav2 Autonomy Stack (after Gazebo is fully loaded)
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=/path/to/ros2_ws/install/inspector_bot/share/inspector_bot/maps/warehouse_map.yaml
```

```bash
# Terminal 3 — Twist Stamper Bridge (Nav2 → diff_drive_controller)
ros2 run twist_stamper twist_stamper --ros-args -r cmd_vel_in:=cmd_vel -r cmd_vel_out:=diff_drive_controller/cmd_vel
```

```bash
# Terminal 4 — RViz2 Visualization
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
# Load your saved .rviz config, or manually add: TF, Map, LaserScan, PointCloud2, Image, RobotModel
```

```bash
# Terminal 5 — Spatial Vision Microservice
ros2 run inspector_vision target_locator --ros-args -p use_sim_time:=true
```

> **Tip:** Set an initial pose estimate in RViz (`2D Pose Estimate`) before sending navigation goals to resolve the AMCL localization prior. Then use `2D Goal Pose` to command the robot toward the cargo target.

---

## Tech Stack

```
ROS 2 Jazzy  ·  Gazebo Harmonic  ·  C++17  ·  Python 3  ·  Nav2
OpenCV (ArUco / AprilTag 36h11)  ·  PCL  ·  tf2_ros  ·  message_filters
ros2_control  ·  cv_bridge  ·  colcon / CMake  ·  AMCL  ·  SLAM Toolbox
```

---

## Future Scope

- **Docker Containerization** : Package the full simulation stack into a multi-stage Docker image with GPU passthrough for reproducible, single-command deployment.
- **MoveIt 2 Integration** : Extend the pipeline with a 6-axis manipulator arm consuming the `cargo_target` TF frame for autonomous pick-and-place operations.
- **Multi-Tag Tracking** : Generalize the vision node to track an array of AprilTag IDs simultaneously, broadcasting unique TF frames per target.
- **Depth Filtering & Outlier Rejection** : Integrate PCL statistical outlier removal and voxel downsampling for robust spatial extraction in noisy real-world sensor data.
- **CI/CD Pipeline** : Add `colcon test` with `ament_lint_auto` and integration tests via `launch_testing` for continuous validation.

---

## License

This project is provided for portfolio and demonstration purposes.
