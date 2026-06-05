# ROS 2 Multimodal Spatial Target Locator

**Autonomous mobile robot simulation with lifecycle-managed RGB-D sensor fusion, AprilTag detection, and 6-DoF spatial pose estimation — orchestrated through a Nav2 Behavior Tree action interface, built on ROS 2 Jazzy, Gazebo Harmonic, and the Nav2 autonomy stack.**

Demo: Autonomous navigation with real-time TF2 pose overlay in RViz 

https://github.com/user-attachments/assets/9f80b6c4-ae4b-4e18-80ff-2921c13c7381

---

## System Architecture

The system is decomposed into four isolated layers, orchestrated by a single master launch file and connected through a lifecycle-managed action server interface:

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         ROS 2  COMPUTE GRAPH                             │
│                                                                          │
│  ┌───────────────┐   ┌───────────────────┐    ┌───────────────────────┐  │
│  │  KINEMATICS   │   │    NAVIGATION     │    │    SPATIAL VISION     │  │
│  │               │   │                   │    │   (LifecycleNode)     │  │
│  │ URDF / Xacro  │   │  Nav2 (A* + DWB)  │    │                       │  │
│  │ ros2_control  │──▶│  AMCL Localization│──▶ │  target_locator       │  │
│  │ diff_drive    │   │  Custom BT w/     │◀── │  Action Server        │  │
│  │ Gazebo Sim    │   │  LocateTarget     │    │  (locate_target)      │  │
│  └───────┬───────┘   └─────────┬─────────┘    └──────────┬────────────┘  │
│          │                     │                         │               │
│          ▼                     ▼                         ▼               │
│     /joint_states         /cmd_vel              /cargo_target            │
│     /scan                 /map                  (TF2 broadcast)          │
│     /camera/image         /odom                 LocateTarget.action      │
│     /camera/points                                                       │
└──────────────────────────────────────────────────────────────────────────┘
```

| Layer | Package | Responsibility |
|---|---|---|
| **Kinematics & Simulation** | `inspector_bot` | Parametric URDF model, `ros2_control` differential drive, Gazebo Harmonic physics, RGB-D + LiDAR sensor plugins, ROS–Gz bridge |
| **Mapping & Autonomy** | `inspector_bot` (Nav2 config) | SLAM-generated static maps, Nav2 global/local planning, AMCL particle filter localization, `twist_stamper` bridge node, custom Behavior Tree with `LocateTarget` action leaf |
| **Action Interface** | `inspector_interfaces` | Custom `LocateTarget.action` definition — goal (`target_id`), result (`success`, `final_pose`), feedback (`status`) |
| **Spatial Vision** | `inspector_vision` | `rclcpp_lifecycle::LifecycleNode` with `rclcpp_action` server, time-synchronized RGB-D fusion, OpenCV preprocessing, PCL depth extraction, `tf2_ros` broadcasting |

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

### Lifecycle-Managed Vision with Action Server Interface
The `target_locator` node is the core of this system — a `rclcpp_lifecycle::LifecycleNode` that exposes a `LocateTarget` action server, enabling Nav2's Behavior Tree to orchestrate the vision pipeline on-demand:

**Lifecycle State Machine:**
- **`on_configure`** — Allocates the action server and TF broadcaster without connecting to any sensor streams (zero bandwidth cost while idle).
- **`on_activate`** — Connects to `message_filters` RGB-D subscribers and binds the `ApproximateTime` synchronizer (sensor data starts flowing only when needed).
- **`on_deactivate`** — Destroys synchronizer and subscriber pointers, immediately severing the DDS network connections to reclaim bandwidth.
- **`on_cleanup`** — Tears down the action server and TF broadcaster.

**Action Server Contract** (`LocateTarget.action`):
```
# Goal       →  string target_id
# Result     →  bool success, geometry_msgs/PoseStamped final_pose
# Feedback   →  string status
```

**Sensor Processing Pipeline** (executes per-frame only when an active goal is running):
1. **Hardware-Tick Synchronization** : `message_filters::Synchronizer` with an `ApproximateTime` policy fuses `sensor_msgs/Image` and `sensor_msgs/PointCloud2` streams arriving from the 720p RGB-D sensor.
2. **Optical Preprocessing** : Raw `BGR8` frames are converted to grayscale via `cv_bridge` + OpenCV to maximize edge contrast for robust AprilTag 36h11 detection under simulated lighting conditions.
3. **Sub-Pixel 2D Extraction** : The four corners of the detected marker are averaged to compute the tag center with sub-pixel precision.
4. **2D → 3D Spatial Mapping** : The 2D pixel coordinate is projected directly into the organized PCL `PointCloud<PointXYZ>` to extract physical `(X, Y, Z)` depth metrics in the camera optical frame.
5. **TF2 Frame Broadcasting** : A `cargo_target` coordinate frame is dynamically published to the ROS 2 TF tree, making the spatial pose immediately consumable by downstream planners, manipulators, or RViz.

### Nav2 Behavior Tree Integration
The navigation stack runs a **custom Behavior Tree** (`custom_nav_tree.xml`) that extends the standard Nav2 `NavigateWithReplanning` tree with a `LocateTarget` action leaf. After the robot successfully navigates to a goal pose, the BT automatically triggers the vision pipeline:

```xml
<Sequence name="AutonomousMissionSequence">
  <RecoveryNode>...</RecoveryNode>   <!-- Standard Nav2 navigate + recovery -->
  <LocateTarget target_id="cargo_box_1" server_name="locate_target" server_timeout="1000"/>
</Sequence>
```

The `LocateTargetAction` BT plugin (`locate_target_bt_node.cpp`) is compiled as a shared library and loaded by Nav2 at runtime via `BT_REGISTER_NODES`. It acts as the bridge between the Behavior Tree XML and the C++ action server.

### Robust Systems Engineering


| Challenge | Root Cause | Resolution |
|---|---|---|
| **Silent frame dropping** | Gazebo sensors publish with `BEST_EFFORT` QoS; the C++ node defaulted to `RELIABLE`, causing DDS incompatibility | Explicitly configured `rmw_qos_profile_sensor_data` on all `message_filters` subscribers |
| **AprilTag detection failures** | Simulated environment lighting washes out contrast; default OpenCV thresholds rejected valid geometry | Upgraded sensor model to 720p, converted streams to grayscale, tuned `minDistanceToBorder=0` and `minMarkerDistanceRate=0.01` |
| **AMCL initialization deadlock** | Global costmap crashes when `/map` frame is absent until an initial pose is published | Diagnosed the initialization ordering dependency and resolved the AMCL ↔ `map_server` startup race |
| **Launch race conditions** | Gazebo physics engine requires seconds to initialize; ROS 2 nodes launch in milliseconds, causing crashes on entity spawn | Implemented `TimerAction` with a 12-second delay in the master bringup to defer infrastructure boot until the physics engine is fully available |
| **Lifecycle subscriber management** | Keeping `message_filters` subscribers active during idle periods wastes DDS bandwidth and creates unnecessary network load | Vision node connects sensors only in `on_activate`, destroys pointers in `on_deactivate` — zero sensor bandwidth when inactive |
| **CMake library collisions** | Anaconda Python environments inject conflicting `libpng` / `Qt5` libraries into the linker search path during `colcon build` | Isolated system library paths and resolved CMake `find_package` precedence conflicts |

---

## Repository Structure

```
ros2-multimodal-mobile-vision-system/
├── inspector_bot/                  # Core robot package
│   ├── urdf/
│   │   └── inspector_bot.urdf.xacro   # Parametric robot model (mass, inertia, sensors)
│   ├── config/
│   │   ├── controllers.yaml            # ros2_control diff-drive configuration
│   │   ├── nav2_params.yaml            # Full Nav2 parameter tuning
│   │   └── custom_nav_tree.xml         # BT with LocateTarget action leaf
│   ├── launch/
│   │   ├── master_bringup.launch.py    # ★ Single-command full system bringup
│   │   ├── sim_robot.launch.py         # Gazebo Harmonic simulation bootstrap
│   │   └── launch_robot.launch.py      # Physical robot launch (ros2_control)
│   ├── maps/
│   │   ├── warehouse_map.pgm           # SLAM-generated occupancy grid
│   │   └── warehouse_map.yaml          # Map metadata (resolution, origin)
│   ├── models/
│   │   └── cargo_box/                  # SDF model with AprilTag 36h11 texture
│   ├── rviz/
│   │   └── production.rviz             # Frozen RViz dashboard configuration
│   └── src/
│       └── inspector_node.cpp          # Base robot node
│
├── inspector_interfaces/           # Custom ROS 2 action definitions
│   └── action/
│       └── LocateTarget.action         # Goal/Result/Feedback contract
│
├── inspector_vision/               # Lifecycle-managed vision microservice
│   ├── src/
│   │   ├── target_locator.cpp          # LifecycleNode + Action Server + sensor fusion
│   │   └── locate_target_bt_node.cpp   # Nav2 BT plugin (shared library)
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
| BehaviorTree.CPP | 4.x |

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
  ros-jazzy-topic-tools \
  ros-jazzy-nav2-behavior-tree \
  ros-jazzy-behaviortree-cpp
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

### Launch (Single Command)

The entire system boots from a single master launch file. Internally, a `TimerAction` enforces a 12-second delay between Gazebo physics initialization and the rest of the stack:

```bash
ros2 launch inspector_bot master_bringup.launch.py
```

This single command orchestrates all five subsystems that previously required separate terminals:

| Subsystem | What it launches |
|---|---|
| **Gazebo Simulation** | `sim_robot.launch.py` — physics engine, robot spawn, sensor bridge (fires immediately) |
| **Nav2 Autonomy** | `bringup_launch.py` — A\* planner, DWB controller, AMCL, map server (fires after 12s delay) |
| **Twist Stamper** | Bridge node remapping `/cmd_vel` → `/diff_drive_controller/cmd_vel` (fires after 12s delay) |
| **RViz Dashboard** | Loads the frozen `production.rviz` configuration (fires after 12s delay) |
| **Vision Microservice** | `LifecycleNode` — boots in `unconfigured` state, ready for lifecycle transitions (fires after 12s delay) |

> **Tip:** Set an initial pose estimate in RViz (`2D Pose Estimate`) before sending navigation goals to resolve the AMCL localization prior. Then use `2D Goal Pose` to command the robot toward the cargo target.

> **Note:** The vision node launches as a `LifecycleNode`. It must be transitioned through `configure` → `activate` before the action server accepts goals. Nav2's Behavior Tree handles this automatically when using the custom nav tree.

---

## Tech Stack

```
ROS 2 Jazzy  ·  Gazebo Harmonic  ·  C++17  ·  Python 3  ·  Nav2
OpenCV (ArUco / AprilTag 36h11)  ·  PCL  ·  tf2_ros  ·  message_filters
rclcpp_lifecycle  ·  rclcpp_action  ·  BehaviorTree.CPP v4  ·  nav2_behavior_tree
ros2_control  ·  cv_bridge  ·  colcon / CMake  ·  AMCL  ·  SLAM Toolbox
```

---

## Future Scope

- **Docker Containerization** : Package the full simulation stack into a multi-stage Docker image with GPU passthrough for reproducible, single-command deployment.
- **MoveIt 2 Integration** : Extend the pipeline with a 6-axis manipulator arm consuming the `cargo_target` TF frame for autonomous pick-and-place operations.
- **Multi-Tag Tracking** : Generalize the vision node to track an array of AprilTag IDs simultaneously, broadcasting unique TF frames per target.
- **Lifecycle Manager Integration** : Wire the vision node into Nav2's `lifecycle_manager` for fully automated `configure` → `activate` transitions at system boot.
- **Depth Filtering & Outlier Rejection** : Integrate PCL statistical outlier removal and voxel downsampling for robust spatial extraction in noisy real-world sensor data.
- **CI/CD Pipeline** : Add `colcon test` with `ament_lint_auto` and integration tests via `launch_testing` for continuous validation.

---

## License

This project is provided for portfolio and demonstration purposes.
