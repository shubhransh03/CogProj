# CogProj Deployment & System Manifest (Phase 5L)

## 1. Target Hardware & Platform Environment

* **Target Hostname:** `beetlebot-117`
* **Static / Assigned IPv4:** `192.168.0.117`
* **Architecture:** `aarch64` (64-bit ARM Cortex-A76)
* **Processor / SoC:** Broadcom BCM2712 (4 cores @ up to 2.4 GHz)
* **Hardware Model:** Raspberry Pi 5 Model B Rev 1.1
* **RAM:** 8 GB LPDDR4X (7.8 GiB visible, ~5.8 GiB available)
* **Storage:** 64 GB MicroSD (`/dev/mmcblk0p2`, 58 GB capacity, 46 GB free, 18% used)
* **Base OS:** Ubuntu 24.04.3 LTS (Noble Numbat)
* **Linux Kernel:** `6.8.0-1065-raspi` (SMP PREEMPT_DYNAMIC)

---

## 2. Middleware & Networking Stack

* **ROS Distribution:** ROS 2 Jazzy Jalisco
* **ROS Domain ID:** `155` (Configured across all nodes and launch environments)
* **RMW Implementation:** `rmw_fastrtps_cpp` (DDS)
* **Communication Isolation:** Headless rover execution; no external GUI dependencies; WebSocket streaming over port 8765.
* **Actuation Isolation:** All CogProj perception nodes strictly omit publishers on `/cmd_vel*`, `/cmd_vel_nav`, and `/cmd_vel_joy`. Motors remain disarmed.

---

## 3. Peripheral Hardware & Interface Manifest

| Peripheral | Interface / Bus | System Device Node | Driver Package | Baud / Rate | Health / Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **CSI Camera** | MIPI CSI-2 (PiSP BCM2712_D0) | `/dev/video*`, `/dev/media0`, `/dev/media2` | `camera_ros` | 30.0 FPS, 640x480 `bgra8` | **Verified Online** (OV5647 sensor) |
| **2D Laser LiDAR** | USB 2.0 (CP210x UART) | `/dev/ttyUSB0` (`/dev/lyra_lidar`) | `sllidar_ros2` | 460800 baud, 10 Hz (505 points) | **Verified Online** (Health: OK) |
| **Robot Base / STM32** | Serial UART / SPI | `/dev/ttyACM*` / Internal | `lyra_bridge` (optional) | Stationary / Passive | Disarmed / Zero motion |
| **WebSockets Bridge** | TCP / Loopback & LAN | `127.0.0.1:8765` / `0.0.0.0:8765` | `foxglove_bridge` | 10 MB buffer limit | **Verified Online** |

---

## 4. Software Package Manifest

### 4.1. CogProj Perception Workspace (`/home/veerobot/CogProj`)

1. **`cogproj_interfaces`** (`ament_cmake`):
   * ROS 2 custom message definitions: `Detection.msg`, `DetectionArray.msg`, `TrackedOre.msg`, `TrackedOreArray.msg`, `OreCountSummary.msg`, `SpatialOre.msg`, `SpatialOreArray.msg`.
2. **`cogproj_detector_base`** (`ament_python`):
   * Framework-independent `BaseOreDetector` ABC, `DetectionResult`, dynamic plugin discovery loader, and zero-dependency `MockDetector`.
3. **`cogproj_camera`** (`ament_python`):
   * `camera_input_node`: Ingests `bgra8`/`bgr8` frames from `/camera/image_raw`, standardizes headers and optical frame IDs to `/cogproj/image_raw`.
4. **`cogproj_detection`** (`ament_python`):
   * `detection_node`: Runs pluggable ore detection (`mock` synthetic generator or `real` ONNX runtime plugin).
5. **`cogproj_tracking`** (`ament_python`):
   * `tracking_node`: Hungarian bipartite matching engine with spatial distance and IoU gating; persistent track ID lifecycle management.
6. **`cogproj_counting`** (`ament_python`):
   * `counting_node`: Confirmation-gated unique ore counter with session deduplication; publishes latched `OreCountSummary`.
7. **`cogproj_visualization`** (`ament_python`):
   * `visualizer_node`: Headless perception HUD annotator rendering bounding boxes, track IDs, HUD status bar, and mission count overlay.
   * `cogproj_viewer`: Remote client monitoring window.
8. **`cogproj_spatial`** (`ament_python`):
   * `spatial_association_node`: Projects 2D bounding boxes with LiDAR `/scan` ranges, computes TF ray projection to `map` frame, and outputs 3D RViz Marker arrays.
9. **`cogproj_bringup`** (`ament_python`):
   * Master configuration, launch pipelines (`demo_showcase.launch.py`, `foxglove_bridge.launch.py`, `integrated_slam_ore_pipeline.launch.py`), and diagnostics (`preflight_check`, `demo_status`, `demo_map_node`).

### 4.2. Base Robot Platform Underlay (`/home/veerobot/lyra_ws`)

* `beetlebot_description`: Robot URDF chassis, joint tree, and sensor transform frames.
* `camera_ros`: Libcamera ROS 2 hardware driver component.
* `sllidar_ros2`: RPLiDAR SDK driver and ROS 2 publisher.
* `lyra_slam`: SLAM Toolbox configuration for stationary mapping.

---

## 5. Dependency Manifest & Reproducible Setup

### 5.1. System APT Dependencies (`dependencies/apt-packages.txt`)
* `ros-jazzy-foxglove-bridge` (v3.6.0-1noble)
* `ros-jazzy-cv-bridge`
* `ros-jazzy-image-transport`
* `ros-jazzy-compressed-image-transport`
* `ros-jazzy-slam-toolbox`
* `ros-jazzy-tf2-ros`
* `ros-jazzy-tf2-geometry-msgs`
* `ros-jazzy-visualization-msgs`
* `libcamera-dev`

### 5.2. Python Runtimes (`dependencies/python-requirements.txt`)
* `onnxruntime` (v1.20+ aarch64)
* `numpy` (v1.26.4)
* `scipy` (v1.13.1)
* `opencv-python` / `python3-opencv` (v4.6.0+)
* `pyyaml` (v6.0.1)
* `pytest` (v8.1.1+)

### 5.3. Reproducible Scripts
* `scripts/install_dependencies.sh`: Installs all required apt and rosdep packages idempotently.
* `scripts/setup_environment.sh`: Sources ROS 2 Jazzy, exports `ROS_DOMAIN_ID=155`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, underlay (`lyra_ws`), and overlay (`CogProj`).
* `scripts/build_workspace.sh`: Staged colcon build with symlink install and automated regression test execution.
* `scripts/hardware_diagnostics.sh`: Comprehensive read-only hardware, bus, permissions, sensor topic, and safety inspection.

---

## 6. Remote Development Integration (VS Code Remote SSH)

* **`.vscode/tasks.json`:** 12 structured development tasks running in dedicated foreground terminal tabs:
  1. Check ROS and Package Environment
  2. Install Missing Project Dependencies
  3. Build CogProj Workspace
  4. Run Hardware Preflight Check
  5. Inspect Camera Status
  6. Inspect LiDAR, Odometry and IMU
  7. Inspect TF and Map Status
  8. Start Mock Demonstration
  9. Start Real ONNX Detector Pipeline
  10. Start Remote Visualization Bridge (Foxglove)
  11. Run Safe SLAM-Only Integration Check
  12. Run Full Regression Tests
* **`.vscode/launch.json`:** Debugging configurations for perception, tracking, counting, visualization, and spatial mapper nodes.
* **`.vscode/extensions.json`:** Recommended extensions for ROS, Python, C++, and Foxglove Studio.

