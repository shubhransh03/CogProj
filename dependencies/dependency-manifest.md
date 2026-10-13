# CogProj Software & Hardware Dependency Manifest

**Project:** CogProj — AI-Based Ore Detection and Counting Rover  
**Target Platform:** Veerobot Beetle Bot, Raspberry Pi 5 (ARM64 / aarch64), Ubuntu 24.04.3 LTS (Noble), ROS 2 Jazzy  
**Workspace:** `/home/veerobot/CogProj`  
**Date:** 2026-10-13  
**Status:** Verified & Resolved  

---

## 1. System Environment & Platform Constraints

| Property | Value | Notes |
| :--- | :--- | :--- |
| **Hostname** | `beetlebot-117` | IP: `192.168.0.117` |
| **Operating System** | Ubuntu 24.04.3 LTS (Noble Numbat) | Kernel: `Linux 6.8.0-1065-raspi` |
| **CPU Architecture** | `aarch64` (ARM64) | Broadcom BCM2712 Quad-Core Cortex-A76 @ 2.4 GHz |
| **Memory** | 8 GB LPDDR4X | Sufficient for parallel perception and model inference |
| **ROS 2 Distribution** | Jazzy Jalisco | Standard LTS release for Ubuntu 24.04 |
| **Domain ID (`ROS_DOMAIN_ID`)** | `155` | Isolated robot mesh domain |
| **RMW Implementation** | `rmw_fastrtps_cpp` | Low-latency intra-host and DDS network streaming |

---

## 2. Complete Dependency Inventory

Every dependency required to build, test, execute, remotely observe, and interface with physical sensors on the Beetle Bot rover is categorized below:

| Package Name | Category | Status | Installed Version | Source / Prefix Path | Function & Role |
| :--- | :--- | :---: | :--- | :--- | :--- |
| **`ros-jazzy-rclpy`** | ROS Core | Required | `7.1.6` | `/opt/ros/jazzy` | Python ROS 2 client library for all perception nodes. |
| **`ros-jazzy-std-msgs`** | ROS Interfaces | Required | System | `/opt/ros/jazzy` | Basic message primitives (`Header`, `String`, etc.). |
| **`ros-jazzy-sensor-msgs`** | ROS Interfaces | Required | System | `/opt/ros/jazzy` | Camera feeds (`Image`, `CompressedImage`, `CameraInfo`), LiDAR (`LaserScan`), and IMU (`Imu`). |
| **`ros-jazzy-geometry-msgs`** | ROS Interfaces | Required | System | `/opt/ros/jazzy` | Coordinate points (`Point`, `Pose`, `TransformStamped`) used by spatial interfaces. |
| **`ros-jazzy-nav-msgs`** | ROS Interfaces | Required | System | `/opt/ros/jazzy` | Odometry (`Odometry`) and SLAM maps (`OccupancyGrid`). |
| **`ros-jazzy-visualization-msgs`** | ROS Interfaces | Required | System | `/opt/ros/jazzy` | 3D visual markers (`Marker`, `MarkerArray`) for RViz spatial localization displays. |
| **`ros-jazzy-cv-bridge`** | Vision Pipeline | Required | `4.1.0` | `/opt/ros/jazzy` | Fast conversions between ROS 2 Image messages and OpenCV numpy arrays (supports `bgra8` and `bgr8`). |
| **`python3-opencv`** | Vision / ML | Required | `4.6.0+dfsg` | `/usr/lib/python3/dist-packages` | OpenCV core image processing and OpenCV DNN engine (`cv2.dnn.readNetFromONNX`) executing YOLOv8s. |
| **`python3-numpy`** | Numerical Math | Required | `1.26.4` | `/usr/lib/python3/dist-packages` | Matrix transformations, pinhole projection geometry, and array slicing. |
| **`python3-yaml`** | Configuration | Required | `6.0.1` | `/usr/lib/python3/dist-packages` | YAML parameter parsing for `cogproj_config.yaml` and model adapter `config.yaml`. |
| **`ros-jazzy-tf2-ros`** | Kinematics | Required | `0.36.16` | `/opt/ros/jazzy` | TF2 listener and transform buffer for projecting sensor data across frames (`map <- odom <- base <- lidar <- cam`). |
| **`ros-jazzy-tf2-geometry-msgs`** | Kinematics | Required | System | `/opt/ros/jazzy` | Point and vector transformation conversions for TF2. |
| **`ros-jazzy-slam-toolbox`** | Mapping / SLAM | Required (Safe) | `2.8.5` | `/opt/ros/jazzy` | 2D LiDAR SLAM mapping node (`async_slam_toolbox_node`). Opt-in; disabled by default. |
| **`ros-jazzy-nav2-lifecycle-manager`** | Mapping / SLAM | Required (Safe) | System | `/opt/ros/jazzy` | Lifecycle state transitions for SLAM Toolbox. |
| **`ros-jazzy-foxglove-bridge`** | Remote Vis | Required | `3.6.0` | `/opt/ros/jazzy` | High-performance WebSocket bridge (port 8765) for remote browser and Foxglove Studio visualization over SSH port forwarding. |
| **`ros-jazzy-rviz2`** | Visualization | Optional (Desktop) | `14.1.23` | `/opt/ros/jazzy` | 3D multi-layer visualization GUI for operator desktop workstations. |
| **`ros-jazzy-rqt-image-view`** | Visualization | Optional (Desktop) | `1.3.0` | `/opt/ros/jazzy` | 2D image viewer for raw and annotated camera feeds over local ROS domain mesh. |
| **`python3-colcon-common-extensions`** | Build Tools | Required | `0.3.0-100` | `/usr/lib/python3/dist-packages` | Build orchestration tool for ament and cmake packages. |
| **`python3-rosdep`** | Dependency Tools | Required | `0.26.0` | `/usr/lib/python3/dist-packages` | Automated dependency resolver across ROS packages. |
| **`python3-pytest`** | Test Harness | Required | `7.4.4` | `/usr/lib/python3/dist-packages` | Automated test runner for Python unit and integration suites. |

---

## 3. External Robot Underlay Workspaces (Read-Only)

CogProj interfaces with pre-existing robot hardware drivers installed in the host environment:

| Workspace Path | Package Name | Role in Beetle Bot Stack | Integration Mode |
| :--- | :--- | :--- | :--- |
| `/home/veerobot/lyra_ws/install` | `camera_ros` | Hardware CSI camera driver (`ov5647` sensor) publishing `/camera/image_raw`. | Consumed externally via topic. Do not duplicate driver. |
| `/home/veerobot/lyra_ws/install` | `sllidar_ros2` | RPLiDAR A1/A2 2D laser scanner driver publishing `/scan`. | Consumed externally via topic or sensor bringup. |
| `/home/veerobot/lyra_ws/install` | `beetlebot_description` | Robot URDF model defining kinematic joints (`base_footprint`, `base_link`, `lidar_link`, `camera_optical_link`). | Audited read-only for TF tree frames and dimensions. |
| `/home/veerobot/lyra_ws/install` | `lyra_slam` | SLAM Toolbox configuration (`slam_toolbox.yaml`). | Referenced read-only by launch files. |

---

## 4. Reproducible Dependency Resolution Command

All package dependencies can be verified or installed using standard ROS 2 tooling:

```bash
# 1. Update package indices (ROS 2 repository)
sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/ros2.list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"

# 2. Install apt package dependencies
sudo apt-get install -y --no-install-recommends $(cat dependencies/apt-packages.txt | grep -v '^#' | tr '\n' ' ')

# 3. Verify rosdep resolution across workspace
rosdep check --from-paths src --ignore-src --rosdistro jazzy
```

