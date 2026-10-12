# CogProj SLAM-Integrated Ore Detection & Counting Architecture

**Project:** CogProj — AI-Based Ore Detection and Counting Rover  
**Target Platform:** Veerobot Beetle Bot, Raspberry Pi 5 ARM64, Ubuntu 24.04.3 LTS, ROS 2 Jazzy  
**Workspace:** `/home/veerobot/CogProj`  
**Robot Stack:** `/home/veerobot/lyra_ws` (Read-only integration)  
**Status:** Software Implementation Complete (Hardware Deployment Pending Calibration)  

---

## 1. Architectural Overview

The integrated CogProj perception and mapping system unifies 2D deep-learning ore detection with 2D LiDAR SLAM mapping and temporal tracking on the Veerobot Beetle Bot.

The architecture comprises six decoupled subsystems:
1. **Sensor Ingestion:** Standard ROS 2 image streaming (`/camera/image_raw`, `bgra8` converted to `bgr8`), LiDAR scan streaming (`/scan`), and camera calibration (`/camera/camera_info`).
2. **Deep-Learning Perception:** Pluggable detector architecture supporting either synthetic mock detection or real YOLOv8s ONNX inference via OpenCV DNN on the Pi 5 CPU.
3. **Temporal Tracking:** IoU-based multi-object tracker with state hysteresis (`TENTATIVE`, `CONFIRMED`, `LOST`) and track ID persistence.
4. **Unique Session Counting:** Cumulative and per-class counter maintaining session counts from confirmed track transitions.
5. **Spatial Association & LiDAR Fusion:** Geometric projection engine that fuses planar LaserScan ranges with camera optical bounding boxes and projects 3D coordinates into the SLAM `map` frame using TF2.
6. **Unified Bringup & Visualization:** Coordinated launch system (`integrated_slam_ore_pipeline.launch.py`) with headless defaults, decoupled SLAM mapping option, and RViz-ready multi-layer visualization.

```mermaid
flowchart TD
    subgraph Sensors ["Hardware Sensors & Drivers"]
        CAM["CSI Camera Node\n/camera/image_raw\n/camera/camera_info"]
        LIDAR["LiDAR Driver\n/scan (sllidar)"]
        EKF["Robot EKF / Odometry\n/odometry/filtered\n/tf (odom -> base)"]
    end

    subgraph Perception ["CogProj 2D Perception Subsystem"]
        CIN["camera_input_node\n(BGRA8 to BGR8)"]
        DET["detection_node\n(YOLOv8s ONNX / Mock)"]
        TRK["tracking_node\n(State Hysteresis)"]
        CNT["counting_node\n(Unique Session Count)"]
        VIS["visualizer_node\n(Annotated 2D Stream)"]
    end

    subgraph SLAM ["Robot Mapping Subsystem"]
        STB["slam_toolbox\n(async_slam_toolbox_node)"]
        MAP["/map OccupancyGrid\n/tf (map -> odom)"]
    end

    subgraph Spatial ["CogProj Spatial Subsystem"]
        SAN["spatial_association_node\n(cogproj_spatial)"]
        PROJ["LidarCameraProjector\n- Intrinsics Validation\n- Scan-to-Camera Transform\n- Box Point Clustering"]
        TF2["TF2 Transform Buffer\nmap -> cam_optical -> lidar"]
    end

    subgraph Outputs ["User & Navigation Outputs"]
        SPO["/cogproj/spatial_ores\n(SpatialOreArray)"]
        MRK["/cogproj/spatial_markers\n(MarkerArray 3D)"]
        IMG["/cogproj/image_annotated\n/cogproj/image_annotated/compressed"]
        SUM["/cogproj/counts_summary\n(OreCountSummary)"]
        RVIZ["RViz2 Visualization"]
    end

    CAM -->|image_raw| CIN
    CAM -->|camera_info| SAN
    CIN -->|image_raw (bgr8)| DET
    CIN -->|image_raw| VIS
    DET -->|detections| TRK
    TRK -->|tracked_ores| CNT
    TRK -->|tracked_ores| VIS
    TRK -->|tracked_ores| SAN
    CNT -->|counts_summary| VIS
    CNT -->|counts_summary| SUM

    LIDAR -->|/scan| STB
    LIDAR -->|/scan| SAN
    EKF -->|/odometry/filtered| STB
    EKF -->|TF: odom -> base| TF2
    STB -->|/map| MAP
    STB -->|TF: map -> odom| TF2
    TF2 --> SAN

    SAN --> PROJ
    SAN --> SPO
    SAN --> MRK
    VIS --> IMG
    MAP --> RVIZ
    MRK --> RVIZ
    IMG --> RVIZ
```

---

## 2. Topic and Message Interface Flow

| Topic Name | Message Type | Publisher Node | QoS Profile | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_ros` | Sensor Data | Raw camera stream (`bgra8` on ov5647). |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `camera_ros` | Sensor Data | Camera intrinsic calibration matrix $K$ and resolution. |
| `/scan` | `sensor_msgs/LaserScan` | `sllidar_node` | Sensor Data | 2D LiDAR planar range scan in `lidar_link`. |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_filter_node` | Reliable | Wheel odometry + IMU fused state estimate. |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher`, `slam_toolbox`, EKF | Dynamic | Coordinate frame kinematic tree. |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | Transient Local | 2D occupancy grid SLAM map. |
| `/cogproj/image_raw` | `sensor_msgs/Image` | `camera_input_node` | Sensor Data | Converted `bgr8` camera feed. |
| `/cogproj/detections` | `cogproj_interfaces/DetectionArray` | `detection_node` | Sensor Data | 2D bounding boxes and class probabilities. |
| `/cogproj/tracked_ores` | `cogproj_interfaces/TrackedOreArray` | `tracking_node` | Sensor Data | Filtered tracks with confirmation states and IDs. |
| `/cogproj/counts_summary` | `cogproj_interfaces/OreCountSummary` | `counting_node` | Transient Local | Cumulative and per-class unique ore totals. |
| `/cogproj/spatial_ores` | `cogproj_interfaces/SpatialOreArray` | `spatial_association_node` | Sensor Data | 3D physical coordinates in map and base frames. |
| `/cogproj/spatial_markers` | `visualization_msgs/MarkerArray` | `spatial_association_node` | Reliable | 3D mapped spheres and text tags for RViz. |
| `/cogproj/image_annotated` | `sensor_msgs/Image` | `visualizer_node` | Sensor Data | 2D video feed with bounding boxes and HUD overlay. |

---

## 3. Coordinate Frames and Kinematic Tree

Based on [`beetlebot.urdf`](file:///home/veerobot/lyra_ws/src/beetlebot_description/urdf/beetlebot.urdf):

```
map
 └── odom                         (published by SLAM Toolbox)
      └── base_footprint          (published by EKF / robot_localization)
           └── base_link          (fixed joint: xyz="0 0 0.163", rpy="0 0 0")
                ├── lidar_link    (fixed joint: xyz="0.085 0.000 0.043", rpy="0 0 3.14159")
                ├── camera_link   (fixed joint: xyz="0.150 0.000 -0.075", rpy="0 0 0")
                │    └── camera_optical_link (fixed joint: xyz="0 0 0", rpy="-1.5708 0 -1.5708")
                └── imu_link      (fixed joint: xyz="0 0 -0.020", rpy="0 0 0")
```

### Frame Conventions
- **`lidar_link`:** X forward, Y left, Z up. Mounted with $\pi$ radians yaw rotation relative to `base_link`.
- **`camera_link`:** X forward, Y left, Z up.
- **`camera_optical_link`:** Standard ROS optical frame: **Z forward (optical axis / depth)**, **X right**, **Y down**.
- **`base_footprint`:** Ground-level footprint: X forward, Y left, Z up (ground plane $Z=0$).
- **`map`:** Global fixed SLAM navigation frame: X East/forward, Y North/left, Z up.

---

## 4. Detection-to-Map Spatial Association Engine

The spatial association engine ([`cogproj_spatial`](file:///home/veerobot/CogProj/src/cogproj_spatial)) connects 2D image detections to 3D map coordinates without fabricating artificial depths.

### Pinhole Projection Geometry
1. **Cartesian Scan Conversion:**
   Valid laser scan ranges $r_i$ at angles $\theta_i$ are converted to 3D Cartesian coordinates in `lidar_link`:
   $$\mathbf{P}_{\text{lidar}} = \begin{bmatrix} r_i \cos\theta_i \\ r_i \sin\theta_i \\ 0 \end{bmatrix}$$
2. **LiDAR-to-Camera Optical Transformation:**
   Using TF2 transform $\mathbf{T}_{\text{cam} \leftarrow \text{lidar}} = (\mathbf{R}, \mathbf{t})$:
   $$\mathbf{P}_{\text{cam}} = \mathbf{R} \mathbf{P}_{\text{lidar}} + \mathbf{t} = \begin{bmatrix} X_{\text{cam}} \\ Y_{\text{cam}} \\ Z_{\text{cam}} \end{bmatrix}$$
3. **Pinhole Projection:**
   Points with forward depth $Z_{\text{cam}} > Z_{\min}$ (where $Z_{\min} = 0.10$ m) are projected into pixel space $(u, v)$ using verified intrinsic matrix $K$:
   $$u = f_x \frac{X_{\text{cam}}}{Z_{\text{cam}}} + c_x, \quad v = f_y \frac{Y_{\text{cam}}}{Z_{\text{cam}}} + c_y$$
4. **Bounding Box Association & Clustering:**
   Points falling inside 2D detection bounding box $[x_{\min}, y_{\min}, x_{\max}, y_{\max}]$ are clustered. Outliers and background wall bleed are rejected by clustering around the median depth $Z_{\text{med}} \pm 0.35$ m.
5. **Map Transformation & Smoothing:**
   Inlier 3D centroids are transformed into the global `map` frame using $\mathbf{T}_{\text{map} \leftarrow \text{cam}}$:
   $$\mathbf{P}_{\text{map}} = \mathbf{R}_{\text{map}\leftarrow\text{cam}} \mathbf{P}_{\text{cam}} + \mathbf{t}_{\text{map}\leftarrow\text{cam}}$$
   Track positions are smoothed temporally across frames using an exponential moving average ($\alpha = 0.70$) to suppress range jitter.

### Zero-Fabrication Safety Guarantees
- If `CameraInfo` is missing, unpopulated, or contains non-positive focal lengths ($f_x \le 0, f_y \le 0$), the status is explicitly set to `UNCALIBRATED_CAMERA`.
- If no LiDAR scan is received or the scan timestamp is stale ($|\Delta t| > 0.50$ s), the status is set to `NO_LIDAR_SCAN`.
- If no LiDAR points fall inside the detection box, the status is set to `NO_LIDAR_POINTS`.
- If TF transforms cannot be looked up, the status is set to `TF_LOOKUP_FAILED`.
- **In all unlocalized conditions, the position fields remain zeroed (`0, 0, 0`), `is_localized = False`, and NO RViz 3D markers are published.**

---

## 5. Interface Changes & Data Definitions

Two typed ROS 2 message definitions were added to `cogproj_interfaces`:

### `SpatialOre.msg`
```
int32 track_id
int32 class_id
string class_name
float32 confidence
geometry_msgs/Point position_map
geometry_msgs/Point position_base
float32 estimated_distance
bool is_localized
string localization_status
```

### `SpatialOreArray.msg`
```
std_msgs/Header header
SpatialOre[] ores
uint32 localized_count
uint32 unlocalized_count
```

**Justification:** Provides a type-safe contract separating 2D image-space tracks from 3D physical world landmarks. Downstream path planners, costmaps, or exploration behaviors can directly consume map coordinates without parsing image or sensor messages.

---

## 6. Unified Bringup Launch Reference

**File:** [`src/cogproj_bringup/launch/integrated_slam_ore_pipeline.launch.py`](file:///home/veerobot/CogProj/src/cogproj_bringup/launch/integrated_slam_ore_pipeline.launch.py)

### Launch Arguments

| Argument | Default | Allowed Values | Description |
| :--- | :--- | :--- | :--- |
| `camera_input_topic` | `/camera/image_raw` | Topic string | Input camera topic. |
| `detector_mode` | `mock` | `mock`, `real` | Perception mode. `mock` generates synthetic detections for fast testing; `real` runs YOLOv8s ONNX model via OpenCV DNN. |
| `model_directory` | `/home/veerobot/CogProj/models/ore_yolo` | Path | Directory containing ONNX adapter and model config. |
| `confidence_threshold` | `0.50` | `0.0` – `1.0` | Minimum score for detection acceptance. |
| `enable_spatial_association`| `true` | `true`, `false` | Enables LiDAR-to-camera projection and map localization. |
| `start_slam` | `false` | `true`, `false` | Starts SLAM Toolbox async mapping node. Disabled by default to avoid conflict with existing robot SLAM. |
| `slam_params_file` | `/home/veerobot/lyra_ws/.../slam_toolbox.yaml` | Path | Path to SLAM Toolbox configuration file. |
| `start_rviz` | `false` | `true`, `false` | Starts RViz2 visualization GUI. Disabled by default for headless operation. |
| `rviz_config` | `cogproj_bringup/rviz/cogproj_slam_ore.rviz` | Path | RViz configuration file. |

---

## 7. Safety Boundary & Change Control

1. **Workspace Boundary:** Work is strictly confined to `/home/veerobot/CogProj`. The protected workspaces `/home/veerobot/lyra_ws`, `/home/veerobot/camera_ws`, and `/home/veerobot/rp_lidar` remain read-only and unmodified.
2. **Actuation Isolation:** The integrated launch contains ZERO motor drivers, hardware bridges, joystick nodes, teleoperation nodes, or Nav2 controller nodes.
3. **Zero Movement Commands:** No CogProj node or launch file advertises, publishes, or remaps to `/cmd_vel`, `/cmd_vel_nav`, or `/cmd_vel_joy`.
4. **Camera Process Protection:** The camera driver is treated as an external service; CogProj subscribes to `/camera/image_raw` without starting duplicate camera drivers or stopping existing camera processes.

---

## 8. Physical Sensor Dependencies & Field Calibration Checklist

Before spatial association can report physical coordinates on the physical robot in the field, the following hardware dependencies must be resolved:

| Dependency | Current State | Required Field Action |
| :--- | :--- | :--- |
| **Camera Intrinsic Calibration ($K$)** | **Uncalibrated (K=0)** | Calibrate the OV5647 camera using `camera_calibration` package with an $8 \times 6$ checkerboard. Save the YAML file to `~/.ros/camera_info/` so `camera_ros` publishes valid focal lengths ($f_x, f_y, c_x, c_y$). |
| **Extrinsic LiDAR-Camera Calibration** | **URDF Static Model** | Verify physical mount offsets ($X=0.15\text{m}, Z=-0.075\text{m}$) against manufactured Beetle Bot chassis. |
| **LiDAR Elevation Geometry** | **Planar at $Z \approx 20.6$ cm** | The Beetle Bot 2D LiDAR scans in a horizontal plane 20.6 cm above ground. Ground-level ore pebbles ($< 5$ cm) will NOT intersect the horizontal laser plane. Field specimens must have sufficient vertical profile ($\ge 15$ cm), or the LiDAR must be pitched forward, or 3D/depth sensors must be evaluated. |
| **Physical SLAM Prerequisites** | **Stopped** | Requires starting the base robot state (`base.launch.py`) and LiDAR (`lidar.launch.py`) from `lyra_ws` in a separate terminal before launching with `start_slam:=true`. |

---

## 9. Implementation vs. Simulation / Testing Status

| Component | Implementation Status | Verification Method |
| :--- | :--- | :--- |
| **Camera Input BGRA-to-BGR** | Production Ready | Tested against physical camera in Phase 5H. |
| **ONNX Detector Adapter** | Production Ready | Tested against physical camera (0.73 FPS) and offline images in Phase 5H. |
| **Multi-Object Tracker** | Production Ready | Verified with automated unit & scenario regression tests. |
| **Unique Session Counter** | Production Ready | Verified with automated unit & scenario regression tests. |
| **Spatial Projection Engine** | Production Ready | Verified with automated tests covering all geometric edge cases. |
| **Spatial Associator Node** | Production Ready | Tested in ROS 2 node lifecycle and synthetic message tests. |
| **Integrated Launch File** | Production Ready | Verified with automated argument and safety audit tests. |
| **RViz Configuration** | Production Ready | Configured with OccupancyGrid, LaserScan, Markers, and Camera overlays. |
| **Physical Hardware Integration** | **Ready for Review** | Architecture, tests, and safety boundaries verified; awaiting hardware testing authorization. |

