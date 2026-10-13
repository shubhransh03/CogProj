# Remote Visualization & Telemetry Guide (Phase 5L)

## 1. Overview & Architecture

Because the Veerobot Beetle Bot operates as a headless embedded platform (Raspberry Pi 5 ARM64), running desktop GUI applications like RViz2 or `rqt_image_view` directly on the robot causes unnecessary GPU/CPU contention and fails in SSH environments without active display servers (`qt.qpa.xcb: could not connect to display`).

CogProj solves this through a dedicated, lightweight **Remote Visualization Bridge** powered by `foxglove_bridge` (ROS 2 Jazzy, WebSocket transport).

```
┌─────────────────────────────────────────────────────────────┐
│             Beetlebot Hardware (192.168.0.117)             │
│                                                             │
│   [CSI Camera]      [RPLiDAR A1]         [SLAM Toolbox]     │
│         │                 │                     │           │
│         ▼                 ▼                     ▼           │
│  /camera/image_raw      /scan                 /map          │
│         │                 │                     │           │
│   [CogProj Pipeline: Detect, Track, Count, Annotate]        │
│         │                                                   │
│   /cogproj/image_annotated  /cogproj/counts_summary         │
│         │                 │                                 │
│         └────────┬────────┘                                 │
│                  ▼                                          │
│          [foxglove_bridge]                                  │
│       (WebSocket Server: 8765)                              │
└──────────────────┬──────────────────────────────────────────┘
                   │  WebSocket Stream
                   ▼
┌─────────────────────────────────────────────────────────────┐
│          Remote Client (Laptop / Workstation)               │
│                                                             │
│  • Foxglove Studio Desktop / Web (ws://localhost:8765)      │
│  • Image Panels (Annotated HUD, Real-time detections)       │
│  • 3D Grid (SLAM Occupancy, LaserScan, Ore Markers)         │
│  • Telemetry HUD (Session counts, active tracks table)      │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Launching the Remote Visualization Bridge

### 2.1. Standalone Bridge Launch
From the robot terminal or VS Code Remote SSH terminal:
```bash
source scripts/setup_environment.sh
ros2 launch cogproj_bringup foxglove_bridge.launch.py port:=8765 address:=127.0.0.1
```
* **Default Port:** `8765`
* **Default Address:** `127.0.0.1` (restricts listening to loopback for secure SSH tunneling)
* **Buffer Limit:** 10 MB

### 2.2. Launch via VS Code Remote SSH Tasks
In VS Code, press `Ctrl+Shift+P` -> **Tasks: Run Task** -> **10. Start Remote Visualization Bridge (Foxglove)**.

---

## 3. Remote Client Connection Methods

### Option A: Secure SSH Port Forwarding (Recommended)
This method allows secure telemetry streaming across firewalls and untrusted networks without exposing open TCP ports.

1. On your remote workstation/laptop, establish an SSH tunnel:
   ```bash
   ssh -L 8765:127.0.0.1:8765 veerobot@192.168.0.117
   ```
2. Open **Foxglove Studio** (desktop app or browser at [studio.foxglove.dev](https://studio.foxglove.dev)).
3. Click **Open connection** -> Select **Foxglove WebSocket**.
4. Enter URL: `ws://localhost:8765` and click **Open**.

### Option B: Direct Local Area Network (LAN) Connection
If your workstation and rover reside on the same private Wi-Fi network:
1. Start the bridge with address binding to all interfaces:
   ```bash
   ros2 launch cogproj_bringup foxglove_bridge.launch.py port:=8765 address:=0.0.0.0
   ```
2. Open Foxglove Studio and connect to:
   ```
   ws://192.168.0.117:8765
   ```

### Option C: Native ROS 2 DDS Peering (Workstation RViz2)
If your workstation runs Ubuntu with ROS 2 Jazzy:
1. Set DDS environment variables matching the robot:
   ```bash
   export ROS_DOMAIN_ID=155
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```
2. Launch RViz2 using the preconfigured CogProj display layout:
   ```bash
   rviz2 -d /home/veerobot/CogProj/src/cogproj_bringup/rviz/cogproj_slam_ore.rviz
   ```

---

## 4. Advertised Topics & Data Dictionary

| Topic Name | Message Type | Transport Rate | Description |
| :--- | :--- | :--- | :--- |
| `/cogproj/image_annotated` | `sensor_msgs/msg/Image` | 3 - 5 FPS (Real) / 30 FPS (Mock) | Full-resolution HUD image with bounding boxes, tracking labels, and status bar. |
| `/cogproj/image_annotated/compressed` | `sensor_msgs/msg/CompressedImage` | 3 - 5 FPS (Real) / 30 FPS (Mock) | Bandwidth-efficient JPEG stream for low-latency wireless observation. |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 30.0 FPS | Unmodified camera stream from hardware sensor (`bgra8`, 640x480). |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 30.0 FPS | Camera calibration parameters and intrinsic matrix $K$. |
| `/cogproj/detections` | `cogproj_interfaces/msg/DetectionArray` | Rate of Detector | 2D image bounding boxes, classes (`resource`, `host_rock`), confidences. |
| `/cogproj/tracked_ores` | `cogproj_interfaces/msg/TrackedOreArray` | Synchronous | Active tracks with persistent IDs and lifecycle states (`CONFIRMED`, `TENTATIVE`). |
| `/cogproj/counts_summary` | `cogproj_interfaces/msg/OreCountSummary` | Latched / On Update | Cumulative mission counts, unique detected ores, and active track counts. |
| `/cogproj/spatial_ores` | `cogproj_interfaces/msg/SpatialOreArray` | Synchronous | Spatial 3D ore positions in `map` or `base_footprint` coordinate frame. |
| `/cogproj/spatial_markers` | `visualization_msgs/msg/MarkerArray` | Synchronous | 3D visual markers (spheres and text labels) rendered in SLAM map. |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10.0 Hz | 360-degree LiDAR range data (505 points, 0.05 m to 40.0 m). |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Latch on Map Update | SLAM 2D occupancy grid map. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Dynamic / Static | Robot coordinate frames (`base_footprint`, `base_link`, `lidar_link`, `camera_optical_link`, `map`). |

---

## 5. Recommended Foxglove Studio Dashboard Layout

To build an intuitive mission control dashboard in Foxglove Studio:

1. **Top-Left Panel (Video Stream):**
   * Panel Type: **Image**
   * Topic: `/cogproj/image_annotated/compressed`
   * Displays the annotated bounding boxes, track IDs, detection HUD, and mission count overlay.
2. **Top-Right Panel (Telemetry Indicators):**
   * Panel Type: **State Transitions** or **Plot / Indicator**
   * Topic: `/cogproj/counts_summary`
   * Plots `total_count` and displays `active_track_count`.
3. **Bottom-Left Panel (3D Spatial Environment):**
   * Panel Type: **3D**
   * Topics to enable:
     * LaserScan: `/scan` (Color by distance)
     * Map: `/map` (Occupancy grid)
     * Markers: `/cogproj/spatial_markers` (3D ore markers)
     * Transforms: `map`, `odom`, `base_footprint`, `camera_optical_link`, `lidar_link`
4. **Bottom-Right Panel (Log Feed):**
   * Panel Type: **Log**
   * Topic: `/rosout`
   * Displays pipeline diagnostic warnings, starvation alerts, and detection timings.

