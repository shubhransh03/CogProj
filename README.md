# CogProj

**AI-Based Ore Detection and Counting Rover**

CogProj is an AI-assisted perception, tracking, and spatial counting system developed for the **Veerobot Beetle Bot** robotic platform. Built on **ROS 2 Jazzy** and running on an embedded **Raspberry Pi 5 ARM64**, CogProj detects, tracks, and localizes unique ores during rover exploration missions.

---

## 1. System & Platform Architecture

* **Target Hardware:** Veerobot Beetle Bot (Raspberry Pi 5 Model B Rev 1.1, aarch64)
* **Operating System:** Ubuntu 24.04.3 LTS (Noble Numbat)
* **ROS 2 Distribution:** Jazzy Jalisco
* **DDS Configuration:** `ROS_DOMAIN_ID=155`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
* **Safety & Isolation:** Strictly self-contained within `/home/veerobot/CogProj`. Zero actuation publishers on `/cmd_vel*`, `/cmd_vel_nav`, or `/cmd_vel_joy`. Motors remain disarmed; rover operates in stationary / passive observation mode during all diagnostics.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       CogProj Perception Architecture                       │
└─────────────────────────────────────────────────────────────────────────────┘
  [Camera: /camera/image_raw]          [RPLiDAR: /scan]          [SLAM: /map]
             │                                 │                       │
             ▼                                 │                       │
      cogproj_camera                           │                       │
   (/cogproj/image_raw)                        │                       │
             │                                 │                       │
             ▼                                 │                       │
     cogproj_detection                         │                       │
   (/cogproj/detections)                       │                       │
             │                                 │                       │
             ▼                                 │                       │
      cogproj_tracking                         │                       │
   (/cogproj/tracked_ores)                     │                       │
             │                                 │                       │
             ├─────────────────────────────────┼───────────────────────┤
             ▼                                 ▼                       ▼
      cogproj_counting                  cogproj_spatial          cogproj_bringup
  (/cogproj/counts_summary)          (/cogproj/spatial_ores)    (demo_showcase)
             │                       (/cogproj/spatial_markers)        │
             ▼                                                         │
   cogproj_visualization                                               │
 (/cogproj/image_annotated)                                            │
(/cogproj/image_annotated/compressed)                                  │
             │                                                         │
             └─────────────────────────┬───────────────────────────────┘
                                       ▼
                             [foxglove_bridge]
                          (WebSocket Server: 8765)
                                       │
                                       ▼
                       Remote Foxglove Studio / Web UI
```

---

## 2. Implemented Packages

1. **`cogproj_interfaces`** (`ament_cmake`): Custom ROS 2 message definitions:
   * `Detection.msg`, `DetectionArray.msg`: 2D bounding boxes, classes, confidences.
   * `TrackedOre.msg`, `TrackedOreArray.msg`: Monotonic track IDs, lifecycle states (`TENTATIVE`, `CONFIRMED`, `LOST`), hits, age.
   * `OreCountSummary.msg`: Cumulative unique ore counts, per-class counts, active tracks.
   * `SpatialOre.msg`, `SpatialOreArray.msg`: 3D ore positions in `map` coordinate frame.
2. **`cogproj_detector_base`** (`ament_python`): Framework-independent `BaseOreDetector` interface, `DetectionResult`, dynamic plugin discovery loader, and built-in synthetic `MockDetector`.
3. **`cogproj_camera`** (`ament_python`): Ingests `bgra8`/`bgr8` frames from `/camera/image_raw`, standardizes timestamps and optical frame IDs to `/cogproj/image_raw`.
4. **`cogproj_detection`** (`ament_python`): Perception node executing either the synthetic `MockDetector` or the trained YOLO ONNX plugin (`models/ore_yolo/adapter.py`).
5. **`cogproj_tracking`** (`ament_python`): Hungarian bipartite matching engine with centroid Euclidean distance and bounding-box IoU gating.
6. **`cogproj_counting`** (`ament_python`): Confirmation-gated unique ore counter with session deduplication; publishes latched `OreCountSummary`.
7. **`cogproj_visualization`** (`ament_python`): Headless perception HUD annotator rendering bounding boxes, track IDs, detection HUD bar, and mission count overlay.
8. **`cogproj_spatial`** (`ament_python`): LiDAR-to-camera projection engine calculating 3D positions in `map` frame and publishing RViz Marker arrays.
9. **`cogproj_bringup`** (`ament_python`): Coordinated launch files (`demo_showcase.launch.py`, `foxglove_bridge.launch.py`, `integrated_slam_ore_pipeline.launch.py`) and diagnostics (`preflight_check`, `demo_status`, `demo_map_node`).

---

## 3. Quickstart & Deployment Scripts

### 3.1. Environment Setup
Source ROS 2 Jazzy, DDS settings, underlays, and overlays:
```bash
cd /home/veerobot/CogProj
source scripts/setup_environment.sh
```

### 3.2. Install Dependencies
Verify and install missing system and ROS packages:
```bash
./scripts/install_dependencies.sh
```

### 3.3. Build Workspace
Build all packages with symlink installs:
```bash
./scripts/build_workspace.sh
# Or build and immediately run the regression test suite:
./scripts/build_workspace.sh --test
```

### 3.4. Run Hardware Diagnostics
Inspect host thermals, storage, USB buses, serial permissions, and active ROS streams:
```bash
./scripts/hardware_diagnostics.sh
```

---

---

## 4. Unified System Operation (One-Command Startup)

CogProj provides a single, coordinated startup manager [`run_cogproj.sh`](file:///home/veerobot/CogProj/scripts/run_cogproj.sh) that non-invasively launches or reuses hardware drivers (CSI camera, RPLiDAR, robot URDF, safety gate), the perception pipeline, SLAM mapping, and the unified laptop web dashboard:

### 4.1. Coordinated System Commands
```bash
# Start complete integrated system with Real ONNX detector
./scripts/run_cogproj.sh

# Or start in synthetic demonstration mode (offline / mock)
./scripts/run_cogproj.sh --mode mock

# Inspect live runtime and hardware status
./scripts/run_cogproj.sh --status

# Stop all managed background components cleanly
./scripts/run_cogproj.sh --stop
# Or:
./scripts/stop_cogproj.sh
```

### 4.2. Laptop Connection & Unified Dashboard
1. Connect via VS Code Remote SSH or forward ports:
   ```bash
   ssh -L 8080:127.0.0.1:8080 -L 8765:127.0.0.1:8765 veerobot@192.168.0.117
   ```
2. Open **[http://localhost:8080](http://localhost:8080)** in your laptop browser to access the **CogProj Operator Console**:
   * **Live Stream:** Large 10+ FPS camera feed with real-time YOLO bounding boxes, class labels, and tracking IDs.
   * **SLAM Occupancy Map:** Canvas rendering of the SLAM map, LiDAR scan points, and rover orientation.
   * **Safe Controls:** Explicit mode selector (`STOPPED`, `MANUAL`, `AUTONOMOUS`).
   * **Keyboard Teleop:** W/A/S/D control with hardware speed clamps ($\le 0.10$ m/s linear, $\le 0.25$ rad/s angular), 0.3s watchdog, and Spacebar immediate stop.
   * **Mission Counts:** Real-time cumulative tallies for `resource` vs `host_rock`.
   * **Safety E-Stop:** Disarms motors and locks the system in `STOPPED` mode.

---

## 5. Verification & Automated Testing

All **159 automated tests** pass cleanly across the workspace:
* **146 colcon package tests** (`cogproj_interfaces`, `cogproj_detector_base`, `cogproj_camera`, `cogproj_detection`, `cogproj_tracking`, `cogproj_counting`, `cogproj_spatial`, `cogproj_visualization`, `cogproj_dashboard`, `cogproj_bringup`).
* **13 model adapter unit tests** (`models/ore_yolo/test_adapter.py`).

Execute all tests:
```bash
source scripts/setup_environment.sh
./scripts/build_workspace.sh --test
pytest models/ore_yolo/test_adapter.py
```

---

## 6. Project Documentation & Verification Reports

* **Operator Manual:** [`docs/final_operator_manual.md`](file:///home/veerobot/CogProj/docs/final_operator_manual.md) — Comprehensive guide for VS Code Remote SSH, dashboard navigation, teleoperation, and diagnostics.
* **Hardware Validation Report:** [`docs/final_hardware_validation.md`](file:///home/veerobot/CogProj/docs/final_hardware_validation.md) — Physical benchmarks, sensor frequencies, thermal metrics, and safety gate audits.
* **Final Completion Report:** [`docs/final_completion_report.md`](file:///home/veerobot/CogProj/docs/final_completion_report.md) — Detailed PASS / PARTIAL / BLOCKED audit across all 12 system objectives.
* **Deployment Manifest:** [`docs/deployment_manifest.md`](file:///home/veerobot/CogProj/docs/deployment_manifest.md) — Operating environment and package manifests.
