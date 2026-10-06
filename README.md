# CogProj

AI-Based Ore Detection and Counting Rover

CogProj is an AI-assisted perception and counting system developed for the Veerobot Beetle Bot robotic platform. It provides a modular, framework-independent perception pipeline built on ROS 2 Jazzy, specifically engineered to detect, track, and count unique ores during rover exploration missions.

## Platform & Environment

* **Target Hardware:** Veerobot Beetle Bot (Raspberry Pi 5, aarch64)
* **Operating System:** Ubuntu 24.04.3 LTS
* **ROS 2 Distribution:** Jazzy Jalisco
* **Workspace Isolation:** Self-contained within `/home/veerobot/CogProj`. Operates with strict isolation from base robot packages (`lyra_ws`, `rp_lidar`, `camera_ws`), creating zero actuation publishers (`/cmd_vel`, `/cmd_vel_nav`, `/cmd_vel_joy`) to prevent unauthorized vehicle movement.

---

## Architecture & Current Status (Phases 1–3)

The perception and counting pipeline processes camera frames into validated unique ore counts across four sequential layers:

```
[Camera Source: /pi_camera/image_raw]
                     │
                     ▼
             cogproj_camera
       (/cogproj/image_raw)
                     │
                     ▼
            cogproj_detection
       (/cogproj/detections)
                     │
                     ▼
            cogproj_tracking
      (/cogproj/tracked_ores)
                     │
                     ▼
            cogproj_counting
     (/cogproj/counts_summary)
```

### Implemented Packages

1. **`cogproj_interfaces`** (`ament_cmake`):
   * `Detection.msg` & `DetectionArray.msg`: Standardized 2D bounding box and confidence representations.
   * `TrackedOre.msg` & `TrackedOreArray.msg`: Persistent track IDs, lifecycle states (`TENTATIVE`, `CONFIRMED`, `LOST`), hits, age, and missed frame counts.
   * `OreCountSummary.msg`: Cumulative mission count, per-class counts, and active track metrics.

2. **`cogproj_detector_base`** (`ament_python`):
   * Framework-independent `BaseOreDetector` abstract interface (`load`, `predict`, `get_metadata`, `unload`).
   * Standardized `DetectionResult` dataclass with boundary and coordinate validation.
   * Dynamic plugin loader supporting external detector packages via runtime spec loading.
   * Built-in `MockDetector` for synthetic offline verification without machine learning runtimes.

3. **`cogproj_camera`** (`ament_python`):
   * `camera_input_node`: Safely subscribes to `/pi_camera/image_raw` using SensorData QoS (`BEST_EFFORT`, `VOLATILE`, depth 5).
   * Converts frames via `cv_bridge` to BGR8 without altering pixels.
   * Preserves exact source header timestamps and `camera_optical_link` frame ID onto `/cogproj/image_raw`.

4. **`cogproj_detection`** (`ament_python`):
   * `detection_node`: Subscribes to `/cogproj/image_raw` and executes detection.
   * Safe operational modes:
     * *Disabled (Default):* Zero inference overhead, dormant subscriber.
     * *Test/Mock Mode:* Ingests camera frames and outputs synthetic detections via `MockDetector`.
     * *Pluggable Real Mode:* Loads external model plugin dynamically when configured.
   * Publishes `DetectionArray` to `/cogproj/detections`.

5. **`cogproj_tracking`** (`ament_python`):
   * `tracking_node`: Deterministic 2D image-space multi-object tracker.
   * Bipartite matching via Hungarian algorithm (`scipy.optimize.linear_sum_assignment`).
   * Gated cost function using normalized centroid Euclidean distance, bounding-box IoU, and hard class ID consistency gates.
   * Monotonic persistent `track_id` assignment with temporal confirmation hysteresis (default 3 hits) and missed-frame retention (default 5 frames).
   * Publishes `TrackedOreArray` to `/cogproj/tracked_ores`.
   * Includes architectural extension hooks for future odometry-based ego-motion compensation.

6. **`cogproj_counting`** (`ament_python`):
   * `counting_node`: Confirmation-gated cumulative unique ore counter.
   * Ignores `TENTATIVE` tracks; increments total and per-class counts only when a track transitions to `CONFIRMED`.
   * Enforces session deduplication using unique track ID sets, preventing recount of the same ore across multiple frames or temporary occlusions.
   * Publishes latched `OreCountSummary` to `/cogproj/counts_summary` using state/summary QoS (`RELIABLE`, `TRANSIENT_LOCAL`, depth 10).

7. **`cogproj_bringup`** (`ament_python`):
   * Master YAML configuration (`config/cogproj_config.yaml`).
   * Launch configurations for modular and complete pipelines:
     * `camera.launch.py`: Camera ingestion only.
     * `detection.launch.py`: Camera ingestion + detection node.
     * `mock_pipeline.launch.py`: End-to-end synthetic detection harness.
     * `tracking_pipeline.launch.py`: Full perception pipeline (camera + detection + tracking + counting).

---

## Implemented Functionality vs. Future Roadmap

### Currently Implemented & Verified
- Modular workspace foundation and custom ROS 2 message IDLs.
- Framework-agnostic detector plugin architecture.
- Non-invasive camera frame ingestion and conversion pipeline.
- Deterministic multi-object tracking with Hungarian association.
- Confirmation-gated unique ore counting and deduplication logic.
- 51 passing automated unit tests covering all components and synthetic integration scenarios.

### Future Work (Not Yet Implemented)
- **Real Ore Model Integration:** Loading user-trained weights once provided (Phase 6).
- **Physical Camera End-to-End Test:** Verification on physical camera feed once camera stack is launched by user.
- **Laptop Visualization:** Annotated HUD streams, RViz markers, or web interface (Phase 4).
- **3D Spatial Localization:** Ground plane raycasting and map-frame ore coordinate estimation (Phase 5).
- **SLAM & Nav2 Autonomous Exploration:** Integration with robot navigation and mapping (Phase 7).

---

## Building and Testing

Source ROS 2 Jazzy and build the workspace:

```bash
cd /home/veerobot/CogProj
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Run test suite (51 tests):

```bash
source install/setup.bash
colcon test
colcon test-result --all --verbose
```
