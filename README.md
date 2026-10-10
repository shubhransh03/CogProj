# CogProj

AI-Based Ore Detection and Counting Rover

CogProj is an AI-assisted perception and counting system developed for the Veerobot Beetle Bot robotic platform. It provides a modular, framework-independent perception pipeline built on ROS 2 Jazzy, specifically engineered to detect, track, and count unique ores during rover exploration missions.

## Platform & Environment

* **Target Hardware:** Veerobot Beetle Bot (Raspberry Pi 5, aarch64)
* **Operating System:** Ubuntu 24.04.3 LTS
* **ROS 2 Distribution:** Jazzy Jalisco
* **Workspace Isolation:** Self-contained within `/home/veerobot/CogProj`. Operates with strict isolation from base robot packages (`lyra_ws`, `rp_lidar`, `camera_ws`), creating zero actuation publishers (`/cmd_vel`, `/cmd_vel_nav`, `/cmd_vel_joy`) to prevent unauthorized vehicle movement.

---

## Architecture & Current Status (Phases 1–4 + Phase 5A Hardening)

The perception and counting pipeline processes camera frames into validated unique ore counts and real-time visual HUD overlays across five sequential layers:

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
                     │
                     ▼
          cogproj_visualization
     (/cogproj/image_annotated)
  (/cogproj/image_annotated/compressed)
                     │
                     ▼
               cogproj_viewer
       (Remote laptop / GUI display)
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
   * Reports active detector status (`DISABLED`, `MOCK`, `ERROR_NO_MODEL`, `REAL:<name>`) via parameter and property.
   * Validates detection bounding boxes and confidence bounds before publishing `DetectionArray` to `/cogproj/detections`.

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

7. **`cogproj_visualization`** (`ament_python`):
   * `visualizer_node`: Real-time headless perception annotator. Renders bounding boxes with persistent track IDs, confidence, tracking states (`CONFIRMED`, `TENTATIVE`, `LOST`), top status HUD bar (camera, detector, tracking, counter states), and bottom mission summary bar (total unique count, active tracks, per-class breakdown).
   * Publishes uncompressed annotated frames to `/cogproj/image_annotated` and bandwidth-efficient JPEG streams to `/cogproj/image_annotated/compressed`.
   * `cogproj_viewer`: Remote monitoring client node with OpenCV window display and headless fallback.

8. **`cogproj_bringup`** (`ament_python`):
   * Master YAML configuration (`config/cogproj_config.yaml`).
   * Launch configurations:
     * `camera.launch.py`: Camera ingestion only.
     * `detection.launch.py`: Camera ingestion + detection node.
     * `mock_pipeline.launch.py`: Camera + detection in mock mode.
     * `mock_full_pipeline.launch.py`: Complete 5-node perception pipeline in mock mode (camera, mock detection, tracking, counting, visualization).
     * `tracking_pipeline.launch.py`: Perception pipeline without visualizer (camera, detection, tracking, counting).
     * `full_perception.launch.py`: All 5 perception nodes.
     * `visualization.launch.py`: Visualization node only.

---

---

## Implemented Functionality vs. Runtime Verification Status

### 1. Implemented Software Components
- Modular 8-package ROS 2 Jazzy workspace foundation.
- Framework-agnostic detector plugin architecture (`BaseOreDetector` ABC, dynamic loader, `MockDetector`).
- Non-invasive camera frame ingestion and conversion node preserving timestamps and frame IDs.
- Deterministic multi-object tracking node with Hungarian bipartite association.
- Confirmation-gated unique ore counting node with session deduplication.
- Headless perception HUD annotator node and compressed image streaming.
- Remote viewer client node with OpenCV window rendering and headless fallback.
- Modular and full-pipeline launch configurations (7 launch files).

### 2. Unit-Tested & Verified Offline
- **78 passing automated unit tests** covering all packages.
- Synthetic frame image conversion, header preservation, and message encoding.
- Tracker lifecycle states (`TENTATIVE`, `CONFIRMED`, `LOST`), IoU matching, and distance gating.
- Multi-frame synthetic scenarios connecting tracker and counter (misses, reappearance, noise rejection).
- Annotator HUD rendering, label placement, and JPEG compression under synthetic inputs.
- Static safety audit confirming **zero actuation publishers** on `/cmd_vel*` across all nodes and launch files.

### 3. Hardware / Runtime Status (Not Yet Verified)
- **Live full-pipeline execution:** Running all 5 perception nodes simultaneously with the active camera stack on physical hardware has **not yet been verified**.
- **Cross-machine ROS 2 discovery:** Topic visibility between the Beetle Bot (Pi 5) and remote laptop/VM environments remains unverified.
- **Physical ore detection and counting:** Evaluating detection, tracking, and counting with real physical ore samples remains unverified.

### 4. Remaining Roadmap Phases
- **Phase 5B:** End-to-End Pipeline Validation with Live Camera feed and MockDetector.
- **Phase 5C:** Diagnostics and Operational Robustness (latency logging, starvation warning).
- **Phase 5D:** Performance Measurement baseline on Raspberry Pi 5 hardware.
- **Phase 5E:** Deployment Documentation and Operator Quickstart.
- **Phase 5F:** Trained Ore Model Integration (deferred until model weights are provided; see [`docs/model_integration_guide.md`](docs/model_integration_guide.md)).

---

## Building and Testing

Source ROS 2 Jazzy and build the workspace:

```bash
cd /home/veerobot/CogProj
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Run test suite:

```bash
source install/setup.bash
colcon test
colcon test-result --all --verbose
```

## Running the Mock Perception Pipeline

To run the full perception pipeline end-to-end without requiring trained model weights:

```bash
source install/setup.bash
ros2 launch cogproj_bringup mock_full_pipeline.launch.py
```
