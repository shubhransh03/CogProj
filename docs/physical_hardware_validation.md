# Physical Hardware & Live Sensor Verification Report (Phase 5L)

## 1. Executive Summary

This report documents the live, empirical verification of the physical rover hardware, sensor interfaces, and AI perception pipeline on the **Veerobot Beetle Bot** (Raspberry Pi 5 ARM64).

* **Hardware Verification Status:** **PASSED**
* **CSI Camera Driver & Stream:** **VERIFIED (30.0 FPS, 640x480 `bgra8`)**
* **RPLiDAR 2D Laser Scanner:** **VERIFIED (10.0 Hz, 505 scan points, Health OK)**
* **Real ONNX Ore Detection:** **VERIFIED (3 - 5 FPS inference, Tracked & Counted)**
* **Robot Actuation Safety:** **VERIFIED (Zero `/cmd_vel` publishers; stationary)**

---

## 2. Host System & Platform Verification

* **Host:** `beetlebot-117` (`192.168.0.117`)
* **Hardware Model:** Raspberry Pi 5 Model B Rev 1.1 (`/proc/device-tree/model`)
* **Architecture:** `aarch64` (ARMv8 64-bit)
* **Kernel:** `6.8.0-1065-raspi` (Ubuntu 24.04.3 LTS)
* **SoC Thermals:** `38.9°C` to `44.4°C` (Measured via `vcgencmd measure_temp`)
* **Throttling State:** `0x0` (No under-voltage, no thermal capping detected)
* **Available Memory:** 5.8 GiB available out of 7.8 GiB
* **Root Storage:** 46 GB free out of 58 GB (18% utilization)
* **User Permissions:** User `veerobot` verified in groups `dialout`, `video`, `render`, `spi`, `i2c`, and `gpio`.

---

## 3. Physical CSI Camera Subsystem Verification

### 3.1. Hardware Device Identification
Inspection of `/dev/media*` and kernel device trees confirmed:
* **Sensor Model:** OmniVision OV5647 MIPI CSI Camera (`ov5647@36`)
* **Bus / Pipeline Handler:** `rpi/pisp` on Broadcom BCM2712_D0 ISP
* **Device Nodes:** `/dev/media0` (CFE device) and `/dev/media2` (ISP device)
* **V4L2 Video Nodes:** 27 nodes active under `/dev/video*`

### 3.2. Live Stream Telemetry
Executing `camera_node` in headless mode yielded the following measured telemetry:
* **Topic:** `/camera/image_raw`
* **Resolution:** 640 x 480 pixels
* **Native Encoding:** `bgra8` (step: 2560 bytes, buffer size: 1,228,800 bytes)
* **Measured Rate:** **29.98 to 30.05 FPS** (10-frame sliding window):
  ```
  average rate: 29.984
      min: 0.032s max: 0.035s std dev: 0.00070s window: 10
  average rate: 30.055
      min: 0.033s max: 0.034s std dev: 0.00050s window: 10
  ```

### 3.3. Calibration State Audit
Inspection of `/camera/camera_info` revealed:
* **Calibration State:** `[UNCALIBRATED]`
* **Intrinsic Matrix $K$:** All zeros (`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`)
* **Cause:** Calibration YAML `/home/veerobot/.ros/camera_info/ov5647__base_axi_pcie_120000_rp1_i2c_80000_ov5647_36_640x480.yaml` does not exist on disk.
* **Architectural Handling:** CogProj's spatial association node honestly marks coordinates as `(UNCALIBRATED)` and does not fabricate inaccurate 3D positions in the absence of valid calibration.

---

## 4. Physical RPLiDAR Subsystem Verification

### 4.1. Hardware Device Identification
* **USB UART Bridge:** `Bus 002 Device 002: ID 10c4:ea60 Silicon Labs CP210x UART Bridge`
* **Device Nodes:** `/dev/ttyUSB0` and udev symlink `/dev/lyra_lidar -> ttyUSB0`
* **Permissions:** `crw-rw-rw- 1 root dialout` (Read/write accessible)
* **Configured Baud Rate:** 460800 baud

### 4.2. Driver Diagnostic Output
Launching `sllidar_node` from `sllidar_ros2` produced:
* **SDK Version:** ROS 2 SDK 1.0.1 / SLLidar SDK 2.1.0
* **Serial Number:** `8BCBE0F8C6E599D6B5E29FF6081E4D19`
* **Firmware Version:** `1.02` (Hardware Rev: `18`)
* **Health Status:** `0` (`SLLidar health status : OK.`)
* **Scan Mode:** `DenseBoost`, 5 kHz sample rate, 40.0 m max range, 10.0 Hz scan rate

### 4.3. Live LaserScan Telemetry
Subscribing to `/scan` produced active LaserScan messages:
* **Frame ID:** `lidar_link`
* **Scan Points:** 505 range measurements per revolution
* **Angular Range:** $-\pi$ to $+\pi$ ($-3.139$ to $+3.119$ rad, increment $0.0124$ rad)
* **Range Limits:** $0.05$ m to $40.0$ m
* **Scan Frequency:** 10.0 Hz (scan time: $0.1016$ s)

---

## 5. End-to-End Real ONNX Detection Pipeline Verification

Running `demo_showcase.launch.py detector_mode:=real` against the physical camera verified live AI inference on the Raspberry Pi 5:

1. **Ingestion & Format Normalization:**
   * `camera_input_node` received `bgra8` frames and normalized them to `bgr8` (step=1920).
2. **ONNX Inference:**
   * Loaded `Rover (2)/best.onnx` (42.8 MB) using `OreDetectorPlugin` (`models/ore_yolo/adapter.py`).
   * Classes: `resource`, `host_rock`.
   * Inference latency: ~200 - 300 ms on Raspberry Pi 5 CPU.
3. **Live Detection & Tracking:**
   * Detected ore target (`resource`, confidence 0.61, bounding box `[405, 23, 639, 368]`).
   * Hungarian tracker confirmed Track `#1`.
   * Counter incremented session total: `total_count: 1` (`resource: 1`).
4. **Annotated HUD Stream:**
   * `/cogproj/image_annotated` published continuously at **3 - 5 FPS**.
   * Foxglove Bridge automatically advertised and streamed all channels over WebSockets.

---

## 6. Safety & Stationary Verification

* **Actuation Topics Checked:** `/cmd_vel`, `/cmd_vel_nav`, `/cmd_vel_joy`, `lyra/armed`.
* **Observed Publisher Count:** **0 active publishers** throughout all diagnostics and pipeline executions.
* **Vehicle Movement:** Zero physical wheel motion; motors remained disarmed.

