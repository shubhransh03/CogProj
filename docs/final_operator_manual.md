# CogProj Operator Manual: Unified Laptop Console & Rover Operations

**Document Version:** 1.0.0  
**Phase:** FINAL INTEGRATED ROVER SYSTEM  
**Platform:** Veerobot Beetle Bot (Raspberry Pi 5 ARM64, Ubuntu 24.04 LTS, ROS 2 Jazzy Jalisco)  
**Target Workspace:** `/home/veerobot/CogProj`  

---

## 1. System Overview & Architecture

CogProj is a complete, self-contained robotics software platform integrating real-time computer vision, multi-object tracking, spatial ore counting, SLAM mapping, and safe motion control on the physical Veerobot Beetle Bot.

The rover is operated headlessly from an operator's laptop through VS Code Remote SSH and a unified web-based application: the **CogProj Operator Console** running on HTTP port `8080`.

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│                            CogProj Operator Console                              │
│                            (http://localhost:8080)                               │
└──────────────────────────────────────┬───────────────────────────────────────────┘
                                       │ WebSocket (JSON Telemetry & Control)
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────────┐
│              cogproj_dashboard / DashboardGatewayNode (Port 8080)                │
├─────────────────────────┬───────────────────────────────┬────────────────────────┤
│     Perception Feeds    │       Control Supervisor      │      Mapping & TF      │
│  • /cogproj/image_annot │  • Mode: STOPPED/MANUAL/AUTO  │  • /scan (RPLiDAR A1)  │
│  • /cogproj/detections  │  • Limits: <=0.1m/s, <=0.25r/s│  • /map (SLAM Toolbox) │
│  • /cogproj/tracked_ores│  • Watchdog: 300ms timeout    │  • /tf, /tf_static     │
│  • /cogproj/counts_sum  │  • Target: /cmd_vel_joy       │  • Action: Nav2        │
└─────────────────────────┴───────────────┬───────────────┴────────────────────────┘
                                          │ Pre-Gate Topic
                                          ▼
                         ┌─────────────────────────────────┐
                         │   lyra_cmd_vel_gate (Safety)    │
                         └────────────────┬────────────────┘
                                          │ Post-Gate Topic (/cmd_vel)
                                          ▼
                         ┌─────────────────────────────────┐
                         │   lyra_bridge (Motor Control)   │
                         └─────────────────────────────────┘
```

---

## 2. Quickstart: One-Command Startup

To launch the complete robot system (hardware drivers, real ONNX ore perception, SLAM mapping, and web console):

### Step 1: Open VS Code Remote SSH Terminal
Connect to `beetlebot-117` (`192.168.0.117`) via VS Code Remote SSH. Open a terminal in `/home/veerobot/CogProj`.

### Step 2: Start the System
```bash
cd /home/veerobot/CogProj
./scripts/run_cogproj.sh
```

The startup manager automatically:
1. Sources ROS 2 Jazzy, DDS Domain ID 155 (`rmw_fastrtps_cpp`), external robot underlays, and the CogProj workspace.
2. Checks dependencies (`python3-opencv` with DNN, `numpy`, `tornado`).
3. Reuses existing camera or LiDAR drivers if already active, or starts them headlessly.
4. Spawns the BeetleBot URDF TF kinematic publisher.
5. Launches `lyra_cmd_vel_gate` to guarantee safety isolation.
6. Starts `async_slam_toolbox_node` for SLAM mapping.
7. Launches the full CogProj perception pipeline (`YOLOv8s_Ore_Detector` ONNX model).
8. Launches the `DashboardGatewayNode` web server on port `8080` and the Foxglove Bridge on port `8765`.

---

## 3. Remote Laptop Access & Port Forwarding

The Beetle Bot operates without a physical monitor or keyboard. All visualization and control occur through the laptop browser.

### Option A: Automatic Port Forwarding via VS Code (Recommended)
1. In VS Code, navigate to the **Ports** panel (`Ctrl+Shift+P` -> **Ports: Focus on Ports View**).
2. Forward the following two ports:
   * **Port 8080** — CogProj Operator Console (HTTP/WebSocket)
   * **Port 8765** — Foxglove Studio Bridge (WebSocket)

### Option B: SSH Local Port Forwarding from Terminal
If connecting from a standard laptop terminal outside VS Code:
```bash
ssh -L 8080:127.0.0.1:8080 -L 8765:127.0.0.1:8765 veerobot@192.168.0.117
```

---

## 4. Operator Console Guide (`http://localhost:8080`)

Open **`http://localhost:8080`** in Google Chrome, Firefox, or Edge.

### 4.1. Top Status Header
* **Connection Status:** Shows `CONNECTED` (green badge) when the real-time WebSocket stream is healthy.
* **Active Mode Badge:** Displays the current operating state:
  * `MODE: STOPPED` (Red) — Motors disarmed, zero velocity permitted.
  * `MODE: MANUAL` (Green) — Keyboard teleoperation active.
  * `MODE: AUTONOMOUS` (Blue) — Nav2 point-and-click navigation active.
* **Emergency Stop Button (`🚨 EMERGENCY STOP`):** Immediately engages zero velocity, disarms motors, cancels active navigation goals, and locks the system into `STOPPED` mode.

### 4.2. Camera & Perception Viewport (Left Panel)
* Displays a live video feed streaming at 10+ FPS directly from the physical OmniVision OV5647 CSI camera.
* Overlays real-time AI detections:
  * **Cyan Bounding Boxes:** `resource` ore targets with model confidence percentage.
  * **Orange Bounding Boxes:** `host_rock` non-resource targets.
  * **Track Badges:** Unique persistent tracker IDs (`ID #...`) and confirmation status (`CONFIRMED`, `TENTATIVE`).
  * **HUD Header:** Active detector mode (`REAL ONNX`), frame rate, and latency.

### 4.3. SLAM Map & Obstacle Viewport (Center Panel)
* Renders a top-down 2D canvas displaying:
  * **Occupancy Grid Map:** Real-time SLAM walls (dark) and free explored space (white).
  * **LiDAR Scan Obstacles:** Real-time RPLiDAR obstacle points (lime green dots) around the rover.
  * **Rover Position & Heading:** Red triangle representing the rover footprint and heading vector.
* **Point-and-Click Autonomous Navigation:** In `AUTONOMOUS` mode, click any free cell on the canvas to dispatch a navigation goal. A green goal pin will appear.

### 4.4. Mode Selector & Safe Teleoperation (Right Panel)
* **Mode Toggle Buttons:**
  * `🛑 STOPPED` — Disarms motors and silences all motion outputs.
  * `🎮 MANUAL / TELEOP` — Arms the manual teleop channel (`/cmd_vel_joy`).
  * `🤖 AUTONOMOUS (NAV2)` — Arms the autonomous navigation channel (`/cmd_vel_nav`).
* **Keyboard Teleoperation Controls (in MANUAL mode):**
  * `W` or `▲`: Move Forward (linear velocity: $+0.08$ m/s, clamped to $\le 0.10$ m/s)
  * `S` or `▼`: Move Backward (linear velocity: $-0.08$ m/s, clamped to $\le 0.10$ m/s)
  * `A` or `◀`: Turn Left (angular velocity: $+0.20$ rad/s, clamped to $\le 0.25$ rad/s)
  * `D` or `▶`: Turn Right (angular velocity: $-0.20$ rad/s, clamped to $\le 0.25$ rad/s)
  * `SPACEBAR` or Key Release: Instant brake ($0.0$ m/s, $0.0$ rad/s).
* **Deadman Switch Watchdog:**
  * Teleoperation packets require a heartbeat every 100 ms.
  * If no key event is received within 300 ms, or if the browser window loses focus, the supervisor instantly halts robot movement.

### 4.5. Mission Ore Counting Summary Table
* Displays real-time session counters:
  * **Total Unique Ores:** Deduplicated count of confirmed targets.
  * **Resource Targets:** Count of valuable ore classifications.
  * **Host Rock Targets:** Count of non-resource stone classifications.
  * **Active Visual Tracks:** Current number of objects tracked in the camera field of view.
  * **Localized (3D):** Number of objects with verified 3D spatial coordinates in the map frame.

### 4.6. Telemetry & Hardware Diagnostics Panel
* **Camera Health:** Frame rate (FPS) and online state.
* **LiDAR Health:** Frequency (10 Hz nominal) and device health status.
* **SLAM State:** `MAPPING` (active occupancy grid) or `NO_MAP` (stationary standby).
* **Spatial Calibration:** `UNCALIBRATED` (accurately reflecting uncalibrated camera matrix).
* **Motor Gate Status:** Displays whether physical motors are armed or disarmed.

---

## 5. Alternative Startup & Operational Modes

### Running in Mock Mode (Synthetic Demonstration)
For testing on laptops without physical camera or LiDAR hardware:
```bash
./scripts/run_cogproj.sh --mode mock
```
This mode uses the built-in `MockDetector` with animated synthetic ore tracks and a simulated demo room map.

### Live System Status Inspection
To inspect running hardware, thermals, user groups, and active ROS 2 graph streams:
```bash
./scripts/run_cogproj.sh --status
# Or:
./scripts/hardware_diagnostics.sh
```

---

## 6. Clean System Shutdown

To stop all managed CogProj processes without leaving orphan ROS nodes:
```bash
./scripts/run_cogproj.sh --stop
# Or:
./scripts/stop_cogproj.sh
```
Or press `Ctrl+C` in the terminal running `run_cogproj.sh`.

The shutdown manager safely:
1. Sends zero-velocity commands to disarm rover motion.
2. Gracefully signals (`SIGINT`) all background nodes and their process trees.
3. Cleans up the runtime PID registry (`log/cogproj_runtime.pids`).
4. Preserves any pre-existing system drivers.

