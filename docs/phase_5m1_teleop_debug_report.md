# Phase 5M.1 Teleoperation Debug and Repair Report

**Project:** CogProj — Beetle Bot Autonomous Rover  
**Target Host:** `beetlebot-117` (`192.168.0.117`, Ubuntu 24.04.3 LTS ARM64, ROS 2 Jazzy)  
**Task:** Phase 5M.1 — Fix Nonfunctional Keyboard Teleoperation  
**Date:** 2026-10-14  
**Status:** **PARTIAL** (Complete browser-to-ROS command chain verified end-to-end in dry run; physical rover movement requires operator-supervised execution)

---

## 1. Executive Summary & Problem Identification

In previous stages, automated tests reported passing status, yet operator keyboard input on the laptop web console (`http://localhost:8080`) produced zero physical wheel motion on the rover.

A systematic trace of the control architecture revealed that while unit tests validated isolated components, the end-to-end integration across the safety gate, hardware motor bridge, and frontend event loop was broken by **four specific, demonstrable defects**.

---

## 2. Root Cause Analysis

### Root Cause 1: Missing `/joy` Deadman Publisher in Dashboard Gateway
- **Mechanism in `lyra_cmd_vel_gate` (`/home/veerobot/lyra_ws/src/lyra_cmd_vel_gate/lyra_cmd_vel_gate/node.py`):**
  The gate node subscribes to two topics:
  1. `/cmd_vel_joy` (`geometry_msgs/msg/Twist`)
  2. `/joy` (`sensor_msgs/msg/Joy`)
  To authorize velocity output to `/cmd_vel`, the gate requires:
  ```python
  self.deadman_pressed = joy_msg.buttons[4] == 1  # deadman_button index 4
  joystick_active = (self.deadman_pressed and time_since_joy < 0.5 and self.is_nonzero(self.last_joy_cmd))
  ```
- **Demonstrated Defect:**
  In `dashboard_gateway_node.py` (prior to this repair), `DashboardGatewayNode` published *only* to `/cmd_vel_joy`. There was **no publisher on `/joy`**. Consequently, `deadman_pressed` was always `False`, and `lyra_cmd_vel_gate` perpetually suppressed all joystick velocity commands and emitted zero velocity.

### Root Cause 2: Motor Hardware Bridge Omitted from Startup Scripts
- **Mechanism:**
  The physical rover's STM32 motor controller is driven via GPIO UART (`/dev/ttyAMA0` at 115200 baud) by `lyra_bridge` (`lyra_node`).
- **Demonstrated Defect:**
  In `scripts/run_cogproj.sh`, section 5 launched the camera driver, RPLiDAR, robot state publisher, and SLAM toolbox, but **completely omitted `lyra_bridge`**. No process was subscribed to `/cmd_vel` or providing hardware actuation services (`/lyra/arm`, `/lyra/disarm`, `/lyra/emergency_stop`).

### Root Cause 3: Hardware Arming State & Service Integration
- **Mechanism:**
  `lyra_bridge` initializes in a `DISARMED` state (`self.armed = False`). When disarmed, the motor loop continuously sends `0.0` wheel velocity to the STM32 regardless of incoming `/cmd_vel`.
- **Demonstrated Defect:**
  Switching between modes was not actively calling `/lyra/arm` on entering `MANUAL` mode or `/lyra/disarm` and `/lyra/emergency_stop` on stopping.

### Root Cause 4: Frontend Key Event Handling & Lack of Heartbeat Loop
- **Mechanism:**
  Both `lyra_cmd_vel_gate` (`joystick_timeout = 0.5s`), `lyra_bridge` (`cmd_vel_timeout_s = 0.5s`), and `ControlSupervisor` (`command_timeout_sec = 0.30s`) enforce hardware watchdogs.
- **Demonstrated Defect:**
  In `static/app.js`:
  1. Arrow keys (`ArrowUp`, `ArrowDown`, `ArrowLeft`, `ArrowRight`) were not handled.
  2. A single `keydown` event was fired when pressing a key, with `activeKeys.has(key)` preventing further events. Because no continuous heartbeat (10 Hz) was emitted while holding the key down, the backend watchdog expired within 300 ms, causing immediate stoppage.
  3. No opposing key cancellation existed (pressing forward + back produced indeterminate states).

---

## 3. Targeted Implementation Changes

### 1. `src/cogproj_dashboard/cogproj_dashboard/dashboard_gateway_node.py`
- Added `/joy` publisher (`sensor_msgs/msg/Joy`) with depth 10.
- Added `/lyra/emergency_stop` service client alongside `/lyra/arm` and `/lyra/disarm`.
- Updated `_publish_velocity(self, linear_x: float, angular_z: float, deadman: bool = False)`:
  - Publishes bounded `Twist` to `/cmd_vel_joy`.
  - Publishes `Joy` message to `/joy` with `buttons = [0, 0, 0, 0, 1 if deadman else 0, 0]` (index 4 asserted when deadman is active, de-asserted on stop/release).
- Updated `_handle_mode_changed` and `_handle_stop_requested` to safely coordinate arming, disarming, and emergency stopping.

### 2. `src/cogproj_dashboard/cogproj_dashboard/supervisor.py`
- Added `_emit_velocity(self, linear_x: float, angular_z: float, deadman: bool)` supporting backwards-compatible 2-parameter and 3-parameter callbacks.
- Emits `deadman=True` on valid clamped motion, and `deadman=False` on deadman release, mode transitions, watchdog timeouts, and emergency stops.

### 3. `src/cogproj_dashboard/static/app.js`
- Implemented `KEY_NORMALIZE` mapping both WASD and Arrow keys (`ArrowUp`, `ArrowDown`, `ArrowLeft`, `ArrowRight`).
- Added opposing key cancellation (Forward vs Back $\to 0.0$ m/s, Left vs Right $\to 0.0$ rad/s).
- Implemented continuous 10 Hz teleoperation heartbeat (`setInterval` at 100 ms) while motion keys are held.
- Added immediate safety stop on keyup, Spacebar, tab blur, or document hide.
- Bound on-screen UI buttons for touch/mouse teleoperation with matching heartbeat behavior.

### 4. `scripts/run_cogproj.sh`
- Added section `5e. STM32 Motor Hardware Bridge` to automatically inspect and launch `lyra_bridge` (`lyra_node`) with `/home/veerobot/lyra_ws/src/lyra_bridge/config/lyra_params.yaml` on `/dev/ttyAMA0`.
- Integrated process tracking and graceful SIGINT cleanup into the supervisor loop.

### 5. Automated Tests
- Updated `src/cogproj_dashboard/test/test_supervisor.py` to verify the deadman boolean flag across motion and stop states.
- Updated `src/cogproj_dashboard/test/test_dashboard_gateway.py` to verify:
  - Gateway publishes strictly to `/cmd_vel_joy` (never `/cmd_vel`).
  - `/joy` topic publisher exists with button index 4 asserted (`1`) on motion and cleared (`0`) on stop.
  - Hardware arming, disarming, and emergency stop service clients are configured.

---

## 4. End-to-End Command Routing Architecture

```
[ Laptop Browser (app.js) ]
         │
         │ WebSocket (10 Hz Heartbeat: {"type":"teleop", "linear":0.35, "angular":0.0, "deadman":true})
         ▼
[ WebServerThread / Tornado ws ]
         │
         ▼
[ ControlSupervisor (supervisor.py) ]
         │ Clamps speed: lin <= 0.35 m/s, ang <= 0.80 rad/s; enforces 0.3s watchdog
         ▼
[ DashboardGatewayNode (dashboard_gateway_node.py) ]
         ├─── /cmd_vel_joy (geometry_msgs/Twist) ───┐
         └─── /joy (sensor_msgs/Joy, buttons[4]=1) ─┼──┐
                                                    │  │
                                                    ▼  ▼
                                      [ lyra_cmd_vel_gate (node.py) ]
                                                    │
                                                    │ (Gate verifies deadman, timeout, priority)
                                                    ▼
                                            /cmd_vel (Twist)
                                                    │
                                                    ▼
                                      [ lyra_bridge (lyra_node) ]
                                                    │
                                                    │ (UART /dev/ttyAMA0, 115200 baud)
                                                    ▼
                                      [ Physical STM32 Motor Controller ]
                                                    │
                                                    ▼
                                      [ Beetle Bot Wheels Motion ]
```

> [!IMPORTANT]
> CogProj **never** publishes directly to `/cmd_vel`. Motion commands must pass through `lyra_cmd_vel_gate`, ensuring system-level safety gating is strictly maintained.

---

## 5. Verification & Test Evidence

### Automated Workspace Test Suite
Full workspace rebuild and test run via `./scripts/build_workspace.sh --test`:
```
Summary: 9 packages finished [15.0s]
build/cogproj_bringup/pytest.xml: 17 tests, 0 errors, 0 failures, 0 skipped
build/cogproj_camera/pytest.xml: 7 tests, 0 errors, 0 failures, 0 skipped
build/cogproj_counting/pytest.xml: 13 tests, 0 errors, 0 failures, 0 skipped
build/cogproj_dashboard/pytest.xml: 16 tests, 0 errors, 0 failures, 0 skipped
build/cogproj_detection/pytest.xml: 25 tests, 0 errors, 0 failures, 0 skipped
build/cogproj_detector_base/pytest.xml: 13 tests, 0 errors, 0 failures, 0 skipped
build/cogproj_spatial/pytest.xml: 17 tests, 0 errors, 0 failures, 0 skipped
build/cogproj_tracking/pytest.xml: 15 tests, 0 errors, 0 failures, 0 skipped
build/cogproj_visualization/pytest.xml: 26 tests, 0 errors, 0 failures, 0 skipped

Summary: 149 tests, 0 errors, 0 failures, 0 skipped
[✓] All tests verified green.
```

### End-to-End Dry-Run Chain Verification
Executed via `scratch/test_teleop_chain_dryrun.py` tracing live ROS 2 message propagation through `lyra_cmd_vel_gate`:
```
======================================================================
PHASE 5M.1: END-TO-END TELEOPERATION DRY-RUN VERIFICATION
======================================================================
[1] Launched lyra_cmd_vel_gate (PID 10147)
[2] Launched cogproj_dashboard dashboard_node on port 8089 (PID 10148)
[3] REST API /api/status responsive. Initial mode: STOPPED
[4] WebSocket connected to ws://127.0.0.1:8089/ws
[5] Mode transition response: {'type': 'mode_response', 'success': True, 'message': 'Mode switched from STOPPED to MANUAL', 'mode': 'MANUAL'}
[6] Sending teleop command: lin=0.35, ang=0.0, deadman=True
    • /cmd_vel_joy received 6 msgs
    • /joy received 6 msgs
    • /cmd_vel (post-gate) received 16 msgs
    [VERIFIED] /cmd_vel_joy Twist linear.x: 0.350
    [VERIFIED] /joy button[4] (deadman): 1
    [VERIFIED] /cmd_vel Gate Twist linear.x: 0.350
[7] Sending deadman release (Stop: lin=0.0, ang=0.0, deadman=False)
    [VERIFIED] /joy button[4] on release: 0
    [VERIFIED] /cmd_vel on release: linear.x=0.000
[8] Sending Emergency Stop
    [VERIFIED] Estop response: {'type': 'mode_response', 'success': True, 'message': 'Emergency Stop Engaged', 'mode': 'STOPPED'}

======================================================================
>>> ALL DRY-RUN CHAIN CHECKS PASSED WITH 100% SUCCESS! <<<
======================================================================
```

---

## 6. Controlled Physical Verification Procedure

> [!CAUTION]
> Physical movement must **never** be performed without an operator physically beside the rover. Ensure the physical emergency stop button is within reach and the test area is clear.

### Step 1: Pre-Motion Rover Elevation (Bench Test)
1. Place the rover on an elevated stand or block so all four wheels can rotate freely without touching the ground.
2. Confirm battery level is sufficient (>11.0V).

### Step 2: Launch CogProj System
On the robot (`beetlebot-117`):
```bash
cd /home/veerobot/CogProj
./scripts/run_cogproj.sh --mode manual
```

### Step 3: Establish Laptop Dashboard Connection
1. In your laptop terminal, set up SSH port forwarding:
   ```bash
   ssh -L 8080:127.0.0.1:8080 -L 8765:127.0.0.1:8765 veerobot@192.168.0.117
   ```
2. Open `http://localhost:8080` in your browser.
3. Confirm status pills indicate:
   - WebSocket: `ONLINE (WS Connected)`
   - Initial Mode: `MODE: STOPPED`
   - Sensors: Camera online, LiDAR online.

### Step 4: Step-by-Step Actuation Checks
1. **Mode Switch:** Click `🎮 MANUAL / TELEOP`. Observe mode pill changes to `MODE: MANUAL` and robot status shows `ARMED`.
2. **Forward Jog Test:** Press and hold `W` or `ArrowUp` for 1 second, then release.
   - Verify wheels turn forward at the configured brisk speed ($\le 0.35$ m/s).
   - Verify wheels stop immediately upon key release.
3. **Reverse Jog Test:** Press and hold `S` or `ArrowDown` for 1 second, then release.
   - Verify wheels rotate in reverse and stop upon release.
4. **Pivot Test:** Press and hold `A` (pivot left), then `D` (pivot right).
   - Verify differential wheel rotation and immediate stop upon release.
5. **Watchdog / Disconnect Test:** Hold `W` and immediately close the browser tab or switch to another window.
   - Verify wheels halt within 300 ms.
6. **Emergency Stop Test:** Click `🚨 EMERGENCY STOP`.
   - Verify system immediately locks to `MODE: STOPPED` and motors disarm.

---

## 7. Phase Conclusion

**Overall Assessment:** **PARTIAL**

- **Software Command Chain:** Completely diagnosed, repaired, built, and verified green through live dry-run checks and 149 passing workspace tests.
- **Physical Verification:** Standby for the operator to follow the controlled physical movement procedure above with wheels elevated.

