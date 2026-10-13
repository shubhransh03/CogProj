# Phase 5M: Control Architecture & Integration Specification

**Project:** CogProj — AI-Based Ore Detection and Counting Rover  
**Platform:** Veerobot Beetle Bot (`beetlebot-117`), Raspberry Pi 5, Ubuntu 24.04.3 LTS, ROS 2 Jazzy  
**Domain ID:** 155 | **RMW:** `rmw_fastrtps_cpp`  
**Phase:** 5M — Complete Keyboard Teleoperation and Autonomous Navigation  

---

## 1. Executive Summary

Phase 5M establishes the complete, production-grade vehicle motion and navigation control stack for CogProj. It unites operator teleoperation, autonomous Nav2 action dispatch, hardware command-velocity gating, and safety watchdog supervision into a cohesive, fail-safe architecture.

All motion commands are strictly arbitrated by a ROS-side `ControlSupervisor` and routed through the robot's existing command-velocity safety gate (`CmdVelGate`), guaranteeing zero unintended movement, hardware deadman compliance, and prompt auto-stopping on network or operator interruption.

---

## 2. Command Velocity Routing & Hardware Safety Gate

### 2.1 Lyra Hardware Architecture
The physical Beetle Bot's motor drive is managed by two lower-level components in `/home/veerobot/lyra_ws`:
1. **`lyra_bridge`**: Hardware bridge communicating with the onboard STM32 motor controller over UART (`/dev/ttyAMA0`, 115200 baud). Exposes services `/lyra/arm`, `/lyra/disarm`, `/lyra/emergency_stop` (`std_srvs/srv/Trigger`), and publishes `/lyra/armed` (`std_msgs/msg/Bool`). Enforces a 0.5s motor timeout.
2. **`lyra_cmd_vel_gate` (`CmdVelGate`)**: Software multiplexer and safety gate that subscribes to `/cmd_vel_joy`, `/cmd_vel_nav`, `/joy`, and `/lyra/armed`, and publishes to the motor driver topic `/cmd_vel`.

### 2.2 Routing Isolation
CogProj **never** writes directly to `/cmd_vel`. The control chain strictly enforces pre-gate isolation:

```
[ Operator Laptop Dashboard ]
          │ (WebSocket JSON)
          ▼
[ cogproj_dashboard: DashboardGatewayNode / ControlSupervisor ]
   │                                                    │
   │ (Manual Teleop Mode)                               │ (Autonomous Nav Mode)
   ├───────────────────────────────┐                    │
   ▼                               ▼                    ▼
/cmd_vel_joy (Twist)          /joy (Joy: btn[4]=1)  navigate_to_pose (Nav2 Action)
   │                               │                    │
   └───────────────┬───────────────┘                    ▼
                   │                           [ Nav2 Controller Server ]
                   │                                    │
                   ▼                                    ▼
       [ lyra_cmd_vel_gate: CmdVelGate ] <─────── /cmd_vel_nav (Twist)
                   │
                   ▼
               /cmd_vel (Twist)
                   │
                   ▼
          [ lyra_bridge: LyraBridge ]
                   │ (UART @ 115200)
                   ▼
        [ STM32 Motor Controller ]
```

### 2.3 Deadman Heartbeat Integration
`CmdVelGate` requires `msg.buttons[4] == 1` on topic `/joy` for joystick/teleop velocity to pass through to `/cmd_vel`. `DashboardGatewayNode` publishes:
- **During Active Teleop**: `sensor_msgs/msg/Joy` with `buttons[4] = 1` along with the clamped `geometry_msgs/msg/Twist` on `/cmd_vel_joy`.
- **Upon Key Release / Watchdog / Stop**: `buttons[4] = 0` and `Twist(linear.x=0.0, angular.z=0.0)`.

---

## 3. ControlSupervisor State Machine

Motion arbitration is governed by `ControlSupervisor` (`src/cogproj_dashboard/cogproj_dashboard/supervisor.py`).

### 3.1 Operating Modes
The system operates in three mutually exclusive modes:
1. **`STOPPED` (Default)**:
   - Initial state at launch.
   - Motors disarmed via `/lyra/disarm`.
   - All teleop and nav commands rejected.
   - Zero velocity broadcast.
2. **`MANUAL`**:
   - Keyboard teleoperation enabled.
   - Requires explicit operator motion arming (`motion_enabled = True`).
   - Velocity clamped to $\le 0.10$ m/s linear, $\le 0.25$ rad/s angular.
   - Commands gated by 300 ms watchdog.
3. **`AUTONOMOUS`**:
   - Waypoint dispatch to Nav2 `navigate_to_pose` action server.
   - Requires explicit operator motion arming (`motion_enabled = True`).
   - Commands gated by prerequisite checks and map clearance validation.

### 3.2 Operator Motion Enablement (Deliberate Arming)
To prevent accidental robot movement:
- Entering `MANUAL` or `AUTONOMOUS` mode does **not** automatically arm motors.
- The operator must explicitly toggle **"⚡ Enable Motion / Motors"** on the dashboard.
- If motion is disabled, velocity commands are rejected with a descriptive reason.
- Switching to `STOPPED` mode automatically disarms motors and disables motion.

### 3.3 Server-Side Velocity Clamping & Sanitation
Before any velocity reaches ROS topics, the server validates:
- Finiteness: Rejects `NaN`, `Inf`, and non-numeric payloads.
- Ceilings:
  $$\text{linear\_x} = \text{clamp}(v_x, -0.10, +0.10)\text{ m/s}$$
  $$\text{angular\_z} = \text{clamp}(\omega_z, -0.25, +0.25)\text{ rad/s}$$

### 3.4 300 ms Watchdog Timer
A ROS timer checks command age at 20 Hz (every 50 ms):
- If no teleop command is received within $300\text{ ms}$, the watchdog trips.
- Deadman active state is de-asserted (`buttons[4] = 0`).
- Velocity is zeroed immediately.
- Watchdog status on the dashboard transitions to `TIMEOUT`.

### 3.5 Single-Session Lease Arbitration
To prevent concurrent browser tabs or operators from issuing conflicting commands:
- The first client to command acquires an exclusive session lease (`session_id`).
- Subsequent clients can observe telemetry and maps, but their control commands are rejected with `"Rejected: another operator session holds the active control lease"`.
- Closing or disconnecting the active session immediately halts the robot and frees the lease.

---

## 4. Nav2 Autonomous Navigation Integration

### 4.1 Prerequisite Validation Matrix
Before dispatching a waypoint to `navigate_to_pose`, `DashboardGatewayNode` evaluates:

| Prerequisite | Condition | Blocker Message |
| :--- | :--- | :--- |
| **Operating Mode** | `mode == ControlMode.AUTONOMOUS` | `Operating mode is not AUTONOMOUS` |
| **Operator Arming** | `motion_enabled == True` | `Motion is disabled (Operator must Enable Motion)` |
| **LiDAR Health** | $(t_{now} - t_{scan}) < 1.5\text{ s}$ | `LiDAR /scan data is offline or stale` |
| **Map Availability** | `_latest_occupancy_grid is not None` | `No occupancy grid map received on /map` |
| **Localization** | TF `map` $\rightarrow$ `base_footprint` valid | `Robot localization in 'map' frame is unavailable` |
| **Nav2 Server** | `nav2_client.server_is_ready()` | `Nav2 navigate_to_pose action server is unavailable` |

If any check fails, the dashboard renders a prominent warning banner listing every active blocker, and the "Dispatch Nav2 Goal" button is disabled.

### 4.2 Map Coordinate & Obstacle Validation
When an operator clicks on the SLAM canvas, screen pixels $(c_x, c_y)$ are converted to map coordinates $(w_x, w_y)$:
$$w_x = \frac{c_x - \text{mapOffsetX}}{\text{mapScale}},\quad w_y = -\frac{c_y - \text{mapOffsetY}}{\text{mapScale}}$$

The server verifies:
1. **Bounds Check**:
   $$g_x = \left\lfloor\frac{w_x - \text{origin\_x}}{\text{resolution}}\right\rfloor,\quad g_y = \left\lfloor\frac{w_y - \text{origin\_y}}{\text{resolution}}\right\rfloor$$
   $$0 \le g_x < \text{width},\quad 0 \le g_y < \text{height}$$
2. **Obstacle Clearance**:
   $$\text{cost} = \text{data}[g_y \cdot \text{width} + g_x]$$
   If $\text{cost} > 50$, the waypoint is rejected (`Goal is an obstacle`).

### 4.3 Nav2 Action Lifecycle
Nav2 goal execution transitions through:
`READY` $\rightarrow$ `GOAL_PENDING` $\rightarrow$ `NAVIGATING` $\rightarrow$ `GOAL_REACHED` / `GOAL_CANCELED` / `GOAL_FAILED`.

---

## 5. Keyboard Teleoperation Specification

| Key | Action | Output ($\text{lin}$, $\text{ang}$) |
| :--- | :--- | :--- |
| **W** / **ArrowUp** | Drive Forward | $+0.10\text{ m/s}$, $0.00\text{ rad/s}$ |
| **S** / **ArrowDown** | Drive Backward | $-0.10\text{ m/s}$, $0.00\text{ rad/s}$ |
| **A** / **ArrowLeft** | Turn Left | $0.00\text{ m/s}$, $+0.25\text{ rad/s}$ |
| **D** / **ArrowRight** | Turn Right | $0.00\text{ m/s}$, $-0.25\text{ rad/s}$ |
| **W + A** | Forward-Left Arc | $+0.10\text{ m/s}$, $+0.25\text{ rad/s}$ |
| **W + S** | Opposing Forward/Back | $0.00\text{ m/s}$, $0.00\text{ rad/s}$ (Canceled) |
| **A + D** | Opposing Left/Right | $0.00\text{ m/s}$, $0.00\text{ rad/s}$ (Canceled) |
| **Spacebar** | Immediate Stop | $0.00\text{ m/s}$, $0.00\text{ rad/s}$ |
| **Tab Hide / Blur** | Focus Lost Auto-Stop | $0.00\text{ m/s}$, $0.00\text{ rad/s}$ (Deadman De-asserted) |
| **WebSocket Disconnect** | Network Interruption | Emergency Stop + Disarm |

